import serial
import struct
import threading
import time
import numpy as np
import pygame
import math
from ArduinoBridge import ArduinoLink

# ==========================================
# CONFIGURACIÓN
# ==========================================
PUERTO_LIDAR = 'COM8'
PUERTO_ARDUINO = 'COM7'
BAUD_ARDUINO = 9600

# --- GEOMETRÍA FÍSICA ---
ROBOT_LARGO_MM = 750
ROBOT_ANCHO_MM = 450
DIST_LIDAR_FRENTE_MM = 280
DIST_LIDAR_ATRAS_MM = ROBOT_LARGO_MM - DIST_LIDAR_FRENTE_MM
MITAD_ANCHO_MM = ROBOT_ANCHO_MM / 2

OFFSET_LIDAR_FISICO = 186.0
ANCHO_ROBOT_M = ROBOT_ANCHO_MM / 1000.0

# --- CONTROL ---
TARGET_CLEARANCE_M = 0.60
PWM_MIN = 60
PWM_MAX = 110
FACTOR_RESPUESTA = 150.0

TRIM_IZQUIERDA = 1.0
TRIM_DERECHA = 1.0

K_DISTANCIA = 1.0
K_ANGULO = 1.5

# --- SEGURIDAD DINÁMICA ---
DISTANCIA_SEGURIDAD_DINAMICA = 900
ANCHO_MAXIMO_OBJETO_DINAMICO = 1200
TIEMPO_PACIENCIA_DINAMICO = 5.0

# --- LIMPIEZA ---
NIVELES_LIMPIEZA = [0.15, 0.40, 1.00]

# --- RASTRO ---
UMBRAL_DENSIDAD_VISITADO = 40
TIEMPO_IGNORAR_RASTRO_RECIENTE = 3.0

# --- GRÁFICOS ---
ANCHO_VENTANA = 1100
ALTO_VENTANA = 720
ESCALA_ZOOM = 0.18

PANEL_W = 320
MAP_W = ANCHO_VENTANA - PANEL_W

FPS_OBJETIVO = 30
FPS_DIBUJO = 20
INTERVALO_DIBUJO = 1.0 / FPS_DIBUJO


# ==========================================
# LIDAR PERSISTENTE / ROBUSTO
# ==========================================
class LidarPersistence:
    def __init__(
        self,
        bin_deg=1.0,
        ttl_s=0.45,
        max_jump_mm=250,
        confirm_needed=2,
        ema_alpha=0.35,
        min_dist_mm=30,
        max_dist_mm=12000,
    ):
        self.bin_deg = float(bin_deg)
        self.nbins = int(round(360.0 / self.bin_deg))
        self.ttl_s = float(ttl_s)

        self.max_jump_mm = float(max_jump_mm)
        self.confirm_needed = int(confirm_needed)
        self.ema_alpha = float(ema_alpha)

        self.min_dist_mm = float(min_dist_mm)
        self.max_dist_mm = float(max_dist_mm)

        self.dist = np.full(self.nbins, np.nan, dtype=np.float32)
        self.ts = np.full(self.nbins, 0.0, dtype=np.float64)

        self.pending_dist = np.full(self.nbins, np.nan, dtype=np.float32)
        self.pending_cnt = np.zeros(self.nbins, dtype=np.int16)
        self.pending_ts = np.full(self.nbins, 0.0, dtype=np.float64)

    def _bin_index(self, ang_deg):
        return int(ang_deg / self.bin_deg) % self.nbins

    def update(self, angles_deg, distances_mm, now=None):
        if now is None:
            now = time.time()
        if len(distances_mm) == 0:
            return

        d = distances_mm.astype(np.float32)
        a = angles_deg.astype(np.float32)

        mask = (d >= self.min_dist_mm) & (d <= self.max_dist_mm)
        if not np.any(mask):
            return

        d = d[mask]
        a = a[mask]

        for ang, dist_new in zip(a, d):
            ang = float(ang) % 360.0
            idx = self._bin_index(ang)

            dist_old = self.dist[idx]
            ts_old = self.ts[idx]
            is_old_valid = (not np.isnan(dist_old)) and ((now - ts_old) <= self.ttl_s)

            if not is_old_valid:
                self.dist[idx] = dist_new
                self.ts[idx] = now
                self.pending_cnt[idx] = 0
                self.pending_dist[idx] = np.nan
                self.pending_ts[idx] = 0.0
                continue

            jump = abs(dist_new - dist_old)

            if jump <= self.max_jump_mm:
                self.dist[idx] = (1.0 - self.ema_alpha) * dist_old + self.ema_alpha * dist_new
                self.ts[idx] = now
                self.pending_cnt[idx] = 0
                self.pending_dist[idx] = np.nan
                self.pending_ts[idx] = 0.0
            else:
                pd = self.pending_dist[idx]
                pc = self.pending_cnt[idx]
                pts = self.pending_ts[idx]

                if pc > 0 and (now - pts) > 0.20:
                    pc = 0
                    pd = np.nan

                if pc == 0:
                    self.pending_dist[idx] = dist_new
                    self.pending_cnt[idx] = 1
                    self.pending_ts[idx] = now
                else:
                    if not np.isnan(pd) and abs(dist_new - pd) <= self.max_jump_mm:
                        pc += 1
                        self.pending_cnt[idx] = pc
                        self.pending_ts[idx] = now
                        if pc >= self.confirm_needed:
                            self.dist[idx] = (1.0 - self.ema_alpha) * dist_old + self.ema_alpha * dist_new
                            self.ts[idx] = now
                            self.pending_cnt[idx] = 0
                            self.pending_dist[idx] = np.nan
                            self.pending_ts[idx] = 0.0
                    else:
                        self.pending_dist[idx] = dist_new
                        self.pending_cnt[idx] = 1
                        self.pending_ts[idx] = now

    def get_scan(self, now=None):
        if now is None:
            now = time.time()
        age = now - self.ts
        mask = (age <= self.ttl_s) & (~np.isnan(self.dist))
        if not np.any(mask):
            return np.empty(0, dtype=np.float32), np.empty(0, dtype=np.float32)

        idxs = np.where(mask)[0]
        angles = (idxs.astype(np.float32) * self.bin_deg) % 360.0
        dists = self.dist[idxs].astype(np.float32)
        return angles, dists


# ==========================================
# GESTOR DE RASTRO OPTIMIZADO
# ==========================================
class TrailManager:
    def __init__(self):
        self.points = np.empty((0, 3), dtype=np.float32)
        self.max_points = 9000
        self.last_add_time = 0.0

    def update(self, speed_mm_s, ang_vel_rad_s, dt):
        if self.points.shape[0] == 0:
            return

        desplazamiento = speed_mm_s * dt
        rot = -ang_vel_rad_s * dt
        cos_r = math.cos(rot)
        sin_r = math.sin(rot)

        x = self.points[:, 0]
        y = self.points[:, 1]

        x_new = x * cos_r - y * sin_r
        y_new = x * sin_r + y * cos_r + desplazamiento

        self.points[:, 0] = x_new
        self.points[:, 1] = y_new

        dist_sq = self.points[:, 0] ** 2 + self.points[:, 1] ** 2
        self.points = self.points[dist_sq < 225000000]

    def add_breadcrumb(self):
        now = time.time()
        if now - self.last_add_time < 0.14:
            return

        self.last_add_time = now

        num_samples = 8
        xs = np.random.uniform(-MITAD_ANCHO_MM, MITAD_ANCHO_MM, num_samples).astype(np.float32)
        ys = np.random.uniform(-DIST_LIDAR_FRENTE_MM, 0, num_samples).astype(np.float32)
        ts = np.full(num_samples, now, dtype=np.float32)

        new_batch = np.column_stack((xs, ys, ts))
        self.points = np.vstack((self.points, new_batch))

        if self.points.shape[0] > self.max_points:
            self.points = self.points[-self.max_points:]

    def draw(self, screen, cx, cy, visual_rotation_rad):
        if self.points.shape[0] == 0:
            return

        cos_v = math.cos(-visual_rotation_rad)
        sin_v = math.sin(-visual_rotation_rad)

        draw_pts = self.points[::4]
        rx = draw_pts[:, 0] * cos_v - draw_pts[:, 1] * sin_v
        ry = draw_pts[:, 0] * sin_v + draw_pts[:, 1] * cos_v

        screen_xs = (cx + (rx * ESCALA_ZOOM)).astype(np.int32)
        screen_ys = (cy + (ry * ESCALA_ZOOM)).astype(np.int32)

        surf = pygame.Surface((MAP_W, ALTO_VENTANA), pygame.SRCALPHA)

        for sx, sy in zip(screen_xs, screen_ys):
            if 0 <= sx < MAP_W and 0 <= sy < ALTO_VENTANA:
                surf.set_at((sx, sy), (255, 255, 255, 40))

        screen.blit(surf, (0, 0))

    def get_density_under_robot(self):
        if self.points.shape[0] == 0:
            return 0

        now = time.time()
        mask_old = (now - self.points[:, 2]) > TIEMPO_IGNORAR_RASTRO_RECIENTE
        old_points = self.points[mask_old]
        if old_points.shape[0] == 0:
            return 0

        dist_sq = old_points[:, 0] ** 2 + old_points[:, 1] ** 2
        return int(np.sum(dist_sq < (300 ** 2)))


# ==========================================
# DRIVER LiDAR
# ==========================================
class LiDAR_LD20:
    def __init__(self, port, baudrate=230400):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        self.buffer_puntos = []
        self.lock = threading.Lock()

    def connect(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            self.running = True
            return True
        except Exception as e:
            print(f"No se pudo abrir LiDAR: {e}")
            return False

    def read_loop(self):
        buffer = b''
        while self.running:
            try:
                if self.serial.in_waiting:
                    buffer += self.serial.read(self.serial.in_waiting)
                else:
                    time.sleep(0.002)
                    continue

                if len(buffer) > 4000:
                    buffer = buffer[-2000:]

                while len(buffer) >= 47:
                    idx = buffer.find(b'\x54')
                    if idx == -1:
                        buffer = b''
                        break
                    if idx > 0:
                        buffer = buffer[idx:]
                    if len(buffer) < 47:
                        break

                    packet = buffer[:47]
                    buffer = buffer[47:]

                    if packet[1] != 0x2C:
                        continue

                    start_angle = struct.unpack('<H', packet[4:6])[0] / 100.0
                    end_angle = struct.unpack('<H', packet[42:44])[0] / 100.0
                    if end_angle < start_angle:
                        end_angle += 360
                    step = (end_angle - start_angle) / 11.0

                    nuevos = []
                    for i in range(12):
                        dist = struct.unpack('<H', packet[6 + (i * 3):8 + (i * 3)])[0]
                        if dist > 0:
                            ang = start_angle + step * i
                            if ang >= 360:
                                ang -= 360
                            nuevos.append((ang, dist))

                    if nuevos:
                        with self.lock:
                            self.buffer_puntos.extend(nuevos)

            except Exception:
                self.running = False

    def obtener_datos_nuevos(self):
        with self.lock:
            if not self.buffer_puntos:
                return np.empty(0, dtype=np.float32), np.empty(0, dtype=np.float32)
            datos = self.buffer_puntos[:]
            self.buffer_puntos.clear()

        angulos, distancias = zip(*datos)
        return np.array(angulos, dtype=np.float32), np.array(distancias, dtype=np.float32)

    def close(self):
        self.running = False
        try:
            if self.serial:
                self.serial.close()
        except Exception:
            pass


# ==========================================
# ARDUINO BRIDGE CON YAW INCREMENTAL ACUMULADO
# ==========================================
class ArduinoBridgeController:
    """
    El Arduino NO manda yaw absoluto.
    Manda incrementos de yaw en grados:
        DANG:<delta_deg>

    Python acumula ese valor y conserva la orientación aunque el Arduino
    se reinicie o se reconecte.
    """
    def __init__(self, port=None, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.link = None
        self.lock = threading.Lock()

        self.connected = False

        # yaw acumulado global que vive en Python
        self.accumulated_yaw_deg = 0.0

        # métricas / estado
        self.last_rx_time = 0.0
        self.last_conn_state = False
        self.last_command = "CMD:0,0,0,0"

    def connect(self):
        try:
            self.link = ArduinoLink(port=self.port, baud=self.baudrate)

            @self.link.on_message("DANG")
            def _on_dang(msg):
                vals = msg.as_floats()
                if vals:
                    self._apply_increment(vals[0])

            @self.link.on_message("GYRO")
            def _on_gyro(msg):
                vals = msg.as_floats()
                if vals:
                    self._apply_increment(vals[0])

            @self.link.on_message("DGY")
            def _on_dgy(msg):
                vals = msg.as_floats()
                if vals:
                    self._apply_increment(vals[0])

            @self.link.on_message("*")
            def _on_any(msg):
                self.last_rx_time = time.time()

            self.link.start()

            t0 = time.time()
            while time.time() - t0 < 3.5:
                if self.link.connected:
                    self.connected = True
                    self.last_conn_state = True
                    return True
                time.sleep(0.05)

            self.connected = bool(self.link.connected)
            self.last_conn_state = self.connected
            return self.connected

        except Exception as e:
            print(f"Error conectando ArduinoBridge: {e}")
            self.connected = False
            self.last_conn_state = False
            return False

    def _apply_increment(self, delta_deg):
        with self.lock:
            self.accumulated_yaw_deg += float(delta_deg)
            self.last_rx_time = time.time()

    def get_yaw(self):
        with self.lock:
            return self.accumulated_yaw_deg

    def is_connected(self):
        state = bool(self.link and self.link.connected)
        self.connected = state
        return state

    def send_command(self, izq, der, aux_pwm, comp_state):
        if not self.link or not self.link.connected:
            return

        try:
            izq = izq * TRIM_IZQUIERDA
            der = der * TRIM_DERECHA

            izq = max(min(int(izq), 255), -255)
            der = max(min(int(der), 255), -255)
            aux_pwm = max(min(int(aux_pwm), 255), 0)
            comp = 1 if comp_state else 0

            cmd = f"CMD:{izq},{der},{aux_pwm},{comp}"
            self.last_command = cmd
            self.link.send(cmd)
        except Exception:
            pass

    def close(self):
        try:
            if self.link and self.link.connected:
                self.link.send("CMD:0,0,0,0")
        except Exception:
            pass


# ==========================================
# LÓGICA DE DETECCIÓN Y CONTROL
# ==========================================
def detectar_intrusos_dinamicos(angulos, distancias, offset_correccion):
    if len(distancias) == 0:
        return False, 0, 0

    ang_corr = np.mod(angulos - OFFSET_LIDAR_FISICO - offset_correccion + 180, 360) - 180
    rads = np.radians(ang_corr)
    xs = distancias * np.cos(rads)
    ys = distancias * np.sin(rads)

    mask = distancias < 2000
    if not np.any(mask):
        return False, 0, 0

    xs = xs[mask]
    ys = ys[mask]
    ds = distancias[mask]
    aa = ang_corr[mask]

    if len(ds) < 3:
        return False, 0, 0

    clusters = []
    current_idx = [0]

    for i in range(1, len(ds)):
        if math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]) < 300:
            current_idx.append(i)
        else:
            clusters.append(current_idx)
            current_idx = [i]
    clusters.append(current_idx)

    for cl in clusters:
        if len(cl) < 3:
            continue

        xcl = xs[cl]
        ycl = ys[cl]
        dcl = ds[cl]
        acl = aa[cl]

        w = math.hypot(float(np.max(xcl) - np.min(xcl)), float(np.max(ycl) - np.min(ycl)))
        min_d = float(np.min(dcl))
        if w < ANCHO_MAXIMO_OBJETO_DINAMICO and min_d < DISTANCIA_SEGURIDAD_DINAMICA:
            return True, min_d, float(np.mean(acl))

    return False, 0, 0


def calcular_margen_real(distancia_lidar, angulo_relativo_deg):
    theta = math.radians(angulo_relativo_deg)
    rx = math.cos(theta)
    ry = math.sin(theta)

    df = DIST_LIDAR_FRENTE_MM / rx if rx > 0 else 9e9
    db = -DIST_LIDAR_ATRAS_MM / rx if rx < 0 else 9e9
    dl = MITAD_ANCHO_MM / ry if ry > 0 else 9e9
    dr = -MITAD_ANCHO_MM / ry if ry < 0 else 9e9

    return float(distancia_lidar) - min(abs(df), abs(db), abs(dl), abs(dr))


def analizar_sectores_reales(angulos, distancias, offset_correccion):
    s = {'FRENTE': 9e9, 'ATRAS': 9e9, 'IZQ': 9e9, 'DER': 9e9}
    if len(distancias) == 0:
        return s

    ang_corr = np.mod(angulos - OFFSET_LIDAR_FISICO - offset_correccion + 180, 360) - 180

    for i in range(len(ang_corr)):
        m = calcular_margen_real(distancias[i], ang_corr[i])
        a = ang_corr[i]
        if abs(a) < 30:
            s['FRENTE'] = min(s['FRENTE'], m)
        elif abs(a) > 150:
            s['ATRAS'] = min(s['ATRAS'], m)
        elif 60 < a < 120:
            s['IZQ'] = min(s['IZQ'], m)
        elif -120 < a < -60:
            s['DER'] = min(s['DER'], m)

    return s


def obtener_referencia_navegacion(angulos, distancias, offset_correccion):
    if len(distancias) == 0:
        return None, None

    ang_corr = np.mod(angulos - OFFSET_LIDAR_FISICO - offset_correccion + 180, 360) - 180

    best_m = 9e9
    best_i = -1

    step = 5 if len(distancias) > 50 else 1
    for i in range(0, len(distancias), step):
        m = calcular_margen_real(distancias[i], ang_corr[i])
        if m < best_m:
            best_m = m
            best_i = i

    if best_i != -1:
        return best_m / 1000.0, np.radians(float(ang_corr[best_i]))

    return None, None


def calcular_control_reactivo(margen_m, ang_rad):
    err = margen_m - TARGET_CLEARANCE_M
    side = "left" if ang_rad > 0 else "right"
    delta = abs(ang_rad) - (math.pi / 2) if side == "left" else (math.pi / 2) - abs(ang_rad)

    if side == "right":
        err *= -1

    va = float(np.clip((K_DISTANCIA * err) + (K_ANGULO * delta), -1.2, 1.2))
    vl = 1.0 * max(min(1 / abs(va + 0.01), 1.0), 0.15)
    return vl, va


def cinematica_diferencial(vl, va):
    def apply(v):
        return (v * FACTOR_RESPUESTA) + (np.sign(v) * PWM_MIN) if abs(v) > 0.05 else 0

    izq = np.clip(apply(vl - (va * ANCHO_ROBOT_M / 2)), -PWM_MAX, PWM_MAX)
    der = np.clip(apply(vl + (va * ANCHO_ROBOT_M / 2)), -PWM_MAX, PWM_MAX)
    return float(izq), float(der)


def obtener_esquinas_robot(cx, cy, ang):
    f = DIST_LIDAR_FRENTE_MM * ESCALA_ZOOM
    b = DIST_LIDAR_ATRAS_MM * ESCALA_ZOOM
    w = (ROBOT_ANCHO_MM / 2) * ESCALA_ZOOM
    pts = [(-w, -f), (w, -f), (w, b), (-w, b)]
    rad = math.radians(-ang)
    c = math.cos(rad)
    s = math.sin(rad)
    return [(cx + x * c - y * s, cy + x * s + y * c) for x, y in pts]


# ==========================================
# UI
# ==========================================
def draw_panel(screen, font, small, state):
    x0 = MAP_W
    pygame.draw.rect(screen, (18, 18, 28), (x0, 0, PANEL_W, ALTO_VENTANA))
    pygame.draw.line(screen, (50, 60, 80), (x0, 0), (x0, ALTO_VENTANA), 2)

    def txt(y, s, color=(220, 230, 245)):
        screen.blit(font.render(s, True, color), (x0 + 16, y))

    def txts(y, s, color=(180, 190, 210)):
        screen.blit(small.render(s, True, color), (x0 + 16, y))

    txt(14, "ROBOT CLEANER PRO", (255, 210, 120))
    txts(44, f"LiDAR: {state['lidar_port']}   Arduino: {state['arduino_port']}")
    pygame.draw.line(screen, (50, 60, 80), (x0 + 16, 70), (x0 + PANEL_W - 16, 70), 1)

    st_color = (120, 255, 140) if state["arduino_ok"] else (255, 150, 100)
    txt(86, f"ESTADO: {state['st']}", st_color)
    txts(116, f"Yaw acum: {state['yaw']:.2f}°")
    txts(140, f"Off: {state['rot_off']:.1f}°   Fix90: {state['fix_90']}°")
    txts(164, f"PWM L/R: {int(state['pl'])} / {int(state['pr'])}")
    txts(188, f"FPS: {state['fps']:.1f}")

    pygame.draw.line(screen, (50, 60, 80), (x0 + 16, 214), (x0 + PANEL_W - 16, 214), 1)

    txt(230, "LIMPIEZA", (220, 230, 245))
    ptxt = state['pump_txt']
    col = (0, 200, 255) if "ON" in ptxt else (190, 190, 210)
    txts(258, ptxt, col)
    txts(282, f"Compresor: {'ON' if state['comp_on'] else 'OFF'}")
    txts(306, f"Potencia: {int(state['power'] * 100)}%")

    pygame.draw.line(screen, (50, 60, 80), (x0 + 16, 334), (x0 + PANEL_W - 16, 334), 1)

    txt(350, "SECTORES (margen mm)", (220, 230, 245))
    sec = state['sec']
    txts(378, f"Frente: {int(sec['FRENTE']) if sec['FRENTE'] < 9e8 else '-'}")
    txts(402, f"Atras : {int(sec['ATRAS']) if sec['ATRAS'] < 9e8 else '-'}")
    txts(426, f"Izq   : {int(sec['IZQ']) if sec['IZQ'] < 9e8 else '-'}")
    txts(450, f"Der   : {int(sec['DER']) if sec['DER'] < 9e8 else '-'}")

    pygame.draw.line(screen, (50, 60, 80), (x0 + 16, 478), (x0 + PANEL_W - 16, 478), 1)

    txt(494, "COMUNICACION", (220, 230, 245))
    txts(522, f"Arduino: {'CONECTADO' if state['arduino_ok'] else 'DESCONECTADO'}")
    txts(546, f"Ult RX: {state['rx_age_txt']}")
    txts(570, f"Ult CMD: {state['last_cmd']}")

    pygame.draw.line(screen, (50, 60, 80), (x0 + 16, 598), (x0 + PANEL_W - 16, 598), 1)

    txt(614, "TECLAS", (220, 230, 245))
    tips = [
        "SPACE: AUTO/MANUAL",
        "WASD: Manual",
        "M: Spiral",
        "R: Toggle limpieza",
        "O: Toggle compresor",
        "V: Nivel potencia",
        "Q/E: Rot offset",
        "T: Fix 90°",
    ]
    y = 642
    for t in tips:
        screen.blit(small.render(t, True, (170, 180, 200)), (x0 + 16, y))
        y += 18


# ==========================================
# MAIN
# ==========================================
def main():
    pygame.init()
    screen = pygame.display.set_mode((ANCHO_VENTANA, ALTO_VENTANA))
    pygame.display.set_caption("ROBOT CLEANER PRO (Bridge + Yaw incremental)")
    clock = pygame.time.Clock()

    font = pygame.font.SysFont("Arial", 18, bold=True)
    small = pygame.font.SysFont("Consolas", 14, bold=False)

    lidar = LiDAR_LD20(PUERTO_LIDAR)
    arduino = ArduinoBridgeController(PUERTO_ARDUINO, BAUD_ARDUINO)

    if not lidar.connect():
        print("No se pudo conectar al LiDAR.")
        pygame.quit()
        return

    if not arduino.connect():
        print("ArduinoBridge no conectado al inicio. El programa seguirá intentando vía el bridge.")

    threading.Thread(target=lidar.read_loop, daemon=True).start()

    trail = TrailManager()

    lidar_mem = LidarPersistence(
        bin_deg=1.0,
        ttl_s=0.45,
        max_jump_mm=250,
        confirm_needed=2,
        ema_alpha=0.35,
    )

    st = 'MANUAL'
    pl, pr = 0.0, 0.0
    cx = 170
    cy = ALTO_VENTANA - 170
    rot_off = 0.0
    fix_90 = 0

    clean_sys = False
    comp_on = False
    pwr_idx = 0

    t_maniobra = 0.0
    yaw_spin = 0.0
    t_spiral = 0.0
    last_yaw = 0.0
    t_patience = 0.0
    evade_dir = 0

    last_render_time = 0.0
    cached_keys = None

    background = pygame.Surface((MAP_W, ALTO_VENTANA))
    background.fill((10, 10, 20))
    for gx in range(0, MAP_W, 60):
        pygame.draw.line(background, (18, 18, 28), (gx, 0), (gx, ALTO_VENTANA), 1)
    for gy in range(0, ALTO_VENTANA, 60):
        pygame.draw.line(background, (18, 18, 28), (0, gy), (MAP_W, gy), 1)

    try:
        while True:
            dt = clock.tick(FPS_OBJETIVO) / 1000.0
            if dt <= 1e-6:
                dt = 1e-3

            now = time.time()

            # ---------------------------
            # EVENTOS
            # ---------------------------
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    raise KeyboardInterrupt

                if e.type == pygame.KEYDOWN:
                    if e.key == pygame.K_SPACE:
                        st = 'AUTO_NAV' if st == 'MANUAL' else 'MANUAL'

                    elif e.key == pygame.K_m:
                        if 'SPIRAL' not in st:
                            st = 'SPIRAL_READY'
                            clean_sys = True
                            comp_on = True
                        elif st == 'SPIRAL_READY':
                            st = 'SPIRAL_RUN'
                            t_spiral = now
                        else:
                            st = 'MANUAL'

                    elif e.key == pygame.K_r:
                        clean_sys = not clean_sys

                    elif e.key == pygame.K_o:
                        comp_on = not comp_on

                    elif e.key == pygame.K_v:
                        pwr_idx = (pwr_idx + 1) % len(NIVELES_LIMPIEZA)

                    elif e.key == pygame.K_t:
                        fix_90 = (fix_90 + 90) % 360

            cached_keys = pygame.key.get_pressed()

            if cached_keys[pygame.K_q]:
                rot_off += 1.0
            if cached_keys[pygame.K_e]:
                rot_off -= 1.0

            if cached_keys[pygame.K_w] or cached_keys[pygame.K_s] or cached_keys[pygame.K_a] or cached_keys[pygame.K_d]:
                st = 'MANUAL'

            # ---------------------------
            # SENSORES
            # ---------------------------
            ang_new, dist_new = lidar.obtener_datos_nuevos()
            curr_yaw = arduino.get_yaw()

            if len(dist_new) > 0:
                lidar_mem.update(ang_new, dist_new, now=now)

            ang, dist = lidar_mem.get_scan(now=now)

            # ---------------------------
            # ODOMETRIA / RASTRO
            # ---------------------------
            speed = ((pl + pr) / 200.0) * 300.0
            dy = curr_yaw - last_yaw
            last_yaw = curr_yaw

            trail.update(speed, math.radians(dy) / dt, dt)
            if clean_sys and speed > 10:
                trail.add_breadcrumb()

            # ---------------------------
            # ANALISIS
            # ---------------------------
            sec = analizar_sectores_reales(ang, dist, fix_90)
            intruder, dist_intr, ang_intr = detectar_intrusos_dinamicos(ang, dist, fix_90)

            if intruder and st not in ['MANUAL', 'WAIT_DYN', 'EVADE_DYN']:
                st = 'WAIT_DYN'
                t_patience = now

            # ---------------------------
            # CONTROL DE ESTADO
            # ---------------------------
            if st == 'MANUAL':
                pl, pr = 0.0, 0.0

                if cached_keys[pygame.K_w]:
                    pl, pr = 60.0, 60.0
                if cached_keys[pygame.K_s]:
                    pl, pr = -60.0, -60.0
                if cached_keys[pygame.K_a]:
                    pl, pr = 60.0, -60.0
                if cached_keys[pygame.K_d]:
                    pl, pr = -60.0, 60.0

            elif st == 'WAIT_DYN':
                pl, pr = 0.0, 0.0
                if not intruder:
                    st = 'AUTO_NAV'
                elif now - t_patience > TIEMPO_PACIENCIA_DINAMICO:
                    st = 'EVADE_DYN'
                    evade_dir = -1 if ang_intr > 0 else 1

            elif st == 'EVADE_DYN':
                pl, pr = 65.0 * evade_dir, -65.0 * evade_dir
                if not intruder and sec['FRENTE'] > (ROBOT_LARGO_MM + 200):
                    st = 'AUTO_NAV'

            elif st == 'SPIRAL_READY':
                pl, pr = 0.0, 0.0

            elif st == 'SPIRAL_RUN':
                if sec['FRENTE'] < 200:
                    st = 'AUTO_NAV'
                else:
                    te = now - t_spiral
                    va = 300.0 / (te + 2.0)
                    va = max(10.0, va)
                    pl = 70.0 + va
                    pr = 70.0 - (va * 0.5)

            elif st == 'AUTO_NAV':
                if sec['FRENTE'] < 150 or sec['IZQ'] < 150 or sec['DER'] < 150:
                    st = 'ESC_REV'
                    t_maniobra = now
                else:
                    mr, arad = obtener_referencia_navegacion(ang, dist, fix_90)
                    if mr is not None:
                        vl, va = calcular_control_reactivo(mr, arad)
                        pl, pr = cinematica_diferencial(vl, va)
                    else:
                        pl, pr = 0.0, 0.0

            elif st == 'ESC_REV':
                if sec['ATRAS'] < 250:
                    st = 'ESC_SPIN'
                    yaw_spin = curr_yaw
                elif now - t_maniobra < 2.0:
                    pl, pr = -60.0, -60.0
                else:
                    st = 'ESC_SPIN'
                    yaw_spin = curr_yaw

            elif st == 'ESC_SPIN':
                pl, pr = 65.0, -65.0
                if sec['FRENTE'] > (ROBOT_LARGO_MM + 200):
                    st = 'AUTO_NAV'
                if abs(curr_yaw - yaw_spin) > 400:
                    st = 'ESC_REV'
                    t_maniobra = now

            # ---------------------------
            # ACTUADORES SMART
            # ---------------------------
            rev = (pl < -10 and pr < -10)
            dens = trail.get_density_under_robot()
            clean_already = dens > UMBRAL_DENSIDAD_VISITADO

            pump = clean_sys and (not rev) and (not clean_already) and st not in ['WAIT_DYN', 'EVADE_DYN']
            pwm_pump = int(255 * NIVELES_LIMPIEZA[pwr_idx]) if pump else 0

            arduino.send_command(pl, pr, pwm_pump, comp_on)

            # ---------------------------
            # TEXTO ESTADO BOMBA
            # ---------------------------
            if not clean_sys:
                ptxt = "OFF"
            else:
                if rev:
                    ptxt = "PAUSA (Reversa)"
                elif clean_already:
                    ptxt = "AHORRO (Zona Limpia)"
                elif st in ['WAIT_DYN', 'EVADE_DYN']:
                    ptxt = "PAUSA (Intruso)"
                else:
                    ptxt = f"ON ({int(NIVELES_LIMPIEZA[pwr_idx] * 100)}%)"

            # ---------------------------
            # DIBUJO LIMITADO
            # ---------------------------
            if (now - last_render_time) >= INTERVALO_DIBUJO:
                last_render_time = now
                vis_rot = curr_yaw + rot_off

                screen.blit(background, (0, 0))

                trail.draw(screen, cx, cy, math.radians(vis_rot))

                esq = obtener_esquinas_robot(cx, cy, vis_rot)
                pygame.draw.polygon(screen, (255, 200, 0), esq, 2)
                pygame.draw.line(screen, (255, 80, 80), esq[0], esq[1], 4)
                pygame.draw.circle(screen, (0, 200, 255), (cx, cy), 6)

                if st == 'WAIT_DYN':
                    pygame.draw.circle(
                        screen,
                        (255, 60, 60),
                        (cx, cy),
                        int(DISTANCIA_SEGURIDAD_DINAMICA * ESCALA_ZOOM),
                        2
                    )

                if len(ang) > 0:
                    ar = np.radians(ang - OFFSET_LIDAR_FISICO - fix_90 - 90 - vis_rot)
                    xd = (cx + dist * ESCALA_ZOOM * np.cos(ar)).astype(np.int32)
                    yd = (cy + dist * ESCALA_ZOOM * np.sin(ar)).astype(np.int32)

                    step = max(2, len(ang) // 260)
                    for i in range(0, len(ang), step):
                        xpi = xd[i]
                        ypi = yd[i]
                        if 0 <= xpi < MAP_W and 0 <= ypi < ALTO_VENTANA:
                            pygame.draw.circle(screen, (0, 255, 120), (xpi, ypi), 2)

                rx_age = now - arduino.last_rx_time if arduino.last_rx_time > 0 else None
                rx_age_txt = f"{rx_age:.2f}s" if rx_age is not None else "--"

                state = {
                    "st": st,
                    "yaw": float(curr_yaw),
                    "rot_off": float(rot_off),
                    "fix_90": int(fix_90),
                    "pl": float(pl),
                    "pr": float(pr),
                    "fps": float(clock.get_fps()),
                    "pump_txt": ptxt,
                    "comp_on": bool(comp_on),
                    "power": float(NIVELES_LIMPIEZA[pwr_idx]),
                    "sec": sec,
                    "arduino_ok": arduino.is_connected(),
                    "rx_age_txt": rx_age_txt,
                    "last_cmd": arduino.last_command[:28],
                    "lidar_port": PUERTO_LIDAR,
                    "arduino_port": PUERTO_ARDUINO,
                }

                draw_panel(screen, font, small, state)
                pygame.display.flip()

    except KeyboardInterrupt:
        pass

    finally:
        try:
            arduino.send_command(0, 0, 0, 0)
        except Exception:
            pass

        try:
            lidar.close()
        except Exception:
            pass

        try:
            arduino.close()
        except Exception:
            pass

        pygame.quit()


if __name__ == "__main__":
    main()