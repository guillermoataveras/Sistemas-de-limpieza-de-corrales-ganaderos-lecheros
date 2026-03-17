import serial
import struct
import threading
import time
import numpy as np
import pygame
import math
import warnings
from sklearn.exceptions import UndefinedMetricWarning

# ==========================================
# CONFIGURACIÓN
# ==========================================
PUERTO_LIDAR = 'COM5'
PUERTO_ARDUINO = 'COM6'
BAUD_ARDUINO = 115200

# --- GEOMETRÍA FÍSICA ---
ROBOT_LARGO_MM = 750
ROBOT_ANCHO_MM = 450
DIST_LIDAR_FRENTE_MM = 280
DIST_LIDAR_ATRAS_MM = ROBOT_LARGO_MM - DIST_LIDAR_FRENTE_MM
MITAD_ANCHO_MM = ROBOT_ANCHO_MM / 2

OFFSET_LIDAR_FÍSICO = 186.0
ANCHO_ROBOT_M = ROBOT_ANCHO_MM / 1000.0

# --- CONTROL ---
TARGET_CLEARANCE_M = 0.60
PWM_MIN = 60
PWM_MAX = 110
FACTOR_RESPUESTA = 150.0

TRIM_IZQUIERDA = 1.0
TRIM_DERECHA   = 1.0

K_DISTANCIA = 1.0
K_ANGULO    = 1.5

# --- SEGURIDAD DINÁMICA ---
DISTANCIA_SEGURIDAD_DINAMICA = 900
ANCHO_MAXIMO_OBJETO_DINAMICO = 1200
TIEMPO_PACIENCIA_DINAMICO = 5.0

NIVELES_LIMPIEZA = [0.15, 0.40, 1.00]

# --- RASTRO ---
UMBRAL_DENSIDAD_VISITADO = 40
TIEMPO_IGNORAR_RASTRO_RECIENTE = 3.0

# GRÁFICOS
ANCHO_VENTANA = 1100        # más ancho para panel
ALTO_VENTANA = 720
ESCALA_ZOOM = 0.18

# Panel UI
PANEL_W = 320
MAP_W = ANCHO_VENTANA - PANEL_W

warnings.filterwarnings("ignore", category=UndefinedMetricWarning)

# ==========================================
# LiDAR PERSISTENTE / ROBUSTO
# ==========================================
class LidarPersistence:
    """
    Mantiene un scan persistente en bins de ángulo (por defecto 1°).
    - TTL: cuánto tiempo un bin se considera válido sin refrescarse.
    - Outlier rejection: si un nuevo punto se aleja demasiado del valor actual,
      no lo acepta de inmediato; requiere repetición/confirmación.
    """
    def __init__(
        self,
        bin_deg=1.0,
        ttl_s=0.45,                # paredes "persisten" ~0.45s si hay dropout
        max_jump_mm=250,           # salto permitido para aceptar sin confirmar
        confirm_needed=2,          # repeticiones para aceptar un salto grande
        ema_alpha=0.35,            # suavizado
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

        # estado por bin
        self.dist = np.full(self.nbins, np.nan, dtype=np.float32)
        self.ts   = np.full(self.nbins, 0.0, dtype=np.float64)

        # para confirmar outliers
        self.pending_dist = np.full(self.nbins, np.nan, dtype=np.float32)
        self.pending_cnt  = np.zeros(self.nbins, dtype=np.int16)
        self.pending_ts   = np.full(self.nbins, 0.0, dtype=np.float64)

    def _bin_index(self, ang_deg):
        # ang_deg en [0,360)
        i = int(ang_deg / self.bin_deg) % self.nbins
        return i

    def update(self, angles_deg, distances_mm, now=None):
        if now is None:
            now = time.time()
        if len(distances_mm) == 0:
            return

        # Filtrado básico
        d = distances_mm.astype(np.float32)
        a = angles_deg.astype(np.float32)

        mask = (d >= self.min_dist_mm) & (d <= self.max_dist_mm)
        if not np.any(mask):
            return
        d = d[mask]
        a = a[mask]

        # Para reducir ruido, si llegan muchos puntos, muestreamos ligeramente
        # (pero ojo: ya vienes “cada paquete” con 12 puntos)
        for ang, dist_new in zip(a, d):
            ang = float(ang) % 360.0
            idx = self._bin_index(ang)

            dist_old = self.dist[idx]
            ts_old = self.ts[idx]
            is_old_valid = (not np.isnan(dist_old)) and ((now - ts_old) <= self.ttl_s)

            if not is_old_valid:
                # Si no hay valor vigente, aceptar directo (y limpiar pending)
                self.dist[idx] = dist_new
                self.ts[idx] = now
                self.pending_cnt[idx] = 0
                self.pending_dist[idx] = np.nan
                self.pending_ts[idx] = 0.0
                continue

            jump = abs(dist_new - dist_old)

            if jump <= self.max_jump_mm:
                # Consistente: actualizar con EMA
                self.dist[idx] = (1.0 - self.ema_alpha) * dist_old + self.ema_alpha * dist_new
                self.ts[idx] = now
                # cancelar pending
                self.pending_cnt[idx] = 0
                self.pending_dist[idx] = np.nan
                self.pending_ts[idx] = 0.0
            else:
                # Salto grande: requiere confirmación (repetición)
                pd = self.pending_dist[idx]
                pc = self.pending_cnt[idx]
                pts = self.pending_ts[idx]

                # si pending está viejo, reiniciar
                if pc > 0 and (now - pts) > 0.20:
                    pc = 0
                    pd = np.nan

                if pc == 0:
                    self.pending_dist[idx] = dist_new
                    self.pending_cnt[idx] = 1
                    self.pending_ts[idx] = now
                else:
                    # confirmamos si el nuevo punto cae cerca del pending
                    if not np.isnan(pd) and abs(dist_new - pd) <= self.max_jump_mm:
                        pc += 1
                        self.pending_cnt[idx] = pc
                        self.pending_ts[idx] = now
                        # cuando se confirma lo suficiente, aceptar (EMA hacia new)
                        if pc >= self.confirm_needed:
                            self.dist[idx] = (1.0 - self.ema_alpha) * dist_old + self.ema_alpha * dist_new
                            self.ts[idx] = now
                            self.pending_cnt[idx] = 0
                            self.pending_dist[idx] = np.nan
                            self.pending_ts[idx] = 0.0
                    else:
                        # no coincide con pending, reiniciar pending
                        self.pending_dist[idx] = dist_new
                        self.pending_cnt[idx] = 1
                        self.pending_ts[idx] = now

    def get_scan(self, now=None):
        """
        Devuelve arrays (angles_deg, distances_mm) sólo de bins vigentes.
        """
        if now is None:
            now = time.time()
        age = now - self.ts
        mask = (age <= self.ttl_s) & (~np.isnan(self.dist))
        if not np.any(mask):
            return np.array([]), np.array([])
        idxs = np.where(mask)[0]
        angles = (idxs.astype(np.float32) * self.bin_deg) % 360.0
        dists = self.dist[idxs].astype(np.float32)
        return angles, dists

# ==========================================
# GESTOR DE RASTRO AVANZADO (AREA + TIEMPO)
# ==========================================
class TrailManager:
    def __init__(self):
        self.points = np.empty((0, 3))
        self.max_points = 15000
        self.last_add_time = 0

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
        y_new = x * sin_r + y * cos_r
        y_new += desplazamiento

        self.points[:, 0] = x_new
        self.points[:, 1] = y_new

        dist_sq = self.points[:, 0]**2 + self.points[:, 1]**2
        mask = dist_sq < 225000000
        self.points = self.points[mask]

    def add_breadcrumb(self):
        now = time.time()
        if now - self.last_add_time > 0.1:
            self.last_add_time = now

            num_samples = 15
            xs = np.random.uniform(-MITAD_ANCHO_MM, MITAD_ANCHO_MM, num_samples)
            ys = np.random.uniform(-DIST_LIDAR_FRENTE_MM, 0, num_samples)
            ts = np.full(num_samples, now)

            new_batch = np.column_stack((xs, ys, ts))
            self.points = np.vstack((self.points, new_batch))

            if self.points.shape[0] > self.max_points:
                self.points = self.points[-self.max_points:]

    def draw(self, screen, cx, cy, visual_rotation_rad):
        if self.points.shape[0] == 0:
            return

        cos_v = math.cos(-visual_rotation_rad)
        sin_v = math.sin(-visual_rotation_rad)

        surf = pygame.Surface((MAP_W, ALTO_VENTANA), pygame.SRCALPHA)

        draw_pts = self.points[::3]
        rx = draw_pts[:, 0] * cos_v - draw_pts[:, 1] * sin_v
        ry = draw_pts[:, 0] * sin_v + draw_pts[:, 1] * cos_v

        screen_xs = cx + (rx * ESCALA_ZOOM)
        screen_ys = cy + (ry * ESCALA_ZOOM)

        for i in range(len(screen_xs)):
            sx, sy = int(screen_xs[i]), int(screen_ys[i])
            if 0 <= sx < MAP_W and 0 <= sy < ALTO_VENTANA:
                pygame.draw.circle(surf, (255, 255, 255, 40), (sx, sy), 3)

        screen.blit(surf, (0, 0))

    def get_density_under_robot(self):
        if self.points.shape[0] == 0:
            return 0

        now = time.time()
        mask_old = (now - self.points[:, 2]) > TIEMPO_IGNORAR_RASTRO_RECIENTE
        old_points = self.points[mask_old]
        if old_points.shape[0] == 0:
            return 0

        dist_sq = old_points[:, 0]**2 + old_points[:, 1]**2
        count = int(np.sum(dist_sq < (300**2)))
        return count

# ==========================================
# DRIVERS
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
        except:
            return False

    def read_loop(self):
        buffer = b''
        while self.running:
            try:
                if self.serial.in_waiting:
                    buffer += self.serial.read(self.serial.in_waiting)
                else:
                    time.sleep(0.001)
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
                        dist = struct.unpack('<H', packet[6+(i*3):8+(i*3)])[0]
                        if dist > 0:
                            ang = start_angle + step * i
                            if ang >= 360:
                                ang -= 360
                            nuevos.append((ang, dist))

                    if nuevos:
                        with self.lock:
                            self.buffer_puntos.extend(nuevos)

            except:
                self.running = False

    def obtener_datos_nuevos(self):
        with self.lock:
            if not self.buffer_puntos:
                return np.array([]), np.array([])
            datos = self.buffer_puntos[:]
            self.buffer_puntos = []
        angulos, distancias = zip(*datos)
        return np.array(angulos, dtype=np.float32), np.array(distancias, dtype=np.float32)

    def close(self):
        self.running = False
        if self.serial:
            self.serial.close()

class ArduinoController:
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        self.current_yaw = 0.0
        self.lock = threading.Lock()

    def connect(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.1)
            time.sleep(2)
            self.running = True
            threading.Thread(target=self._read_loop, daemon=True).start()
            return True
        except:
            return False

    def _read_loop(self):
        while self.running:
            try:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("ANG:"):
                        with self.lock:
                            self.current_yaw = float(line.split(":")[1])
                else:
                    time.sleep(0.005)
            except:
                pass

    def get_yaw(self):
        with self.lock:
            return self.current_yaw

    def send_command(self, izq, der, aux_pwm, comp_state):
        if not self.serial:
            return
        try:
            izq = izq * TRIM_IZQUIERDA
            der = der * TRIM_DERECHA
            izq = max(min(int(izq), 255), -255)
            der = max(min(int(der), 255), -255)
            aux_pwm = max(min(int(aux_pwm), 255), 0)
            comp = 1 if comp_state else 0
            self.serial.write(f"<{izq},{der},{aux_pwm},{comp}>".encode())
        except:
            pass

    def close(self):
        self.running = False
        if self.serial:
            self.serial.close()

# ==========================================
# LÓGICA DE DETECCIÓN Y CONTROL
# ==========================================
def detectar_intrusos_dinamicos(angulos, distancias, offset_correccion):
    if len(distancias) == 0:
        return False, 0, 0

    ang_corr = np.mod(angulos - OFFSET_LIDAR_FÍSICO - offset_correccion + 180, 360) - 180
    rads = np.radians(ang_corr)
    xs = distancias * np.cos(rads)
    ys = distancias * np.sin(rads)

    puntos_interes = []
    for i in range(len(distancias)):
        if distancias[i] < 2000:
            puntos_interes.append((xs[i], ys[i], distancias[i], ang_corr[i]))

    if not puntos_interes:
        return False, 0, 0

    clusters = []
    current = [puntos_interes[0]]
    for i in range(1, len(puntos_interes)):
        if math.hypot(puntos_interes[i][0]-puntos_interes[i-1][0],
                      puntos_interes[i][1]-puntos_interes[i-1][1]) < 300:
            current.append(puntos_interes[i])
        else:
            clusters.append(current)
            current = [puntos_interes[i]]
    clusters.append(current)

    for cl in clusters:
        if len(cl) < 3:
            continue
        w = math.hypot(max(p[0] for p in cl)-min(p[0] for p in cl),
                       max(p[1] for p in cl)-min(p[1] for p in cl))
        if w < ANCHO_MAXIMO_OBJETO_DINAMICO and min(p[2] for p in cl) < DISTANCIA_SEGURIDAD_DINAMICA:
            return True, min(p[2] for p in cl), sum(p[3] for p in cl)/len(cl)

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

    ang_corr = np.mod(angulos - OFFSET_LIDAR_FÍSICO - offset_correccion + 180, 360) - 180

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

    ang_corr = np.mod(angulos - OFFSET_LIDAR_FÍSICO - offset_correccion + 180, 360) - 180

    best_m = 9e9
    best_i = -1
    for i in range(0, len(distancias), 5):
        m = calcular_margen_real(distancias[i], ang_corr[i])
        if m < best_m:
            best_m = m
            best_i = i

    if best_i != -1:
        return best_m/1000.0, np.radians(float(ang_corr[best_i]))
    return None, None

def calcular_control_reactivo(margen_m, ang_rad):
    err = margen_m - TARGET_CLEARANCE_M
    side = "left" if ang_rad > 0 else "right"
    delta = abs(ang_rad) - (math.pi/2) if side == "left" else (math.pi/2) - abs(ang_rad)
    if side == "right":
        err *= -1
    va = float(np.clip((K_DISTANCIA*err) + (K_ANGULO*delta), -1.2, 1.2))
    vl = 1.0 * max(min(1/abs(va+0.01), 1.0), 0.15)
    return vl, va

def cinematica_diferencial(vl, va):
    def apply(v):
        return (v*FACTOR_RESPUESTA) + (np.sign(v)*PWM_MIN) if abs(v) > 0.05 else 0
    izq = np.clip(apply(vl - (va*ANCHO_ROBOT_M/2)), -PWM_MAX, PWM_MAX)
    der = np.clip(apply(vl + (va*ANCHO_ROBOT_M/2)), -PWM_MAX, PWM_MAX)
    return float(izq), float(der)

def obtener_esquinas_robot(cx, cy, ang):
    f = DIST_LIDAR_FRENTE_MM * ESCALA_ZOOM
    b = DIST_LIDAR_ATRAS_MM * ESCALA_ZOOM
    w = (ROBOT_ANCHO_MM/2) * ESCALA_ZOOM
    pts = [(-w, -f), (w, -f), (w, b), (-w, b)]
    rad = math.radians(-ang)
    c = math.cos(rad)
    s = math.sin(rad)
    return [(cx + x*c - y*s, cy + x*s + y*c) for x, y in pts]

# ==========================================
# UI helpers
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
    txts(44, f"LiDAR: {PUERTO_LIDAR}   Arduino: {PUERTO_ARDUINO}")
    pygame.draw.line(screen, (50, 60, 80), (x0 + 16, 70), (x0 + PANEL_W - 16, 70), 1)

    txt(86, f"ESTADO: {state['st']}", (120, 255, 140))
    txts(116, f"Yaw: {state['yaw']:.1f}°   Off: {state['rot_off']:.1f}°   Fix90: {state['fix_90']}°")
    txts(140, f"PWM L/R: {int(state['pl'])} / {int(state['pr'])}")
    txts(164, f"FPS: {state['fps']:.1f}")

    pygame.draw.line(screen, (50, 60, 80), (x0 + 16, 190), (x0 + PANEL_W - 16, 190), 1)

    # Limpieza
    ptxt = state['pump_txt']
    col = (0, 200, 255) if "ON" in ptxt else (190, 190, 210)
    txt(206, "LIMPIEZA", (220, 230, 245))
    txts(234, ptxt, col)
    txts(258, f"Compresor: {'ON' if state['comp_on'] else 'OFF'}")
    txts(282, f"Potencia: {int(state['power']*100)}%")

    pygame.draw.line(screen, (50, 60, 80), (x0 + 16, 310), (x0 + PANEL_W - 16, 310), 1)

    # Sectores
    txt(326, "SECTORES (margen mm)", (220, 230, 245))
    sec = state['sec']
    txts(354, f"Frente: {int(sec['FRENTE']) if sec['FRENTE'] < 9e8 else '-'}")
    txts(378, f"Atras : {int(sec['ATRAS'])  if sec['ATRAS']  < 9e8 else '-'}")
    txts(402, f"Izq   : {int(sec['IZQ'])   if sec['IZQ']   < 9e8 else '-'}")
    txts(426, f"Der   : {int(sec['DER'])   if sec['DER']   < 9e8 else '-'}")

    pygame.draw.line(screen, (50, 60, 80), (x0 + 16, 454), (x0 + PANEL_W - 16, 454), 1)

    # Tips
    txt(470, "TECLAS", (220, 230, 245))
    tips = [
        "SPACE: AUTO/MANUAL",
        "WASD: Manual",
        "M: Spiral (Ready/Run)",
        "R: Toggle limpieza",
        "O: Toggle compresor",
        "V: Nivel potencia",
        "Q/E: Rot offset",
        "T: Fix 90° (0/90/180/270)"
    ]
    y = 498
    for t in tips:
        screen.blit(small.render(t, True, (170, 180, 200)), (x0 + 16, y))
        y += 20

# ==========================================
# MAIN
# ==========================================
def main():
    pygame.init()
    screen = pygame.display.set_mode((ANCHO_VENTANA, ALTO_VENTANA))
    pygame.display.set_caption("ROBOT CLEANER PRO (Persistente)")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Arial", 18, bold=True)
    small = pygame.font.SysFont("Consolas", 14, bold=False)

    lidar = LiDAR_LD20(PUERTO_LIDAR)
    arduino = ArduinoController(PUERTO_ARDUINO)

    if not lidar.connect():
        print("No se pudo conectar al LiDAR.")
        return
    if not arduino.connect():
        print("Arduino OFF")

    threading.Thread(target=lidar.read_loop, daemon=True).start()

    trail = TrailManager()

    # Persistencia LiDAR
    lidar_mem = LidarPersistence(
        bin_deg=1.0,
        ttl_s=0.45,
        max_jump_mm=250,
        confirm_needed=2,
        ema_alpha=0.35
    )

    st = 'MANUAL'
    pl = 0
    pr = 0
    cx = 170
    cy = ALTO_VENTANA - 170
    rot_off = 0
    fix_90 = 0

    yaw_ini = None
    clean_sys = False
    comp_on = False
    pwr_idx = 0

    t_maniobra = 0
    yaw_spin = 0
    t_spiral = 0
    last_yaw = 0
    t_patience = 0
    evade_dir = 0

    try:
        while True:
            dt = clock.get_time() / 1000.0
            if dt <= 1e-6:
                dt = 1e-3

            now = time.time()

            ang_new, dist_new = lidar.obtener_datos_nuevos()
            curr_yaw = arduino.get_yaw()
            if yaw_ini is None:
                yaw_ini = curr_yaw
                last_yaw = curr_yaw

            # Alimentar persistencia
            if len(dist_new) > 0:
                lidar_mem.update(ang_new, dist_new, now=now)

            # Usar scan persistente para TODO
            ang, dist = lidar_mem.get_scan(now=now)

            # Odometría y Rastro
            speed = ((pl + pr) / 200.0) * 300.0
            dy = curr_yaw - last_yaw
            last_yaw = curr_yaw
            if dy > 180:
                dy -= 360
            elif dy < -180:
                dy += 360

            trail.update(speed, math.radians(dy) / dt, dt)
            if clean_sys and speed > 10:
                trail.add_breadcrumb()

            keys = pygame.key.get_pressed()
            if keys[pygame.K_q]:
                rot_off += 1
            if keys[pygame.K_e]:
                rot_off -= 1
            if keys[pygame.K_w] or keys[pygame.K_s] or keys[pygame.K_a] or keys[pygame.K_d]:
                st = 'MANUAL'

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
                            t_spiral = time.time()
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

            # ==========================
            # Render MAPA
            # ==========================
            vis_rot = (curr_yaw - yaw_ini) + rot_off
            screen.fill((10, 10, 20))

            # Zona mapa (fondo)
            pygame.draw.rect(screen, (10, 10, 20), (0, 0, MAP_W, ALTO_VENTANA))
            # grid suave
            for gx in range(0, MAP_W, 60):
                pygame.draw.line(screen, (18, 18, 28), (gx, 0), (gx, ALTO_VENTANA), 1)
            for gy in range(0, ALTO_VENTANA, 60):
                pygame.draw.line(screen, (18, 18, 28), (0, gy), (MAP_W, gy), 1)

            # rastro
            trail.draw(screen, cx, cy, math.radians(vis_rot))

            # robot
            esq = obtener_esquinas_robot(cx, cy, vis_rot)
            pygame.draw.polygon(screen, (255, 200, 0), esq, 2)
            pygame.draw.line(screen, (255, 80, 80), esq[0], esq[1], 4)  # frente
            pygame.draw.circle(screen, (0, 200, 255), (cx, cy), 6)

            # LiDAR puntos persistentes (más legibles)
            if len(ang) > 0:
                ar = np.radians(ang - OFFSET_LIDAR_FÍSICO - fix_90 - 90 - vis_rot)
                xd = cx + dist * ESCALA_ZOOM * np.cos(ar)
                yd = cy + dist * ESCALA_ZOOM * np.sin(ar)

                step = max(1, len(ang) // 420)
                for i in range(0, len(ang), step):
                    xpi = int(xd[i])
                    ypi = int(yd[i])
                    if 0 <= xpi < MAP_W and 0 <= ypi < ALTO_VENTANA:
                        pygame.draw.circle(screen, (0, 255, 120), (xpi, ypi), 2)

            # ==========================
            # LÓGICA CONTROL (con scan persistente)
            # ==========================
            sec = analizar_sectores_reales(ang, dist, fix_90)
            intruder, dist_intr, ang_intr = detectar_intrusos_dinamicos(ang, dist, fix_90)

            if intruder and st not in ['MANUAL', 'WAIT_DYN', 'EVADE_DYN']:
                st = 'WAIT_DYN'
                t_patience = time.time()

            if st == 'MANUAL':
                pl, pr = 0, 0
                if keys[pygame.K_w]:
                    pl, pr = 60, 60
                if keys[pygame.K_s]:
                    pl, pr = -60, -60
                if keys[pygame.K_a]:
                    pl, pr = 60, -60
                if keys[pygame.K_d]:
                    pl, pr = -60, 60

            elif st == 'WAIT_DYN':
                pl, pr = 0, 0
                pygame.draw.circle(
                    screen, (255, 60, 60), (cx, cy),
                    int(DISTANCIA_SEGURIDAD_DINAMICA * ESCALA_ZOOM), 2
                )
                if not intruder:
                    st = 'AUTO_NAV'
                elif time.time() - t_patience > TIEMPO_PACIENCIA_DINAMICO:
                    st = 'EVADE_DYN'
                    evade_dir = -1 if ang_intr > 0 else 1

            elif st == 'EVADE_DYN':
                pl, pr = 65 * evade_dir, -65 * evade_dir
                if not intruder and sec['FRENTE'] > (ROBOT_LARGO_MM + 200):
                    st = 'AUTO_NAV'

            elif st == 'SPIRAL_READY':
                pl, pr = 0, 0

            elif st == 'SPIRAL_RUN':
                if sec['FRENTE'] < 200:
                    st = 'AUTO_NAV'
                else:
                    te = time.time() - t_spiral
                    va = 300.0 / (te + 2.0)
                    va = max(10, va)
                    pl = 70 + va
                    pr = 70 - (va * 0.5)

            elif st == 'AUTO_NAV':
                if sec['FRENTE'] < 150 or sec['IZQ'] < 150 or sec['DER'] < 150:
                    st = 'ESC_REV'
                    t_maniobra = time.time()
                else:
                    mr, arad = obtener_referencia_navegacion(ang, dist, fix_90)
                    if mr is not None:
                        vl, va = calcular_control_reactivo(mr, arad)
                        pl, pr = cinematica_diferencial(vl, va)

            elif st == 'ESC_REV':
                if sec['ATRAS'] < 250:
                    st = 'ESC_SPIN'
                    yaw_spin = curr_yaw
                elif time.time() - t_maniobra < 2.0:
                    pl, pr = -60, -60
                else:
                    st = 'ESC_SPIN'
                    yaw_spin = curr_yaw

            elif st == 'ESC_SPIN':
                pl, pr = 65, -65
                if sec['FRENTE'] > (ROBOT_LARGO_MM + 200):
                    st = 'AUTO_NAV'
                if abs(curr_yaw - yaw_spin) > 400:
                    st = 'ESC_REV'
                    t_maniobra = time.time()

            # ==========================
            # Actuadores Smart
            # ==========================
            rev = (pl < -10 and pr < -10)
            dens = trail.get_density_under_robot()
            clean_already = dens > UMBRAL_DENSIDAD_VISITADO

            pump = clean_sys and (not rev) and (not clean_already) and st not in ['WAIT_DYN', 'EVADE_DYN']
            pwm_pump = int(255 * NIVELES_LIMPIEZA[pwr_idx]) if pump else 0
            arduino.send_command(pl, pr, pwm_pump, comp_on)

            # ==========================
            # UI panel
            # ==========================
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
                    ptxt = f"ON ({int(NIVELES_LIMPIEZA[pwr_idx]*100)}%)"

            fps = clock.get_fps()
            state = {
                "st": st,
                "yaw": float(curr_yaw),
                "rot_off": float(rot_off),
                "fix_90": int(fix_90),
                "pl": float(pl),
                "pr": float(pr),
                "fps": float(fps),
                "pump_txt": ptxt,
                "comp_on": bool(comp_on),
                "power": float(NIVELES_LIMPIEZA[pwr_idx]),
                "sec": sec,
            }
            draw_panel(screen, font, small, state)

            pygame.display.flip()
            clock.tick(30)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            if arduino:
                arduino.send_command(0, 0, 0, 0)
        except:
            pass
        try:
            lidar.close()
        except:
            pass
        try:
            if arduino:
                arduino.close()
        except:
            pass
        pygame.quit()

if __name__ == "__main__":
    main()