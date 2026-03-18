import serial
import struct
import threading
import time
import numpy as np
import pygame
import math
from ArduinoBridge import ArduinoLink

# ==========================================
# CONFIGURACIÓN GENERAL
# ==========================================
PUERTO_LIDAR = 'COM8'
PUERTO_ARDUINO = 'COM7'
BAUD_ARDUINO = 115200

ANCHO_VENTANA = 1440
ALTO_VENTANA = 900
PANEL_W = 360
MAP_W = ANCHO_VENTANA - PANEL_W

FPS_OBJETIVO = 30
FPS_DIBUJO = 20
INTERVALO_DIBUJO = 1.0 / FPS_DIBUJO

# ==========================================
# MAPA GLOBAL FIJO: 8x8 METROS
# ==========================================
MAPA_ANCHO_MM = 8000
MAPA_ALTO_MM = 8000
CELDA_MM = 200  # 40x40 celdas

GRID_COLS = MAPA_ANCHO_MM // CELDA_MM
GRID_ROWS = MAPA_ALTO_MM // CELDA_MM

# inicio abajo a la izquierda
ROBOT_X_INICIAL_MM = 250.0
ROBOT_Y_INICIAL_MM = 250.0
ROBOT_YAW_INICIAL_DEG = 0.0

# ==========================================
# GEOMETRÍA ROBOT
# ==========================================
ROBOT_LARGO_MM = 750
ROBOT_ANCHO_MM = 450
DIST_LIDAR_FRENTE_MM = 280
DIST_LIDAR_ATRAS_MM = ROBOT_LARGO_MM - DIST_LIDAR_FRENTE_MM
MITAD_ANCHO_MM = ROBOT_ANCHO_MM / 2

OFFSET_LIDAR_FISICO = 186.0

# ==========================================
# LIMPIEZA / ACTUADORES
# ==========================================
NIVELES_LIMPIEZA = [0.15, 0.40, 1.00]
RADIO_LIMPIEZA_MM = 220

# ==========================================
# LIDAR
# ==========================================
LIDAR_MIN_MM = 30
LIDAR_MAX_MM = 12000

LIDAR_BIN_DEG = 1.0
LIDAR_TTL_S = 0.45
LIDAR_MAX_JUMP_MM = 250
LIDAR_CONFIRM_NEEDED = 2
LIDAR_EMA_ALPHA = 0.35

# ==========================================
# CONTROL MANUAL
# ==========================================
PWM_BASE = 90
PWM_GIRO = 80
PWM_STEP = 5
PWM_MIN = 35
PWM_MAX = 180

# velocidad lineal estimada para mapa
MM_S_A_PWM_100 = 420.0

# ==========================================
# COLORES
# ==========================================
COLOR_BG = (10, 10, 18)
COLOR_PANEL = (18, 18, 28)
COLOR_GRID = (45, 50, 62)
COLOR_BORDER = (95, 105, 125)
COLOR_DISCOVERED = (38, 48, 66)
COLOR_CLEANED = (65, 120, 90)
COLOR_LIDAR = (0, 255, 120)
COLOR_ROBOT = (0, 220, 255)
COLOR_HEADING = (255, 100, 100)
COLOR_TEXT = (220, 230, 245)
COLOR_SUBTEXT = (180, 190, 210)
COLOR_WARN = (255, 170, 90)

# ==========================================
# LIDAR PERSISTENTE
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
# DRIVER LiDAR LD20
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
# BRIDGE ARDUINO
# ==========================================
class ArduinoBridgeController:
    """
    Espera mensajes:
      DANG:<delta_deg>

    Y envía:
      CMD:<izq>,<der>,<aux_pwm>,<comp>
    """
    def __init__(self, port=None, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.link = None
        self.lock = threading.Lock()

        self.connected = False
        self.last_rx_time = 0.0
        self.last_command = "CMD:0,0,0,0"

        self.accumulated_yaw_deg = 0.0
        self.last_dang = 0.0

    def connect(self):
        try:
            self.link = ArduinoLink(port=self.port, baud=self.baudrate)

            @self.link.on_message("DANG")
            def _on_dang(msg):
                vals = msg.as_floats()
                if vals:
                    with self.lock:
                        self.accumulated_yaw_deg += float(vals[0])
                        self.last_dang = float(vals[0])
                        self.last_rx_time = time.time()

            @self.link.on_message("*")
            def _on_any(msg):
                self.last_rx_time = time.time()

            self.link.start()

            t0 = time.time()
            while time.time() - t0 < 3.5:
                if self.link.connected:
                    self.connected = True
                    return True
                time.sleep(0.05)

            self.connected = bool(self.link.connected)
            return self.connected

        except Exception as e:
            print(f"Error conectando ArduinoBridge: {e}")
            self.connected = False
            return False

    def is_connected(self):
        self.connected = bool(self.link and self.link.connected)
        return self.connected

    def get_yaw(self):
        with self.lock:
            return self.accumulated_yaw_deg

    def get_last_dang(self):
        with self.lock:
            return self.last_dang

    def send_command(self, izq, der, aux_pwm, comp_state):
        if not self.link or not self.link.connected:
            return

        try:
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
# MAPA DE CUADRÍCULA
# ==========================================
class GridMap:
    def __init__(self, width_mm, height_mm, cell_mm):
        self.width_mm = width_mm
        self.height_mm = height_mm
        self.cell_mm = cell_mm

        self.cols = int(width_mm // cell_mm)
        self.rows = int(height_mm // cell_mm)

        self.discovered = np.zeros((self.rows, self.cols), dtype=np.uint8)
        self.cleaned = np.zeros((self.rows, self.cols), dtype=np.uint8)

    def world_to_cell(self, x_mm, y_mm):
        col = int(x_mm // self.cell_mm)
        row = int(y_mm // self.cell_mm)
        if 0 <= col < self.cols and 0 <= row < self.rows:
            return row, col
        return None

    def mark_discovered(self, x_mm, y_mm):
        idx = self.world_to_cell(x_mm, y_mm)
        if idx is not None:
            r, c = idx
            self.discovered[r, c] = 1

    def mark_cleaned_disk(self, x_mm, y_mm, radius_mm=220):
        r_cells = max(1, int(radius_mm / self.cell_mm) + 1)
        center = self.world_to_cell(x_mm, y_mm)
        if center is None:
            return

        cr, cc = center
        for rr in range(cr - r_cells, cr + r_cells + 1):
            for cc2 in range(cc - r_cells, cc + r_cells + 1):
                if 0 <= rr < self.rows and 0 <= cc2 < self.cols:
                    cell_center_x = (cc2 + 0.5) * self.cell_mm
                    cell_center_y = (rr + 0.5) * self.cell_mm
                    if (cell_center_x - x_mm) ** 2 + (cell_center_y - y_mm) ** 2 <= radius_mm ** 2:
                        self.cleaned[rr, cc2] = 1

    def get_discovered_pct(self):
        return 100.0 * float(np.mean(self.discovered))

    def get_cleaned_pct(self):
        return 100.0 * float(np.mean(self.cleaned))

    def draw(self, screen, area_rect):
        x0, y0, w, h = area_rect
        cell_w = w / self.cols
        cell_h = h / self.rows

        for r in range(self.rows):
            for c in range(self.cols):
                rx = int(x0 + c * cell_w)
                ry = int(y0 + (self.rows - 1 - r) * cell_h)

                color = (22, 22, 30)
                if self.discovered[r, c]:
                    color = COLOR_DISCOVERED
                if self.cleaned[r, c]:
                    color = COLOR_CLEANED

                pygame.draw.rect(screen, color, (rx, ry, int(cell_w) + 1, int(cell_h) + 1))

        for c in range(self.cols + 1):
            gx = int(x0 + c * cell_w)
            pygame.draw.line(screen, COLOR_GRID, (gx, y0), (gx, y0 + h), 1)

        for r in range(self.rows + 1):
            gy = int(y0 + r * cell_h)
            pygame.draw.line(screen, COLOR_GRID, (x0, gy), (x0 + w, gy), 1)

        pygame.draw.rect(screen, COLOR_BORDER, area_rect, 2)

    def draw_robot(self, screen, area_rect, x_mm, y_mm, yaw_deg):
        x0, y0, w, h = area_rect

        sx = x0 + (x_mm / self.width_mm) * w
        sy = y0 + h - (y_mm / self.height_mm) * h

        pygame.draw.circle(screen, COLOR_ROBOT, (int(sx), int(sy)), 8)

        heading_len = 24
        rad = math.radians(yaw_deg)
        ex = sx + heading_len * math.cos(rad)
        ey = sy - heading_len * math.sin(rad)

        pygame.draw.line(screen, COLOR_HEADING, (int(sx), int(sy)), (int(ex), int(ey)), 3)

    def draw_lidar_hits(self, screen, area_rect, points_xy_mm):
        x0, y0, w, h = area_rect

        for x_mm, y_mm in points_xy_mm:
            if 0 <= x_mm <= self.width_mm and 0 <= y_mm <= self.height_mm:
                sx = x0 + (x_mm / self.width_mm) * w
                sy = y0 + h - (y_mm / self.height_mm) * h
                pygame.draw.circle(screen, COLOR_LIDAR, (int(sx), int(sy)), 2)


# ==========================================
# UI
# ==========================================
def draw_panel(screen, font, small, state):
    x0 = MAP_W
    pygame.draw.rect(screen, COLOR_PANEL, (x0, 0, PANEL_W, ALTO_VENTANA))
    pygame.draw.line(screen, (50, 60, 80), (x0, 0), (x0, ALTO_VENTANA), 2)

    def txt(y, s, color=COLOR_TEXT):
        screen.blit(font.render(s, True, color), (x0 + 16, y))

    def txts(y, s, color=COLOR_SUBTEXT):
        screen.blit(small.render(s, True, color), (x0 + 16, y))

    txt(14, "ROBOT CLEANER PRO", (255, 210, 120))
    txts(42, f"LiDAR: {state['lidar_port']}")
    txts(64, f"Arduino: {state['arduino_port']}")

    pygame.draw.line(screen, (50, 60, 80), (x0 + 16, 94), (x0 + PANEL_W - 16, 94), 1)

    txt(110, "POSE GLOBAL")
    txts(138, f"X: {state['x_mm']:.1f} mm")
    txts(160, f"Y: {state['y_mm']:.1f} mm")
    txts(182, f"Yaw: {state['yaw_deg']:.2f} deg")

    pygame.draw.line(screen, (50, 60, 80), (x0 + 16, 214), (x0 + PANEL_W - 16, 214), 1)

    txt(230, "CONTROL MANUAL")
    txts(258, f"PWM avance: {state['pwm_base']}")
    txts(280, f"PWM giro: {state['pwm_turn']}")
    txts(302, f"Motor L/R: {int(state['pl'])} / {int(state['pr'])}")
    txts(324, f"Vel est: {state['v_est']:.1f} mm/s")

    pygame.draw.line(screen, (50, 60, 80), (x0 + 16, 354), (x0 + PANEL_W - 16, 354), 1)

    txt(370, "ACTUADORES")
    txts(398, f"Limpieza: {'ON' if state['clean_on'] else 'OFF'}")
    txts(420, f"Compresor: {'ON' if state['comp_on'] else 'OFF'}")
    txts(442, f"Potencia: {int(state['power']*100)}%")
    txts(464, f"PWM auxiliar: {state['aux_pwm']}")

    pygame.draw.line(screen, (50, 60, 80), (x0 + 16, 494), (x0 + PANEL_W - 16, 494), 1)

    txt(510, "MAPA 8x8 m")
    txts(538, f"Descubierto: {state['disc_pct']:.1f}%")
    txts(560, f"Limpio: {state['clean_pct']:.1f}%")
    txts(582, f"Ult DANG: {state['last_dang']:.4f}")
    txts(604, f"Arduino: {'OK' if state['arduino_ok'] else 'DESCONECTADO'}")
    txts(626, f"Ult RX: {state['rx_age_txt']}")

    pygame.draw.line(screen, (50, 60, 80), (x0 + 16, 656), (x0 + PANEL_W - 16, 656), 1)

    txt(672, "TECLAS")
    tips = [
        "W/S: avanzar / reversa",
        "A/D: girar",
        "R: toggle limpieza",
        "O: toggle compresor",
        "V: cambiar potencia",
        "1/2: PWM -/+ avance",
        "3/4: PWM -/+ giro",
        "X: parar todo",
    ]
    y = 700
    for t in tips:
        screen.blit(small.render(t, True, (170, 180, 200)), (x0 + 16, y))
        y += 18


# ==========================================
# UTILIDADES DE MOVIMIENTO
# ==========================================
def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def get_manual_drive(keys, pwm_base, pwm_turn):
    pl = 0.0
    pr = 0.0

    if keys[pygame.K_w]:
        pl, pr = pwm_base, pwm_base
    elif keys[pygame.K_s]:
        pl, pr = -pwm_base, -pwm_base
    elif keys[pygame.K_a]:
        pl, pr = -pwm_turn, pwm_turn
    elif keys[pygame.K_d]:
        pl, pr = pwm_turn, -pwm_turn

    return pl, pr


def estimate_linear_speed_mm_s(pl, pr):
    if pl * pr > 0:
        pwm_avg = (abs(pl) + abs(pr)) / 2.0
        sign = 1.0 if pl > 0 else -1.0
        return sign * (pwm_avg / 100.0) * MM_S_A_PWM_100
    return 0.0


def transform_lidar_to_global(angles_deg, distances_mm, robot_x_mm, robot_y_mm, yaw_deg):
    if len(distances_mm) == 0:
        return []

    ang_rel = np.radians(angles_deg - OFFSET_LIDAR_FISICO)
    yaw_rad = math.radians(yaw_deg)

    xg = robot_x_mm + distances_mm * np.cos(ang_rel + yaw_rad)
    yg = robot_y_mm + distances_mm * np.sin(ang_rel + yaw_rad)

    pts = []
    for x, y in zip(xg, yg):
        pts.append((float(x), float(y)))
    return pts


# ==========================================
# MAIN
# ==========================================
def main():
    pygame.init()
    screen = pygame.display.set_mode((ANCHO_VENTANA, ALTO_VENTANA))
    pygame.display.set_caption("Robot Cleaner Pro - Manual Test 8x8m")
    clock = pygame.time.Clock()

    font = pygame.font.SysFont("Arial", 18, bold=True)
    small = pygame.font.SysFont("Consolas", 14)

    lidar = LiDAR_LD20(PUERTO_LIDAR)
    arduino = ArduinoBridgeController(PUERTO_ARDUINO, BAUD_ARDUINO)

    if not lidar.connect():
        print("No se pudo conectar al LiDAR.")
        pygame.quit()
        return

    if not arduino.connect():
        print("ArduinoBridge no conectado al inicio.")

    threading.Thread(target=lidar.read_loop, daemon=True).start()

    lidar_mem = LidarPersistence(
        bin_deg=LIDAR_BIN_DEG,
        ttl_s=LIDAR_TTL_S,
        max_jump_mm=LIDAR_MAX_JUMP_MM,
        confirm_needed=LIDAR_CONFIRM_NEEDED,
        ema_alpha=LIDAR_EMA_ALPHA,
        min_dist_mm=LIDAR_MIN_MM,
        max_dist_mm=LIDAR_MAX_MM,
    )

    grid_map = GridMap(MAPA_ANCHO_MM, MAPA_ALTO_MM, CELDA_MM)

    robot_x_mm = ROBOT_X_INICIAL_MM
    robot_y_mm = ROBOT_Y_INICIAL_MM
    yaw_offset_deg = ROBOT_YAW_INICIAL_DEG

    clean_sys = False
    comp_on = False
    pwr_idx = 0

    pwm_base = PWM_BASE
    pwm_turn = PWM_GIRO

    pl = 0.0
    pr = 0.0
    v_est = 0.0

    last_render_time = 0.0

    # área cuadrada máxima para ver TODO el mapa 8x8 completo
    map_size = min(MAP_W - 40, ALTO_VENTANA - 40)
    map_x = 20
    map_y = (ALTO_VENTANA - map_size) // 2
    map_rect = (map_x, map_y, map_size, map_size)

    try:
        while True:
            dt = clock.tick(FPS_OBJETIVO) / 1000.0
            if dt <= 1e-6:
                dt = 1e-3

            now = time.time()

            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    raise KeyboardInterrupt

                if e.type == pygame.KEYDOWN:
                    if e.key == pygame.K_r:
                        clean_sys = not clean_sys
                    elif e.key == pygame.K_o:
                        comp_on = not comp_on
                    elif e.key == pygame.K_v:
                        pwr_idx = (pwr_idx + 1) % len(NIVELES_LIMPIEZA)
                    elif e.key == pygame.K_1:
                        pwm_base = clamp(pwm_base - PWM_STEP, PWM_MIN, PWM_MAX)
                    elif e.key == pygame.K_2:
                        pwm_base = clamp(pwm_base + PWM_STEP, PWM_MIN, PWM_MAX)
                    elif e.key == pygame.K_3:
                        pwm_turn = clamp(pwm_turn - PWM_STEP, PWM_MIN, PWM_MAX)
                    elif e.key == pygame.K_4:
                        pwm_turn = clamp(pwm_turn + PWM_STEP, PWM_MIN, PWM_MAX)
                    elif e.key == pygame.K_x:
                        pl, pr = 0.0, 0.0
                        clean_sys = False
                        comp_on = False
                        arduino.send_command(0, 0, 0, 0)

            keys = pygame.key.get_pressed()
            pl, pr = get_manual_drive(keys, pwm_base, pwm_turn)

            aux_pwm = int(255 * NIVELES_LIMPIEZA[pwr_idx]) if clean_sys else 0
            arduino.send_command(pl, pr, aux_pwm, comp_on)

            # ---------------------------
            # YAW REAL POR GIROSCOPIO
            # ---------------------------
            robot_yaw_deg = yaw_offset_deg + arduino.get_yaw()
            last_dang = arduino.get_last_dang()

            # ---------------------------
            # POSICIÓN ESTIMADA POR COMANDO
            # ---------------------------
            v_est = estimate_linear_speed_mm_s(pl, pr)
            yaw_rad = math.radians(robot_yaw_deg)

            robot_x_mm += v_est * dt * math.cos(yaw_rad)
            robot_y_mm += v_est * dt * math.sin(yaw_rad)

            robot_x_mm = clamp(robot_x_mm, 0.0, MAPA_ANCHO_MM)
            robot_y_mm = clamp(robot_y_mm, 0.0, MAPA_ALTO_MM)

            # ---------------------------
            # LIDAR
            # ---------------------------
            ang_new, dist_new = lidar.obtener_datos_nuevos()
            if len(dist_new) > 0:
                lidar_mem.update(ang_new, dist_new, now=now)

            ang, dist = lidar_mem.get_scan(now=now)
            lidar_hits_global = transform_lidar_to_global(ang, dist, robot_x_mm, robot_y_mm, robot_yaw_deg)

            for xp, yp in lidar_hits_global:
                grid_map.mark_discovered(xp, yp)

            if clean_sys and (abs(pl) > 5 or abs(pr) > 5):
                grid_map.mark_cleaned_disk(robot_x_mm, robot_y_mm, radius_mm=RADIO_LIMPIEZA_MM)

            # ---------------------------
            # DIBUJO
            # ---------------------------
            if (now - last_render_time) >= INTERVALO_DIBUJO:
                last_render_time = now

                screen.fill(COLOR_BG)

                grid_map.draw(screen, map_rect)
                grid_map.draw_lidar_hits(screen, map_rect, lidar_hits_global)
                grid_map.draw_robot(screen, map_rect, robot_x_mm, robot_y_mm, robot_yaw_deg)

                rx_age = now - arduino.last_rx_time if arduino.last_rx_time > 0 else None
                rx_age_txt = f"{rx_age:.2f}s" if rx_age is not None else "--"

                state = {
                    "lidar_port": PUERTO_LIDAR,
                    "arduino_port": PUERTO_ARDUINO,
                    "x_mm": robot_x_mm,
                    "y_mm": robot_y_mm,
                    "yaw_deg": robot_yaw_deg,
                    "pwm_base": pwm_base,
                    "pwm_turn": pwm_turn,
                    "pl": pl,
                    "pr": pr,
                    "v_est": v_est,
                    "clean_on": clean_sys,
                    "comp_on": comp_on,
                    "power": NIVELES_LIMPIEZA[pwr_idx],
                    "aux_pwm": aux_pwm,
                    "disc_pct": grid_map.get_discovered_pct(),
                    "clean_pct": grid_map.get_cleaned_pct(),
                    "last_dang": last_dang,
                    "arduino_ok": arduino.is_connected(),
                    "rx_age_txt": rx_age_txt,
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