import math
import time
import struct
import threading
import numpy as np
import pygame
import serial

# ==========================================
# PUERTOS FIJOS
# ==========================================
PUERTO_LIDAR = "COM8"
PUERTO_ARDUINO = "COM7"

BAUD_LIDAR = 230400
BAUD_ARDUINO = 115200

# ==========================================
# VENTANA / LAYOUT
# ==========================================
ANCHO_VENTANA = 1500
ALTO_VENTANA = 920
PANEL_W = 360
MAP_W = ANCHO_VENTANA - PANEL_W

MARGEN = 18
MAP_TOP = 18
HELP_H = 165
MAP_BOTTOM = ALTO_VENTANA - HELP_H - MARGEN

FPS_OBJETIVO = 30
FPS_DIBUJO = 20
INTERVALO_DIBUJO = 1.0 / FPS_DIBUJO

# ==========================================
# MAPA FIJO 8x8 m
# ==========================================
MAPA_ANCHO_MM = 8000
MAPA_ALTO_MM = 8000
CELDA_MM = 100

GRID_COLS = MAPA_ANCHO_MM // CELDA_MM
GRID_ROWS = MAPA_ALTO_MM // CELDA_MM

# Referencia global = centro del LiDAR
# Se coloca al centro del mapa para que toda la cuadrícula inicial
# del robot esté siempre contenida dentro del mapa visible.
LIDAR_X_INICIAL_MM = MAPA_ANCHO_MM / 2
LIDAR_Y_INICIAL_MM = MAPA_ALTO_MM / 2
LIDAR_YAW_INICIAL_DEG = 0.0

# ==========================================
# GEOMETRÍA DEL ROBOT
# Referencia = centro del LiDAR
# ==========================================
ROBOT_LARGO_MM = 750
ROBOT_ANCHO_MM = 450
DIST_LIDAR_FRENTE_MM = 280
DIST_LIDAR_ATRAS_MM = ROBOT_LARGO_MM - DIST_LIDAR_FRENTE_MM
MITAD_ANCHO_MM = ROBOT_ANCHO_MM / 2

OFFSET_LIDAR_FISICO = 186.0

# ==========================================
# ODOMETRÍA / ESTIMACIÓN DE POSE
# ==========================================
WHEEL_TRACK_MM     = 395.0   # separación centro-centro ruedas motrices

# Velocidad lineal estimada.
# Ajusta VEL_MM_PER_PWM midiendo cuánto avanza el robot en 1 s a un PWM fijo.
# Ejemplo: si a PWM=100 avanza ~350 mm/s → VEL_MM_PER_PWM = 3.5
VEL_MM_PER_PWM     = 3.5     # mm/s por unidad de PWM

# Por debajo de este PWM (abs) se considera que la rueda no se mueve
PWM_VEL_THRESHOLD  = 30

# Corrección LiDAR (solo cuando el robot está parado)
LIDAR_CORR_ALPHA        = 0.10   # fracción del error que se corrige por tick
LIDAR_CORR_MIN_HITS     = 10     # mínimo de hits para confiar en el centroide
LIDAR_CORR_MAX_MM       = 80.0   # corrección máxima permitida por tick (anti-jump)
LIDAR_CORR_STABLE_S     = 0.30   # segundos quieto antes de activar corrección

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
SIGNO_MOTOR_IZQ = -1
SIGNO_MOTOR_DER = 1

# ==========================================
# ACTUADORES
# ==========================================
NIVELES_LIMPIEZA = [0.15, 0.40, 1.00]
RADIO_LIMPIEZA_MM = 220

# ==========================================
# MODOS DE OPERACIÓN
# ==========================================
MODE_MANUAL = "MANUAL"
MODE_RUTA   = "RUTA"

# Waypoints
WP_RADIO_PX        = 7    # radio del círculo de cada waypoint en pantalla
WP_REACH_MM        = 150  # distancia para considerar waypoint alcanzado
COLOR_RUTA_LINE    = (255, 200,  50)
COLOR_WP_NORMAL    = (255, 160,  30)
COLOR_WP_ACTIVE    = ( 80, 220, 255)
COLOR_WP_DONE      = (100, 200, 120)
COLOR_WP_BLOCKED   = (255,  70,  70)
COLOR_MODE_MANUAL  = (120, 255, 140)
COLOR_MODE_RUTA    = ( 80, 200, 255)

# ==========================================
# FOLLOWER — MÁQUINA DE ESTADOS
# ==========================================
FSM_IDLE         = "IDLE"
FSM_ALIGN        = "ALIGN"
FSM_ADVANCE      = "ADVANCE"
FSM_WP_REACHED   = "WP_REACHED"
FSM_ROUTE_DONE   = "ROUTE_DONE"
FSM_BLOCKED      = "BLOCKED"

# Umbrales de alineación
ALIGN_START_DEG  = 10.0   # empieza a girar si |error| > este valor
ALIGN_DONE_DEG   =  4.0   # considera alineado cuando |error| < este valor

# PWM del follower (independientes del control manual)
PWM_FOLLOWER_ALIGN   = 65    # giro en sitio durante ALIGN
PWM_FOLLOWER_ADVANCE = 85    # velocidad base de avance
PWM_FOLLOWER_MIN     = 35    # mínimo PWM para que el robot se mueva
K_HEADING            =  1.6  # ganancia proporcional de corrección de heading

# ==========================================
# ALERTA DINÁMICA
# ==========================================
RADIO_ALERTA_DINAMICA_MM = 500.0
UMBRAL_PUNTOS_DINAMICOS = 6

# ==========================================
# COLORES
# ==========================================
COLOR_BG = (10, 10, 18)
COLOR_PANEL = (18, 18, 28)
COLOR_CARD = (26, 29, 40)
COLOR_GRID = (210, 214, 220)
COLOR_BORDER = (95, 105, 125)
COLOR_UNDISCOVERED = (245, 247, 250)
COLOR_DISCOVERED = (214, 232, 248)
COLOR_CLEANED = (165, 220, 182)
COLOR_LIDAR = (0, 220, 110)
COLOR_ROBOT_FILL = (255, 220, 70)
COLOR_ROBOT = (255, 180, 0)
COLOR_LIDAR_CENTER = (0, 220, 255)
COLOR_HEADING = (255, 90, 90)
COLOR_ALERT = (255, 70, 70)
COLOR_TEXT = (230, 236, 245)
COLOR_SUBTEXT = (185, 194, 212)
COLOR_OK = (120, 255, 140)
COLOR_WARN = (255, 170, 90)
COLOR_KEYBOX = (36, 40, 54)
COLOR_KEYBOX_BORDER = (72, 78, 96)

# ==========================================
# UTILIDADES
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


def transform_scan_to_global(angles_deg, distances_mm, lidar_x_mm, lidar_y_mm, yaw_deg):
    if len(distances_mm) == 0:
        return []

    ang_rel = np.radians(angles_deg - OFFSET_LIDAR_FISICO)
    yaw_rad = math.radians(yaw_deg)

    xg = lidar_x_mm + distances_mm * np.cos(ang_rel + yaw_rad)
    yg = lidar_y_mm + distances_mm * np.sin(ang_rel + yaw_rad)

    return [(float(x), float(y)) for x, y in zip(xg, yg)]


def is_point_inside_robot_body_rel(xr, yr):
    return (-DIST_LIDAR_ATRAS_MM <= xr <= DIST_LIDAR_FRENTE_MM) and (-MITAD_ANCHO_MM <= yr <= MITAD_ANCHO_MM)


def detect_dynamic_obstacles(angles_deg, distances_mm):
    if len(distances_mm) == 0:
        return False, 0

    ang_rel = np.radians(angles_deg - OFFSET_LIDAR_FISICO)
    xs = distances_mm * np.cos(ang_rel)
    ys = distances_mm * np.sin(ang_rel)

    count = 0
    for xr, yr in zip(xs, ys):
        d = math.hypot(xr, yr)
        if d <= RADIO_ALERTA_DINAMICA_MM:
            if not is_point_inside_robot_body_rel(xr, yr):
                count += 1

    return count >= UMBRAL_PUNTOS_DINAMICOS, count


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

    def reset(self):
        self.dist[:] = np.nan
        self.ts[:] = 0.0
        self.pending_dist[:] = np.nan
        self.pending_cnt[:] = 0
        self.pending_ts[:] = 0.0

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
        self.thread = None
        self.last_rx_time = 0.0

    def connect(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            self.running = True
            self.thread = threading.Thread(target=self.read_loop, daemon=True)
            self.thread.start()
            return True
        except Exception as e:
            print(f"No se pudo abrir LiDAR: {e}")
            return False

    def read_loop(self):
        buffer = b''
        while self.running:
            try:
                if self.serial and self.serial.in_waiting:
                    buffer += self.serial.read(self.serial.in_waiting)
                    self.last_rx_time = time.time()
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
# ARDUINO DIRECT SERIAL
# ==========================================
class ArduinoSerialController:
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        self.thread = None
        self.lock = threading.Lock()

        self.accumulated_yaw_deg = 0.0
        self.last_dang = 0.0
        self.last_rx_time = 0.0
        self.last_command = "CMD:0,0,0,0"

    def connect(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.05)
            time.sleep(2.0)
            self.running = True
            self.thread = threading.Thread(target=self.read_loop, daemon=True)
            self.thread.start()
            return True
        except Exception as e:
            print(f"No se pudo abrir Arduino: {e}")
            return False

    def read_loop(self):
        while self.running:
            try:
                if self.serial and self.serial.in_waiting:
                    line = self.serial.readline().decode("utf-8", errors="ignore").strip()
                    if not line:
                        continue
                    self.last_rx_time = time.time()

                    if line.startswith("DANG:"):
                        try:
                            val = float(line.split(":", 1)[1])
                            with self.lock:
                                self.accumulated_yaw_deg += val
                                self.last_dang = val
                        except Exception:
                            pass
                else:
                    time.sleep(0.005)
            except Exception:
                self.running = False

    def is_connected(self):
        return self.serial is not None and self.serial.is_open and self.running

    def get_yaw(self):
        with self.lock:
            return self.accumulated_yaw_deg

    def get_last_dang(self):
        with self.lock:
            return self.last_dang

    def reset_yaw_accumulator(self):
        with self.lock:
            self.accumulated_yaw_deg = 0.0
            self.last_dang = 0.0

    def send_command(self, izq, der, aux_pwm, comp_state):
        if not self.serial or not self.serial.is_open:
            return

        try:
            izq = max(min(int(izq), 255), -255)
            der = max(min(int(der), 255), -255)
            aux_pwm = max(min(int(aux_pwm), 255), 0)
            comp = 1 if comp_state else 0

            cmd = f"CMD:{izq},{der},{aux_pwm},{comp}\n"
            self.last_command = cmd.strip()
            self.serial.write(cmd.encode("utf-8"))
        except Exception:
            pass

    def close(self):
        self.running = False
        try:
            if self.serial and self.serial.is_open:
                self.serial.write(b"CMD:0,0,0,0\n")
                self.serial.close()
        except Exception:
            pass


# ==========================================
# MAPA
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

    def reset(self):
        self.discovered.fill(0)
        self.cleaned.fill(0)

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

    def world_to_screen(self, area_rect, x_mm, y_mm):
        x0, y0, w, h = area_rect
        sx = x0 + (x_mm / self.width_mm) * w
        sy = y0 + h - (y_mm / self.height_mm) * h
        return sx, sy

    def draw(self, screen, area_rect):
        x0, y0, w, h = area_rect
        cell_w = w / self.cols
        cell_h = h / self.rows

        pygame.draw.rect(screen, COLOR_UNDISCOVERED, area_rect)

        for r in range(self.rows):
            for c in range(self.cols):
                rx = int(x0 + c * cell_w)
                ry = int(y0 + (self.rows - 1 - r) * cell_h)

                color = COLOR_UNDISCOVERED
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

    def draw_robot(self, screen, area_rect, lidar_x_mm, lidar_y_mm, yaw_deg, alert=False):
        corners_local = [
            (+DIST_LIDAR_FRENTE_MM, +MITAD_ANCHO_MM),
            (+DIST_LIDAR_FRENTE_MM, -MITAD_ANCHO_MM),
            (-DIST_LIDAR_ATRAS_MM, -MITAD_ANCHO_MM),
            (-DIST_LIDAR_ATRAS_MM, +MITAD_ANCHO_MM),
        ]

        yaw_rad = math.radians(yaw_deg)
        c = math.cos(yaw_rad)
        s = math.sin(yaw_rad)

        pts_screen = []
        for xr, yr in corners_local:
            xw = lidar_x_mm + xr * c - yr * s
            yw = lidar_y_mm + xr * s + yr * c
            sx, sy = self.world_to_screen(area_rect, xw, yw)
            pts_screen.append((int(sx), int(sy)))

        if len(pts_screen) >= 3:
            pygame.draw.polygon(screen, COLOR_ROBOT_FILL, pts_screen, 0)
            pygame.draw.polygon(screen, COLOR_ROBOT, pts_screen, 2)

        sx, sy = self.world_to_screen(area_rect, lidar_x_mm, lidar_y_mm)
        pygame.draw.circle(screen, COLOR_LIDAR_CENTER, (int(sx), int(sy)), 5)

        hx = lidar_x_mm + 260.0 * math.cos(yaw_rad)
        hy = lidar_y_mm + 260.0 * math.sin(yaw_rad)
        hsx, hsy = self.world_to_screen(area_rect, hx, hy)
        pygame.draw.line(screen, COLOR_HEADING, (int(sx), int(sy)), (int(hsx), int(hsy)), 3)

        rx, _ = self.world_to_screen(area_rect, lidar_x_mm + RADIO_ALERTA_DINAMICA_MM, lidar_y_mm)
        radius_px = int(abs(rx - sx))
        pygame.draw.circle(screen, COLOR_ALERT if alert else (135, 135, 150), (int(sx), int(sy)), radius_px, 1)

    def draw_lidar_hits(self, screen, area_rect, points_xy_mm):
        for x_mm, y_mm in points_xy_mm:
            if 0 <= x_mm <= self.width_mm and 0 <= y_mm <= self.height_mm:
                sx, sy = self.world_to_screen(area_rect, x_mm, y_mm)
                pygame.draw.circle(screen, COLOR_LIDAR, (int(sx), int(sy)), 2)

    def screen_to_world(self, area_rect, sx, sy):
        """Convierte coordenadas de pantalla a coordenadas globales en mm."""
        x0, y0, w, h = area_rect
        x_mm = (sx - x0) / w * self.width_mm
        y_mm = (1.0 - (sy - y0) / h) * self.height_mm
        return x_mm, y_mm

    def is_inside_map_rect(self, area_rect, sx, sy):
        x0, y0, w, h = area_rect
        return x0 <= sx <= x0 + w and y0 <= sy <= y0 + h

    def draw_route(self, screen, area_rect, waypoints, active_idx, fsm_state=FSM_IDLE):
        """
        Dibuja la ruta de waypoints sobre el mapa.
        waypoints  : lista de (x_mm, y_mm)
        active_idx : índice del waypoint objetivo actual
        fsm_state  : estado actual del follower (afecta color del waypoint activo)
        """
        if not waypoints:
            return

        # Convertir todos a pantalla
        pts_screen = [
            self.world_to_screen(area_rect, x, y)
            for x, y in waypoints
        ]

        # Líneas entre waypoints
        if len(pts_screen) >= 2:
            for i in range(len(pts_screen) - 1):
                p1 = (int(pts_screen[i][0]),     int(pts_screen[i][1]))
                p2 = (int(pts_screen[i+1][0]),   int(pts_screen[i+1][1]))
                color_line = COLOR_WP_DONE if i < active_idx else COLOR_RUTA_LINE
                pygame.draw.line(screen, color_line, p1, p2, 2)

        # Color del waypoint activo según estado FSM
        if fsm_state == FSM_BLOCKED:
            active_color = COLOR_WP_BLOCKED
        elif fsm_state == FSM_ALIGN:
            active_color = (255, 230, 80)       # amarillo: girando
        else:
            active_color = COLOR_WP_ACTIVE      # cyan: avanzando / idle

        # Puntos de waypoints
        lbl_font = pygame.font.SysFont("Consolas", 11)
        for i, (sx, sy) in enumerate(pts_screen):
            if i < active_idx:
                color = COLOR_WP_DONE
                r = WP_RADIO_PX - 2
            elif i == active_idx:
                color = active_color
                r = WP_RADIO_PX + 2
            else:
                color = COLOR_WP_NORMAL
                r = WP_RADIO_PX

            pygame.draw.circle(screen, color, (int(sx), int(sy)), r)
            pygame.draw.circle(screen, (255, 255, 255), (int(sx), int(sy)), r, 1)

            lbl = lbl_font.render(str(i + 1), True, (255, 255, 255))
            screen.blit(lbl, (int(sx) + r + 2, int(sy) - 7))


# ==========================================
# RESET
# ==========================================
def reset_runtime_state(grid_map, lidar_mem, arduino):
    grid_map.reset()
    lidar_mem.reset()
    if arduino is not None:
        arduino.reset_yaw_accumulator()


# ==========================================
# UI
# ==========================================
def draw_card(screen, x, y, w, h, title, font, small):
    rect = pygame.Rect(x, y, w, h)
    pygame.draw.rect(screen, COLOR_CARD, rect, border_radius=12)
    pygame.draw.rect(screen, COLOR_BORDER, rect, 1, border_radius=12)
    screen.blit(font.render(title, True, COLOR_TEXT), (x + 12, y + 10))
    return rect


def draw_panel(screen, font, small, state):
    x0 = MAP_W
    pygame.draw.rect(screen, COLOR_PANEL, (x0, 0, PANEL_W, ALTO_VENTANA))
    pygame.draw.line(screen, (50, 60, 80), (x0, 0), (x0, ALTO_VENTANA), 2)

    title_font = pygame.font.SysFont("Arial", 20, bold=True)
    label_y = 16
    screen.blit(title_font.render("ROBOT CLEANER PRO", True, (255, 215, 125)), (x0 + 16, label_y))
    screen.blit(small.render(f"LiDAR: {state['lidar_port']}  |  Arduino: {state['arduino_port']}", True, COLOR_SUBTEXT), (x0 + 16, label_y + 30))

    cx = x0 + 12
    cw = PANEL_W - 24
    y = 72

    # ── MODO ──────────────────────────────────────────────
    modo = state.get("modo", MODE_MANUAL)
    modo_color = COLOR_MODE_RUTA if modo == MODE_RUTA else COLOR_MODE_MANUAL
    card = draw_card(screen, cx, y, cw, 68, "MODO", font, small)
    mode_font = pygame.font.SysFont("Arial", 22, bold=True)
    screen.blit(mode_font.render(f"▶  {modo}", True, modo_color), (card.x + 12, card.y + 34))
    y += 82

    card = draw_card(screen, cx, y, cw, 148, "POSE", font, small)
    corr_mm  = state.get("pose_correction_mm", 0.0)
    moving   = state.get("robot_moving", False)
    corr_col = COLOR_OK if corr_mm < 5.0 else COLOR_WARN

    # Fuente activa de corrección
    if moving:
        fuente_txt = "ODOM (en movimiento)"
        fuente_col = COLOR_SUBTEXT
    else:
        fuente_txt = "ODOM + LiDAR corr."
        fuente_col = (120, 200, 255)

    info = [
        (f"X: {state['x_mm']:.1f} mm",            COLOR_SUBTEXT),
        (f"Y: {state['y_mm']:.1f} mm",            COLOR_SUBTEXT),
        (f"Yaw: {state['yaw_deg']:.2f} deg",       COLOR_SUBTEXT),
        (f"Ult DANG: {state['last_dang']:.4f}",    COLOR_SUBTEXT),
        (f"Fuente: {fuente_txt}",                  fuente_col),
        (f"Corr LiDAR: {corr_mm:.1f} mm",         corr_col),
    ]
    for i, (s, col) in enumerate(info):
        screen.blit(small.render(s, True, col), (card.x + 12, card.y + 40 + i * 17))

    y += 162   # POSE card ahora es más alta
    card = draw_card(screen, cx, y, cw, 122, "CONTROL", font, small)
    info = [
        f"PWM avance: {state['pwm_base']}",
        f"PWM giro: {state['pwm_turn']}",
        f"Motor L/R: {int(state['pl'])} / {int(state['pr'])}",
        f"Comando: W A S D",
    ]
    for i, s in enumerate(info):
        screen.blit(small.render(s, True, COLOR_SUBTEXT), (card.x + 12, card.y + 40 + i * 18))

    y += 136
    card = draw_card(screen, cx, y, cw, 122, "ACTUADORES", font, small)
    info = [
        f"Limpieza: {'ON' if state['clean_on'] else 'OFF'}",
        f"Compresor: {'ON' if state['comp_on'] else 'OFF'}",
        f"Potencia: {int(state['power'] * 100)}%",
        f"PWM auxiliar: {state['aux_pwm']}",
    ]
    for i, s in enumerate(info):
        screen.blit(small.render(s, True, COLOR_SUBTEXT), (card.x + 12, card.y + 40 + i * 18))

    y += 136
    card = draw_card(screen, cx, y, cw, 150, "MAPA / ESTADO", font, small)
    map_lines = [
        (f"Descubierto: {state['disc_pct']:.1f}%", COLOR_SUBTEXT),
        (f"Limpio: {state['clean_pct']:.1f}%", COLOR_SUBTEXT),
        (f"Alert hits: {state['dyn_count']}", COLOR_ALERT if state['dyn_alert'] else COLOR_SUBTEXT),
        (f"Dinamico: {'SI' if state['dyn_alert'] else 'NO'}", COLOR_ALERT if state['dyn_alert'] else COLOR_OK),
        (f"Arduino: {'OK' if state['arduino_ok'] else 'DESCONECTADO'}", COLOR_OK if state['arduino_ok'] else COLOR_WARN),
        (f"LiDAR: {'OK' if state['lidar_ok'] else 'DESCONECTADO'}", COLOR_OK if state['lidar_ok'] else COLOR_WARN),
    ]
    for i, (s, col) in enumerate(map_lines):
        screen.blit(small.render(s, True, col), (card.x + 12, card.y + 40 + i * 18))

    screen.blit(small.render(f"Ult RX Ard: {state['rx_age_txt']}", True, COLOR_SUBTEXT), (card.x + 12, card.y + 40 + 6 * 18))
    screen.blit(small.render(f"Ult RX LiDAR: {state['lidar_rx_txt']}", True, COLOR_SUBTEXT), (card.x + 12, card.y + 40 + 7 * 18))

    # ── RUTA ──────────────────────────────────────────────
    y += 164
    n_wps    = state.get("n_waypoints", 0)
    wp_idx   = state.get("waypoint_idx", 0)
    fsm      = state.get("fsm_state", FSM_IDLE)
    modo_now = state.get("modo", MODE_MANUAL)

    FSM_COLORS = {
        FSM_IDLE:       COLOR_SUBTEXT,
        FSM_ALIGN:      (255, 230,  80),
        FSM_ADVANCE:    ( 80, 220, 255),
        FSM_WP_REACHED: (120, 255, 140),
        FSM_BLOCKED:    (255,  70,  70),
        FSM_ROUTE_DONE: (120, 255, 140),
    }
    fsm_color = FSM_COLORS.get(fsm, COLOR_SUBTEXT)

    card = draw_card(screen, cx, y, cw, 116, "RUTA", font, small)
    screen.blit(
        small.render(f"Estado:    {fsm}", True,
                     fsm_color if modo_now == MODE_RUTA else COLOR_SUBTEXT),
        (card.x + 12, card.y + 40)
    )
    screen.blit(
        small.render(f"Waypoints: {n_wps}", True, COLOR_SUBTEXT),
        (card.x + 12, card.y + 58)
    )
    screen.blit(
        small.render(
            f"Activo:    {wp_idx + 1 if n_wps > 0 else '-'} / {n_wps if n_wps > 0 else '-'}",
            True, COLOR_WP_ACTIVE if modo_now == MODE_RUTA else COLOR_SUBTEXT
        ),
        (card.x + 12, card.y + 76)
    )
    screen.blit(
        small.render("[Click] Añadir   [RClick] Borrar ult.", True, COLOR_SUBTEXT),
        (card.x + 12, card.y + 94)
    )


def draw_help_bar(screen, font, small, state, area_rect):
    x, y, w, h = area_rect
    pygame.draw.rect(screen, COLOR_CARD, area_rect, border_radius=12)
    pygame.draw.rect(screen, COLOR_BORDER, area_rect, 1, border_radius=12)

    screen.blit(font.render("CONTROLES", True, COLOR_TEXT), (x + 14, y + 10))
    screen.blit(small.render("Se reordenaron para que siempre queden visibles en pantalla.", True, COLOR_SUBTEXT), (x + 14, y + 36))

    items = [
        ("Tab",       "Manual / Ruta"),
        ("W / S",     "Avanzar / reversa"),
        ("A / D",     "Girar"),
        ("R",         "Toggle limpieza"),
        ("O",         "Toggle compresor"),
        ("V",         "Cambiar potencia"),
        ("1 / 2",     "PWM avance - / +"),
        ("3 / 4",     "PWM giro - / +"),
        ("P",         "Reset mapa / yaw"),
        ("X",         "Stop total"),
        ("BkSp",      "Limpiar waypoints"),
    ]

    cols = 3
    inner_x = x + 14
    inner_y = y + 62
    gap_x = 12
    gap_y = 10
    box_w = (w - 28 - gap_x * (cols - 1)) // cols
    box_h = 38

    for idx, (key_txt, desc) in enumerate(items):
        col = idx % cols
        row = idx // cols
        bx = inner_x + col * (box_w + gap_x)
        by = inner_y + row * (box_h + gap_y)
        rect = pygame.Rect(bx, by, box_w, box_h)
        pygame.draw.rect(screen, COLOR_KEYBOX, rect, border_radius=10)
        pygame.draw.rect(screen, COLOR_KEYBOX_BORDER, rect, 1, border_radius=10)
        screen.blit(font.render(key_txt, True, (255, 214, 120)), (bx + 10, by + 6))
        screen.blit(small.render(desc, True, COLOR_SUBTEXT), (bx + 82, by + 10))

    legend_x = x + w - 205
    legend_y = y + 10
    legend_items = [
        (COLOR_UNDISCOVERED, "Mapa base visible"),
        (COLOR_DISCOVERED, "Descubierto"),
        (COLOR_CLEANED, "Limpio"),
        (COLOR_LIDAR, "Puntos LiDAR"),
    ]
    for i, (col, label) in enumerate(legend_items):
        ly = legend_y + i * 18
        pygame.draw.rect(screen, col, (legend_x, ly + 2, 12, 12))
        pygame.draw.rect(screen, COLOR_BORDER, (legend_x, ly + 2, 12, 12), 1)
        screen.blit(small.render(label, True, COLOR_SUBTEXT), (legend_x + 18, ly))


def get_map_rect():
    available_w = MAP_W - 2 * MARGEN
    available_h = MAP_BOTTOM - MAP_TOP
    map_size = min(available_w, available_h)
    map_x = MARGEN + (available_w - map_size) // 2
    map_y = MAP_TOP + (available_h - map_size) // 2
    return (map_x, map_y, map_size, map_size)


# ==========================================
# ESTIMACIÓN DE POSE
# ==========================================

def odometry_step(x_mm, y_mm, yaw_deg, pl_cmd, pr_cmd, dt):
    """
    Dead reckoning por cinemática diferencial.

    pl_cmd / pr_cmd : PWM con signo ya aplicado al motor (-255..255)
    yaw_deg         : heading actual del giroscopio (fuente de verdad para ángulo)
    dt              : tiempo transcurrido en segundos

    Retorna (new_x_mm, new_y_mm)
    """
    if dt <= 0.0 or dt > 0.25:
        return x_mm, y_mm

    def pwm_to_vel(pwm):
        if abs(pwm) < PWM_VEL_THRESHOLD:
            return 0.0
        return float(pwm) * VEL_MM_PER_PWM   # mm/s con signo

    v_l = pwm_to_vel(pl_cmd)
    v_r = pwm_to_vel(pr_cmd)

    v_linear = (v_l + v_r) * 0.5            # velocidad lineal del centro

    yaw_rad = math.radians(yaw_deg)
    new_x = x_mm + v_linear * math.cos(yaw_rad) * dt
    new_y = y_mm + v_linear * math.sin(yaw_rad) * dt

    # Clampear dentro del mapa
    new_x = clamp(new_x, 0.0, float(MAPA_ANCHO_MM))
    new_y = clamp(new_y, 0.0, float(MAPA_ALTO_MM))

    return new_x, new_y


def lidar_correction_step(x_mm, y_mm, hits_global,
                           ref_centroid, stationary_since,
                           now):
    """
    Corrección suave de posición usando consistencia del scan LiDAR.

    Solo actúa cuando el robot lleva LIDAR_CORR_STABLE_S segundos quieto.
    Compara el centroide actual de los hits con el centroide de referencia
    guardado en el último momento estable; aplica LIDAR_CORR_ALPHA del error.

    Parámetros
    ----------
    x_mm, y_mm       : posición estimada actual
    hits_global      : lista de (x, y) en mm del scan actual
    ref_centroid     : (cx, cy) referencia guardada, o None
    stationary_since : timestamp (time.time()) desde que el robot está quieto,
                       o None si está en movimiento
    now              : time.time() actual

    Retorna
    -------
    new_x, new_y         : posición corregida
    new_ref_centroid     : centroide de referencia actualizado (o None)
    correction_mm        : magnitud de la corrección aplicada (para debug)
    """
    # ── Robot en movimiento: limpiar referencia ────────────────
    if stationary_since is None:
        return x_mm, y_mm, None, 0.0

    # ── Aún no llevamos suficiente tiempo quietos ──────────────
    if (now - stationary_since) < LIDAR_CORR_STABLE_S:
        return x_mm, y_mm, ref_centroid, 0.0

    # ── Necesitamos suficientes hits ───────────────────────────
    if len(hits_global) < LIDAR_CORR_MIN_HITS:
        return x_mm, y_mm, ref_centroid, 0.0

    # Centroide del scan actual
    xs = [p[0] for p in hits_global]
    ys = [p[1] for p in hits_global]
    cx_now = sum(xs) / len(xs)
    cy_now = sum(ys) / len(ys)

    # Primera vez que nos quedamos quietos: guardar referencia
    if ref_centroid is None:
        return x_mm, y_mm, (cx_now, cy_now), 0.0

    # Error entre referencia y centroide actual
    ex = cx_now - ref_centroid[0]
    ey = cy_now - ref_centroid[1]
    error_mm = math.hypot(ex, ey)

    if error_mm < 1.0:
        # Sin deriva apreciable
        return x_mm, y_mm, ref_centroid, 0.0

    # Limitar la corrección máxima por tick
    scale = LIDAR_CORR_ALPHA
    if error_mm * scale > LIDAR_CORR_MAX_MM:
        scale = LIDAR_CORR_MAX_MM / error_mm

    # La deriva del centroide es opuesta al error de posición:
    # si los hits se corrieron +ex en X, el robot se fue -ex
    corr_x = clamp(x_mm - ex * scale, 0.0, float(MAPA_ANCHO_MM))
    corr_y = clamp(y_mm - ey * scale, 0.0, float(MAPA_ALTO_MM))

    correction_applied = math.hypot(corr_x - x_mm, corr_y - y_mm)

    # Actualizar referencia al centroide actual (el mapa ya se ajustó)
    return corr_x, corr_y, (cx_now, cy_now), correction_applied


# ==========================================
# FOLLOWER DE WAYPOINTS
# ==========================================
def _normalize_angle(deg):
    """Normaliza un ángulo al rango (-180, 180]."""
    while deg >  180.0: deg -= 360.0
    while deg <= -180.0: deg += 360.0
    return deg


def follow_route_step(robot_x, robot_y, yaw_deg,
                      waypoints, wp_idx, fsm_state,
                      dyn_alert):
    """
    Ejecuta un paso de la máquina de estados del follower.

    Retorna:
        pl          : PWM motor izquierdo  (-255..255)
        pr          : PWM motor derecho    (-255..255)
        new_state   : nuevo estado FSM
        new_wp_idx  : nuevo índice de waypoint
    """
    pl, pr = 0.0, 0.0

    # ── Sin waypoints ──────────────────────────────────────────────
    if not waypoints or wp_idx >= len(waypoints):
        return 0.0, 0.0, FSM_IDLE, wp_idx

    target_x, target_y = waypoints[wp_idx]

    # Distancia y heading al waypoint actual
    dx = target_x - robot_x
    dy = target_y - robot_y
    dist_mm = math.hypot(dx, dy)
    target_heading = math.degrees(math.atan2(dy, dx))
    heading_error  = _normalize_angle(target_heading - yaw_deg)

    # ══════════════════════════════════════════════════════════════
    # IDLE — arrancar si hay waypoints
    # ══════════════════════════════════════════════════════════════
    if fsm_state == FSM_IDLE:
        return 0.0, 0.0, FSM_ALIGN, wp_idx

    # ══════════════════════════════════════════════════════════════
    # ALIGN — girar en sitio hacia el waypoint
    # ══════════════════════════════════════════════════════════════
    elif fsm_state == FSM_ALIGN:
        if abs(heading_error) < ALIGN_DONE_DEG:
            # Ya está alineado → pasar a avanzar
            return 0.0, 0.0, FSM_ADVANCE, wp_idx

        # Determinar sentido de giro
        pwm = PWM_FOLLOWER_ALIGN
        if heading_error > 0:
            # Girar a la izquierda (CCW): izq atrás, der adelante
            pl, pr = -pwm, +pwm
        else:
            # Girar a la derecha (CW): izq adelante, der atrás
            pl, pr = +pwm, -pwm

        return pl, pr, FSM_ALIGN, wp_idx

    # ══════════════════════════════════════════════════════════════
    # ADVANCE — avanzar con corrección proporcional de heading
    # ══════════════════════════════════════════════════════════════
    elif fsm_state == FSM_ADVANCE:
        # Obstáculo detectado → bloquear
        if dyn_alert:
            return 0.0, 0.0, FSM_BLOCKED, wp_idx

        # Waypoint alcanzado
        if dist_mm < WP_REACH_MM:
            return 0.0, 0.0, FSM_WP_REACHED, wp_idx

        # Si el error de heading creció demasiado, re-alinear
        if abs(heading_error) > ALIGN_START_DEG * 2.5:
            return 0.0, 0.0, FSM_ALIGN, wp_idx

        # Corrección proporcional de heading
        correction = clamp(K_HEADING * heading_error,
                           -PWM_FOLLOWER_ADVANCE * 0.6,
                            PWM_FOLLOWER_ADVANCE * 0.6)

        base = PWM_FOLLOWER_ADVANCE
        pl = clamp(base - correction, PWM_FOLLOWER_MIN, 255)
        pr = clamp(base + correction, PWM_FOLLOWER_MIN, 255)

        return pl, pr, FSM_ADVANCE, wp_idx

    # ══════════════════════════════════════════════════════════════
    # WP_REACHED — avanzar al siguiente o terminar ruta
    # ══════════════════════════════════════════════════════════════
    elif fsm_state == FSM_WP_REACHED:
        next_idx = wp_idx + 1
        if next_idx >= len(waypoints):
            return 0.0, 0.0, FSM_ROUTE_DONE, wp_idx
        return 0.0, 0.0, FSM_ALIGN, next_idx

    # ══════════════════════════════════════════════════════════════
    # BLOCKED — esperar a que el obstáculo despeje
    # ══════════════════════════════════════════════════════════════
    elif fsm_state == FSM_BLOCKED:
        if not dyn_alert:
            # Obstáculo despejado → re-alinear por seguridad
            return 0.0, 0.0, FSM_ALIGN, wp_idx
        return 0.0, 0.0, FSM_BLOCKED, wp_idx

    # ══════════════════════════════════════════════════════════════
    # ROUTE_DONE — señal para que main() cambie a MANUAL
    # ══════════════════════════════════════════════════════════════
    elif fsm_state == FSM_ROUTE_DONE:
        return 0.0, 0.0, FSM_ROUTE_DONE, wp_idx

    # Fallback
    return 0.0, 0.0, FSM_IDLE, wp_idx


# ==========================================
# MAIN
# ==========================================
def main():
    pygame.init()
    screen = pygame.display.set_mode((ANCHO_VENTANA, ALTO_VENTANA))
    pygame.display.set_caption("Robot Cleaner Pro - UI + Full Grid Visible")
    clock = pygame.time.Clock()

    font = pygame.font.SysFont("Arial", 18, bold=True)
    small = pygame.font.SysFont("Consolas", 14)

    lidar = LiDAR_LD20(PUERTO_LIDAR, BAUD_LIDAR)
    arduino = ArduinoSerialController(PUERTO_ARDUINO, BAUD_ARDUINO)

    lidar_ok = lidar.connect()
    arduino_ok = arduino.connect()

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

    lidar_x_mm = LIDAR_X_INICIAL_MM
    lidar_y_mm = LIDAR_Y_INICIAL_MM

    clean_sys = False
    comp_on = False
    pwr_idx = 0

    pwm_base = PWM_BASE
    pwm_turn = PWM_GIRO

    pl = 0.0
    pr = 0.0

    # ── Modo y ruta ──────────────────────────────────────
    modo = MODE_MANUAL
    ruta_waypoints = []   # lista de (x_mm, y_mm)
    waypoint_idx   = 0   # índice del waypoint activo
    fsm_state      = FSM_IDLE

    # ── Estimación de pose ───────────────────────────────
    # lidar_x_mm / lidar_y_mm son la pose estimada (se actualizan cada tick)
    lidar_ref_centroid     = None   # centroide LiDAR de referencia (robot parado)
    lidar_stationary_since = None   # timestamp desde que el robot está quieto
    last_pl_cmd            = 0.0
    last_pr_cmd            = 0.0
    last_pose_correction   = 0.0   # magnitud de última corrección LiDAR (debug)

    last_render_time = 0.0
    robot_moving     = False   # inicializar antes del primer render

    map_rect = get_map_rect()
    help_rect = (MARGEN, ALTO_VENTANA - HELP_H - MARGEN, MAP_W - 2 * MARGEN, HELP_H)

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
                    if e.key == pygame.K_TAB:
                        modo = MODE_RUTA if modo == MODE_MANUAL else MODE_MANUAL
                        if modo == MODE_RUTA:
                            # Entrar en RUTA: detener motores, reiniciar follower
                            pl, pr = 0.0, 0.0
                            arduino.send_command(0, 0, 0, 0)
                            waypoint_idx = 0
                            fsm_state    = FSM_IDLE   # arrancará en el próximo tick
                        else:
                            # Volver a MANUAL: detener follower
                            pl, pr = 0.0, 0.0
                            arduino.send_command(0, 0, 0, 0)
                            fsm_state = FSM_IDLE
                    elif e.key == pygame.K_BACKSPACE:
                        ruta_waypoints.clear()
                        waypoint_idx = 0
                    elif e.key == pygame.K_r:
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
                    elif e.key == pygame.K_p:
                        reset_runtime_state(grid_map, lidar_mem, arduino)
                        lidar_x_mm = LIDAR_X_INICIAL_MM
                        lidar_y_mm = LIDAR_Y_INICIAL_MM
                        clean_sys = False
                        comp_on = False
                        pl = 0.0
                        pr = 0.0
                        lidar_ref_centroid     = None
                        lidar_stationary_since = None
                        last_pose_correction   = 0.0

                # ── Click del mouse para gestionar waypoints ──────────
                if e.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = e.pos
                    if grid_map.is_inside_map_rect(map_rect, mx, my):
                        if e.button == 1:   # click izquierdo → añadir
                            wx, wy = grid_map.screen_to_world(map_rect, mx, my)
                            # Clampear dentro del mapa
                            wx = clamp(wx, 0.0, float(MAPA_ANCHO_MM))
                            wy = clamp(wy, 0.0, float(MAPA_ALTO_MM))
                            ruta_waypoints.append((wx, wy))
                        elif e.button == 3: # click derecho → borrar último
                            if ruta_waypoints:
                                ruta_waypoints.pop()
                                waypoint_idx = clamp(waypoint_idx, 0, max(0, len(ruta_waypoints) - 1))

            # ── Pipeline de sensores ───────────────────────────────
            yaw_deg   = LIDAR_YAW_INICIAL_DEG + arduino.get_yaw()
            last_dang = arduino.get_last_dang()

            lidar_hits_global = []
            dyn_alert = False
            dyn_count = 0

            ang_new, dist_new = lidar.obtener_datos_nuevos()
            if len(dist_new) > 0:
                lidar_mem.update(ang_new, dist_new, now=now)

            ang, dist = lidar_mem.get_scan(now=now)
            dyn_alert, dyn_count = detect_dynamic_obstacles(ang, dist)
            lidar_hits_global   = transform_scan_to_global(
                ang, dist, lidar_x_mm, lidar_y_mm, yaw_deg
            )

            # ── Decisión de movimiento según modo ─────────────────
            if modo == MODE_MANUAL:
                keys = pygame.key.get_pressed()
                pl, pr = get_manual_drive(keys, pwm_base, pwm_turn)
            else:
                # Modo RUTA: ejecutar un paso del follower
                pl, pr, fsm_state, waypoint_idx = follow_route_step(
                    lidar_x_mm, lidar_y_mm, yaw_deg,
                    ruta_waypoints, waypoint_idx,
                    fsm_state, dyn_alert
                )

                # Ruta terminada → volver a MANUAL automáticamente
                if fsm_state == FSM_ROUTE_DONE:
                    modo      = MODE_MANUAL
                    fsm_state = FSM_IDLE
                    pl, pr    = 0.0, 0.0
                    arduino.send_command(0, 0, 0, 0)

            for xp, yp in lidar_hits_global:
                grid_map.mark_discovered(xp, yp)

            pl_cmd = pl * SIGNO_MOTOR_IZQ
            pr_cmd = pr * SIGNO_MOTOR_DER

            aux_pwm = int(255 * NIVELES_LIMPIEZA[pwr_idx]) if clean_sys else 0
            arduino.send_command(pl_cmd, pr_cmd, aux_pwm, comp_on)

            # ── Estimación de pose ─────────────────────────────────
            # 1. Dead reckoning: actualizar x,y con cinemática diferencial
            lidar_x_mm, lidar_y_mm = odometry_step(
                lidar_x_mm, lidar_y_mm, yaw_deg,
                pl_cmd, pr_cmd, dt
            )

            # 2. Detectar si el robot está parado
            robot_moving = (abs(pl_cmd) >= PWM_VEL_THRESHOLD or
                            abs(pr_cmd) >= PWM_VEL_THRESHOLD)

            if robot_moving:
                lidar_stationary_since = None
                lidar_ref_centroid     = None   # resetear referencia al moverse
            else:
                if lidar_stationary_since is None:
                    lidar_stationary_since = now

            # 3. Corrección LiDAR (solo cuando parado el tiempo suficiente)
            (lidar_x_mm, lidar_y_mm,
             lidar_ref_centroid,
             last_pose_correction) = lidar_correction_step(
                lidar_x_mm, lidar_y_mm,
                lidar_hits_global,
                lidar_ref_centroid,
                lidar_stationary_since,
                now
            )

            last_pl_cmd = pl_cmd
            last_pr_cmd = pr_cmd

            # ── Mapa ───────────────────────────────────────────────
            if clean_sys and (abs(pl) > 5 or abs(pr) > 5):
                grid_map.mark_cleaned_disk(lidar_x_mm, lidar_y_mm, radius_mm=RADIO_LIMPIEZA_MM)

            arduino_ok = arduino.is_connected()
            lidar_ok = bool(lidar and lidar.running)

            if (now - last_render_time) >= INTERVALO_DIBUJO:
                last_render_time = now

                screen.fill(COLOR_BG)

                grid_map.draw(screen, map_rect)
                grid_map.draw_lidar_hits(screen, map_rect, lidar_hits_global)
                grid_map.draw_route(screen, map_rect, ruta_waypoints, waypoint_idx, fsm_state)
                grid_map.draw_robot(screen, map_rect, lidar_x_mm, lidar_y_mm, yaw_deg, alert=dyn_alert)

                rx_age = now - arduino.last_rx_time if arduino.last_rx_time > 0 else None
                rx_age_txt = f"{rx_age:.2f}s" if rx_age is not None else "--"

                lidar_age = now - lidar.last_rx_time if lidar.last_rx_time > 0 else None
                lidar_rx_txt = f"{lidar_age:.2f}s" if lidar_age is not None else "--"

                state = {
                    "modo": modo,
                    "fsm_state": fsm_state,
                    "n_waypoints": len(ruta_waypoints),
                    "waypoint_idx": waypoint_idx,
                    "lidar_port": PUERTO_LIDAR,
                    "arduino_port": PUERTO_ARDUINO,
                    "x_mm": lidar_x_mm,
                    "y_mm": lidar_y_mm,
                    "yaw_deg": yaw_deg,
                    "pwm_base": pwm_base,
                    "pwm_turn": pwm_turn,
                    "pl": pl,
                    "pr": pr,
                    "pose_correction_mm": last_pose_correction,
                    "robot_moving": robot_moving,
                    "clean_on": clean_sys,
                    "comp_on": comp_on,
                    "power": NIVELES_LIMPIEZA[pwr_idx],
                    "aux_pwm": aux_pwm,
                    "disc_pct": grid_map.get_discovered_pct(),
                    "clean_pct": grid_map.get_cleaned_pct(),
                    "last_dang": last_dang,
                    "dyn_alert": dyn_alert,
                    "dyn_count": dyn_count,
                    "arduino_ok": arduino_ok,
                    "lidar_ok": lidar_ok,
                    "rx_age_txt": rx_age_txt,
                    "lidar_rx_txt": lidar_rx_txt,
                }

                draw_panel(screen, font, small, state)
                draw_help_bar(screen, font, small, state, help_rect)
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
