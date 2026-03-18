import math
import time
import struct
import threading
import numpy as np
import pygame
import serial
import serial.tools.list_ports
from ArduinoBridge import ArduinoLink

# ==========================================
# CONFIG
# ==========================================
BAUD_ARDUINO = 115200
BAUD_LIDAR = 230400

ANCHO_VENTANA = 1520
ALTO_VENTANA = 930
PANEL_W = 400
MAP_W = ANCHO_VENTANA - PANEL_W

FPS_OBJETIVO = 30
FPS_DIBUJO = 20
INTERVALO_DIBUJO = 1.0 / FPS_DIBUJO

# ==========================================
# MAPA GLOBAL FIJO 8x8 m
# ==========================================
MAPA_ANCHO_MM = 8000
MAPA_ALTO_MM = 8000
CELDA_MM = 100  # 80x80 grid

GRID_COLS = MAPA_ANCHO_MM // CELDA_MM
GRID_ROWS = MAPA_ALTO_MM // CELDA_MM

# Pose inicial del LIDAR (esquina inferior izquierda, con margen)
LIDAR_X_INICIAL_MM = 400.0
LIDAR_Y_INICIAL_MM = 400.0
LIDAR_YAW_INICIAL_DEG = 0.0

# ==========================================
# GEOMETRÍA ROBOT
# Referencia = centro del LiDAR
# ==========================================
ROBOT_LARGO_MM = 750
ROBOT_ANCHO_MM = 450
DIST_LIDAR_FRENTE_MM = 280
DIST_LIDAR_ATRAS_MM = ROBOT_LARGO_MM - DIST_LIDAR_FRENTE_MM
MITAD_ANCHO_MM = ROBOT_ANCHO_MM / 2

OFFSET_LIDAR_FISICO = 186.0

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

# ==========================================
# ACTUADORES
# ==========================================
NIVELES_LIMPIEZA = [0.15, 0.40, 1.00]
RADIO_LIMPIEZA_MM = 220

# ==========================================
# DETECCIÓN DINÁMICA
# ==========================================
RADIO_ALERTA_DINAMICA_MM = 500.0
UMBRAL_PUNTOS_DINAMICOS = 6

# ==========================================
# SCAN MATCHING LIGERO
# ==========================================
SCAN_SUBSAMPLE = 4
MATCH_COARSE_RANGE_MM = 220
MATCH_COARSE_STEP_MM = 40
MATCH_FINE_RANGE_MM = 60
MATCH_FINE_STEP_MM = 10
MIN_HITS_FOR_MATCH = 24

# ==========================================
# COLORES
# ==========================================
COLOR_BG = (10, 10, 18)
COLOR_PANEL = (18, 18, 28)
COLOR_GRID = (42, 48, 60)
COLOR_BORDER = (95, 105, 125)
COLOR_DISCOVERED = (32, 42, 58)
COLOR_CLEANED = (65, 120, 90)
COLOR_OCCUPIED = (170, 190, 210)
COLOR_LIDAR = (0, 255, 120)
COLOR_ROBOT = (255, 210, 0)
COLOR_LIDAR_CENTER = (0, 220, 255)
COLOR_HEADING = (255, 100, 100)
COLOR_ALERT = (255, 70, 70)
COLOR_TEXT = (220, 230, 245)
COLOR_SUBTEXT = (180, 190, 210)
COLOR_WARN = (255, 170, 90)
COLOR_BTN = (45, 55, 75)
COLOR_BTN_HOVER = (65, 80, 105)
COLOR_OK = (120, 255, 140)

# ==========================================
# UI BUTTONS
# ==========================================
class UIButton:
    def __init__(self, rect, text):
        self.rect = pygame.Rect(rect)
        self.text = text

    def draw(self, screen, font, mouse_pos):
        hovered = self.rect.collidepoint(mouse_pos)
        color = COLOR_BTN_HOVER if hovered else COLOR_BTN
        pygame.draw.rect(screen, color, self.rect, border_radius=8)
        pygame.draw.rect(screen, (110, 120, 145), self.rect, 2, border_radius=8)
        txt = font.render(self.text, True, COLOR_TEXT)
        screen.blit(txt, txt.get_rect(center=self.rect.center))

    def clicked(self, event):
        return event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and self.rect.collidepoint(event.pos)


# ==========================================
# SERIAL PORT HELPERS
# ==========================================
def list_serial_ports():
    return sorted([p.device for p in serial.tools.list_ports.comports()])


def cycle_port(current, ports, step):
    if not ports:
        return None
    if current not in ports:
        return ports[0]
    idx = ports.index(current)
    return ports[(idx + step) % len(ports)]


# ==========================================
# LIDAR PERSISTENCE
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
# LIDAR DRIVER
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
# ARDUINO BRIDGE
# ==========================================
class ArduinoBridgeController:
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

    def reset_yaw_accumulator(self):
        with self.lock:
            self.accumulated_yaw_deg = 0.0
            self.last_dang = 0.0

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
# GRID MAP + OCCUPANCY
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
        self.occupied_hits = np.zeros((self.rows, self.cols), dtype=np.uint16)

    def reset(self):
        self.discovered.fill(0)
        self.cleaned.fill(0)
        self.occupied_hits.fill(0)

    def world_to_cell(self, x_mm, y_mm):
        col = int(x_mm // self.cell_mm)
        row = int(y_mm // self.cell_mm)
        if 0 <= col < self.cols and 0 <= row < self.rows:
            return row, col
        return None

    def cell_center(self, row, col):
        x = (col + 0.5) * self.cell_mm
        y = (row + 0.5) * self.cell_mm
        return x, y

    def mark_discovered(self, x_mm, y_mm):
        idx = self.world_to_cell(x_mm, y_mm)
        if idx is not None:
            r, c = idx
            self.discovered[r, c] = 1

    def mark_occupied(self, x_mm, y_mm):
        idx = self.world_to_cell(x_mm, y_mm)
        if idx is not None:
            r, c = idx
            if self.occupied_hits[r, c] < 65000:
                self.occupied_hits[r, c] += 1

    def is_occupied_cell(self, row, col, thresh=2):
        if 0 <= row < self.rows and 0 <= col < self.cols:
            return self.occupied_hits[row, col] >= thresh
        return False

    def raytrace_and_update(self, x0_mm, y0_mm, x1_mm, y1_mm):
        dist = math.hypot(x1_mm - x0_mm, y1_mm - y0_mm)
        n = max(2, int(dist / (self.cell_mm * 0.5)))

        for i in range(n):
            t = i / max(1, n - 1)
            x = x0_mm + (x1_mm - x0_mm) * t
            y = y0_mm + (y1_mm - y0_mm) * t
            self.mark_discovered(x, y)

        self.mark_occupied(x1_mm, y1_mm)

    def mark_cleaned_disk(self, x_mm, y_mm, radius_mm=220):
        r_cells = max(1, int(radius_mm / self.cell_mm) + 1)
        center = self.world_to_cell(x_mm, y_mm)
        if center is None:
            return

        cr, cc = center
        for rr in range(cr - r_cells, cr + r_cells + 1):
            for cc2 in range(cc - r_cells, cc + r_cells + 1):
                if 0 <= rr < self.rows and 0 <= cc2 < self.cols:
                    cell_center_x, cell_center_y = self.cell_center(rr, cc2)
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

        for r in range(self.rows):
            for c in range(self.cols):
                rx = int(x0 + c * cell_w)
                ry = int(y0 + (self.rows - 1 - r) * cell_h)

                color = (22, 22, 30)
                if self.discovered[r, c]:
                    color = COLOR_DISCOVERED
                if self.cleaned[r, c]:
                    color = COLOR_CLEANED
                if self.occupied_hits[r, c] >= 2:
                    color = COLOR_OCCUPIED

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

        pygame.draw.polygon(screen, COLOR_ROBOT, pts_screen, 2)

        sx, sy = self.world_to_screen(area_rect, lidar_x_mm, lidar_y_mm)
        pygame.draw.circle(screen, COLOR_LIDAR_CENTER, (int(sx), int(sy)), 5)

        hx = lidar_x_mm + 260.0 * math.cos(yaw_rad)
        hy = lidar_y_mm + 260.0 * math.sin(yaw_rad)
        hsx, hsy = self.world_to_screen(area_rect, hx, hy)
        pygame.draw.line(screen, COLOR_HEADING, (int(sx), int(sy)), (int(hsx), int(hsy)), 3)

        # círculo de alerta dinámica 0.5 m
        rx, _ = self.world_to_screen(area_rect, lidar_x_mm + RADIO_ALERTA_DINAMICA_MM, lidar_y_mm)
        radius_px = int(abs(rx - sx))
        pygame.draw.circle(screen, COLOR_ALERT if alert else (120, 120, 140), (int(sx), int(sy)), radius_px, 1)

    def draw_lidar_hits(self, screen, area_rect, points_xy_mm):
        for x_mm, y_mm in points_xy_mm:
            if 0 <= x_mm <= self.width_mm and 0 <= y_mm <= self.height_mm:
                sx, sy = self.world_to_screen(area_rect, x_mm, y_mm)
                pygame.draw.circle(screen, COLOR_LIDAR, (int(sx), int(sy)), 2)


# ==========================================
# GEOMETRY HELPERS
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
# LIDAR-BASED POSE TRACKER
# Simplified ROS-like scan matcher using occupancy grid
# ==========================================
class PoseTracker:
    def __init__(self, x0_mm, y0_mm, yaw0_deg):
        self.x_mm = x0_mm
        self.y_mm = y0_mm
        self.yaw_deg = yaw0_deg
        self.last_match_score = 0
        self.last_match_used = False

    def reset(self, x_mm, y_mm, yaw_deg):
        self.x_mm = x_mm
        self.y_mm = y_mm
        self.yaw_deg = yaw_deg
        self.last_match_score = 0
        self.last_match_used = False

    def _score_pose(self, grid_map, scan_hits_world):
        score = 0
        for xw, yw in scan_hits_world:
            idx = grid_map.world_to_cell(xw, yw)
            if idx is None:
                continue
            r, c = idx
            if grid_map.is_occupied_cell(r, c, thresh=2):
                score += 3
            elif grid_map.discovered[r, c]:
                score += 1
        return score

    def _candidate_hits(self, angles_deg, distances_mm, x_mm, y_mm, yaw_deg):
        if len(distances_mm) == 0:
            return []

        # subsampling
        a = angles_deg[::SCAN_SUBSAMPLE]
        d = distances_mm[::SCAN_SUBSAMPLE]
        return transform_scan_to_global(a, d, x_mm, y_mm, yaw_deg)

    def update_pose_from_scan(self, grid_map, angles_deg, distances_mm, yaw_deg):
        self.yaw_deg = yaw_deg
        self.last_match_used = False
        self.last_match_score = 0

        if len(distances_mm) < MIN_HITS_FOR_MATCH:
            return self.x_mm, self.y_mm, self.yaw_deg

        occupied_cells = np.count_nonzero(grid_map.occupied_hits >= 2)
        if occupied_cells < 20:
            # todavía no hay suficiente mapa para alinear
            return self.x_mm, self.y_mm, self.yaw_deg

        # coarse search
        best_x = self.x_mm
        best_y = self.y_mm
        best_score = -1

        for dx in range(-MATCH_COARSE_RANGE_MM, MATCH_COARSE_RANGE_MM + 1, MATCH_COARSE_STEP_MM):
            for dy in range(-MATCH_COARSE_RANGE_MM, MATCH_COARSE_RANGE_MM + 1, MATCH_COARSE_STEP_MM):
                cx = self.x_mm + dx
                cy = self.y_mm + dy
                hits = self._candidate_hits(angles_deg, distances_mm, cx, cy, yaw_deg)
                sc = self._score_pose(grid_map, hits)
                if sc > best_score:
                    best_score = sc
                    best_x = cx
                    best_y = cy

        # fine search
        fine_best_x = best_x
        fine_best_y = best_y
        fine_best_score = best_score

        for dx in range(-MATCH_FINE_RANGE_MM, MATCH_FINE_RANGE_MM + 1, MATCH_FINE_STEP_MM):
            for dy in range(-MATCH_FINE_RANGE_MM, MATCH_FINE_RANGE_MM + 1, MATCH_FINE_STEP_MM):
                cx = best_x + dx
                cy = best_y + dy
                hits = self._candidate_hits(angles_deg, distances_mm, cx, cy, yaw_deg)
                sc = self._score_pose(grid_map, hits)
                if sc > fine_best_score:
                    fine_best_score = sc
                    fine_best_x = cx
                    fine_best_y = cy

        self.x_mm = clamp(fine_best_x, 0.0, MAPA_ANCHO_MM)
        self.y_mm = clamp(fine_best_y, 0.0, MAPA_ALTO_MM)
        self.last_match_score = fine_best_score
        self.last_match_used = True

        return self.x_mm, self.y_mm, self.yaw_deg


# ==========================================
# RESET
# ==========================================
def reset_runtime_state(grid_map, lidar_mem, arduino, pose_tracker):
    grid_map.reset()
    lidar_mem.reset()
    if arduino is not None:
        arduino.reset_yaw_accumulator()
    pose_tracker.reset(LIDAR_X_INICIAL_MM, LIDAR_Y_INICIAL_MM, LIDAR_YAW_INICIAL_DEG)


# ==========================================
# PANEL
# ==========================================
def draw_panel(screen, font, small, state, buttons, mouse_pos):
    x0 = MAP_W
    pygame.draw.rect(screen, COLOR_PANEL, (x0, 0, PANEL_W, ALTO_VENTANA))
    pygame.draw.line(screen, (50, 60, 80), (x0, 0), (x0, ALTO_VENTANA), 2)

    def txt(y, s, color=COLOR_TEXT):
        screen.blit(font.render(s, True, color), (x0 + 16, y))

    def txts(y, s, color=COLOR_SUBTEXT):
        screen.blit(small.render(s, True, color), (x0 + 16, y))

    txt(12, "ROBOT CLEANER PRO", (255, 210, 120))
    txts(42, f"LiDAR COM: {state['lidar_port']}")
    txts(64, f"Arduino COM: {state['arduino_port']}")
    txts(86, f"Puertos: {', '.join(state['ports']) if state['ports'] else 'ninguno'}")

    pygame.draw.line(screen, (50, 60, 80), (x0 + 16, 116), (x0 + PANEL_W - 16, 116), 1)

    txt(132, "POSE GLOBAL (ref = centro LiDAR)")
    txts(160, f"X: {state['x_mm']:.1f} mm")
    txts(182, f"Y: {state['y_mm']:.1f} mm")
    txts(204, f"Yaw: {state['yaw_deg']:.2f} deg")
    txts(226, f"Scan match score: {state['match_score']}")
    txts(248, f"Match used: {'YES' if state['match_used'] else 'NO'}")

    pygame.draw.line(screen, (50, 60, 80), (x0 + 16, 278), (x0 + PANEL_W - 16, 278), 1)

    txt(294, "CONTROL MANUAL")
    txts(322, f"PWM avance: {state['pwm_base']}")
    txts(344, f"PWM giro: {state['pwm_turn']}")
    txts(366, f"Motor L/R: {int(state['pl'])} / {int(state['pr'])}")

    pygame.draw.line(screen, (50, 60, 80), (x0 + 16, 396), (x0 + PANEL_W - 16, 396), 1)

    txt(412, "ACTUADORES")
    txts(440, f"Limpieza: {'ON' if state['clean_on'] else 'OFF'}")
    txts(462, f"Compresor: {'ON' if state['comp_on'] else 'OFF'}")
    txts(484, f"Potencia: {int(state['power']*100)}%")
    txts(506, f"PWM auxiliar: {state['aux_pwm']}")

    pygame.draw.line(screen, (50, 60, 80), (x0 + 16, 536), (x0 + PANEL_W - 16, 536), 1)

    txt(552, "MAPA 8x8 m")
    txts(580, f"Descubierto: {state['disc_pct']:.1f}%")
    txts(602, f"Limpio: {state['clean_pct']:.1f}%")
    txts(624, f"Ult DANG: {state['last_dang']:.4f}")
    txts(646, f"Alert hits: {state['dyn_count']}", COLOR_ALERT if state['dyn_alert'] else COLOR_SUBTEXT)
    txts(668, f"ALERTA DINAMICA: {'YES' if state['dyn_alert'] else 'NO'}", COLOR_ALERT if state['dyn_alert'] else COLOR_OK)

    arduino_col = COLOR_OK if state['arduino_ok'] else COLOR_WARN
    lidar_col = COLOR_OK if state['lidar_ok'] else COLOR_WARN
    txts(690, f"Arduino: {'OK' if state['arduino_ok'] else 'DESCONECTADO'}", arduino_col)
    txts(712, f"LiDAR: {'OK' if state['lidar_ok'] else 'DESCONECTADO'}", lidar_col)
    txts(734, f"Ult RX: {state['rx_age_txt']}")

    for btn in buttons:
        btn.draw(screen, small, mouse_pos)

    txt(820, "TECLAS")
    tips = [
        "W/S: avanzar / reversa",
        "A/D: girar",
        "R: toggle limpieza",
        "O: toggle compresor",
        "V: cambiar potencia",
        "1/2: PWM -/+ avance",
        "3/4: PWM -/+ giro",
        "L/K: COM LiDAR +/-",
        "U/J: COM Arduino +/-",
        "F5: refrescar puertos",
        "P: reset logico",
        "X: stop total",
    ]
    y = 848
    for t in tips:
        screen.blit(small.render(t, True, (170, 180, 200)), (x0 + 16, y))
        y += 18


# ==========================================
# MAIN
# ==========================================
def main():
    pygame.init()
    screen = pygame.display.set_mode((ANCHO_VENTANA, ALTO_VENTANA))
    pygame.display.set_caption("Robot Cleaner Pro - LiDAR-based pose")
    clock = pygame.time.Clock()

    font = pygame.font.SysFont("Arial", 18, bold=True)
    small = pygame.font.SysFont("Consolas", 14)

    available_ports = list_serial_ports()
    selected_lidar_port = available_ports[0] if available_ports else None
    selected_arduino_port = available_ports[1] if len(available_ports) > 1 else selected_lidar_port

    lidar = None
    arduino = None
    lidar_ok = False
    arduino_ok = False

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
    pose_tracker = PoseTracker(LIDAR_X_INICIAL_MM, LIDAR_Y_INICIAL_MM, LIDAR_YAW_INICIAL_DEG)

    clean_sys = False
    comp_on = False
    pwr_idx = 0

    pwm_base = PWM_BASE
    pwm_turn = PWM_GIRO

    pl = 0.0
    pr = 0.0

    last_render_time = 0.0

    map_size = min(MAP_W - 40, ALTO_VENTANA - 40)
    map_x = 20
    map_y = (ALTO_VENTANA - map_size) // 2
    map_rect = (map_x, map_y, map_size, map_size)

    btn_refresh = UIButton((MAP_W + 16, 760, 110, 34), "Refrescar")
    btn_connect = UIButton((MAP_W + 136, 760, 110, 34), "Conectar")
    btn_disconnect = UIButton((MAP_W + 256, 760, 110, 34), "Cerrar")
    btn_reset = UIButton((MAP_W + 16, 804, 350, 34), "Reset mapa / pose / yaw")
    buttons = [btn_refresh, btn_connect, btn_disconnect, btn_reset]

    try:
        while True:
            dt = clock.tick(FPS_OBJETIVO) / 1000.0
            if dt <= 1e-6:
                dt = 1e-3

            now = time.time()
            mouse_pos = pygame.mouse.get_pos()

            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    raise KeyboardInterrupt

                if btn_refresh.clicked(e):
                    available_ports = list_serial_ports()
                    if available_ports:
                        if selected_lidar_port not in available_ports:
                            selected_lidar_port = available_ports[0]
                        if selected_arduino_port not in available_ports:
                            selected_arduino_port = available_ports[0]

                elif btn_connect.clicked(e):
                    try:
                        if lidar:
                            lidar.close()
                    except Exception:
                        pass
                    try:
                        if arduino:
                            arduino.close()
                    except Exception:
                        pass

                    lidar = None
                    arduino = None
                    lidar_ok = False
                    arduino_ok = False

                    if selected_lidar_port:
                        lidar = LiDAR_LD20(selected_lidar_port, BAUD_LIDAR)
                        lidar_ok = lidar.connect()

                    if selected_arduino_port:
                        arduino = ArduinoBridgeController(selected_arduino_port, BAUD_ARDUINO)
                        arduino_ok = arduino.connect()

                elif btn_disconnect.clicked(e):
                    try:
                        if lidar:
                            lidar.close()
                    except Exception:
                        pass
                    try:
                        if arduino:
                            arduino.close()
                    except Exception:
                        pass
                    lidar = None
                    arduino = None
                    lidar_ok = False
                    arduino_ok = False

                elif btn_reset.clicked(e):
                    reset_runtime_state(grid_map, lidar_mem, arduino, pose_tracker)
                    clean_sys = False
                    comp_on = False
                    pl = 0.0
                    pr = 0.0

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
                        if arduino:
                            arduino.send_command(0, 0, 0, 0)
                    elif e.key == pygame.K_F5:
                        available_ports = list_serial_ports()
                    elif e.key == pygame.K_l:
                        selected_lidar_port = cycle_port(selected_lidar_port, available_ports, +1)
                    elif e.key == pygame.K_k:
                        selected_lidar_port = cycle_port(selected_lidar_port, available_ports, -1)
                    elif e.key == pygame.K_u:
                        selected_arduino_port = cycle_port(selected_arduino_port, available_ports, +1)
                    elif e.key == pygame.K_j:
                        selected_arduino_port = cycle_port(selected_arduino_port, available_ports, -1)
                    elif e.key == pygame.K_p:
                        reset_runtime_state(grid_map, lidar_mem, arduino, pose_tracker)
                        clean_sys = False
                        comp_on = False
                        pl = 0.0
                        pr = 0.0

            keys = pygame.key.get_pressed()
            pl, pr = get_manual_drive(keys, pwm_base, pwm_turn)

            aux_pwm = int(255 * NIVELES_LIMPIEZA[pwr_idx]) if clean_sys else 0
            if arduino:
                arduino.send_command(pl, pr, aux_pwm, comp_on)

            yaw_deg = LIDAR_YAW_INICIAL_DEG + (arduino.get_yaw() if arduino else 0.0)
            last_dang = arduino.get_last_dang() if arduino else 0.0

            lidar_hits_global = []
            dyn_alert = False
            dyn_count = 0

            if lidar and lidar_ok:
                ang_new, dist_new = lidar.obtener_datos_nuevos()
                if len(dist_new) > 0:
                    lidar_mem.update(ang_new, dist_new, now=now)

                ang, dist = lidar_mem.get_scan(now=now)

                dyn_alert, dyn_count = detect_dynamic_obstacles(ang, dist)

                # Pose estimation from LiDAR + gyro
                x_mm, y_mm, yaw_deg = pose_tracker.update_pose_from_scan(grid_map, ang, dist, yaw_deg)

                lidar_hits_global = transform_scan_to_global(ang, dist, x_mm, y_mm, yaw_deg)

                for xp, yp in lidar_hits_global:
                    grid_map.raytrace_and_update(x_mm, y_mm, xp, yp)

                if clean_sys and (abs(pl) > 5 or abs(pr) > 5):
                    grid_map.mark_cleaned_disk(x_mm, y_mm, radius_mm=RADIO_LIMPIEZA_MM)

            else:
                x_mm = pose_tracker.x_mm
                y_mm = pose_tracker.y_mm

            arduino_ok = arduino.is_connected() if arduino else False
            lidar_ok = bool(lidar and lidar.running)

            if (now - last_render_time) >= INTERVALO_DIBUJO:
                last_render_time = now

                screen.fill(COLOR_BG)

                grid_map.draw(screen, map_rect)
                grid_map.draw_lidar_hits(screen, map_rect, lidar_hits_global)
                grid_map.draw_robot(screen, map_rect, pose_tracker.x_mm, pose_tracker.y_mm, yaw_deg, alert=dyn_alert)

                rx_age = now - arduino.last_rx_time if (arduino and arduino.last_rx_time > 0) else None
                rx_age_txt = f"{rx_age:.2f}s" if rx_age is not None else "--"

                state = {
                    "lidar_port": selected_lidar_port or "None",
                    "arduino_port": selected_arduino_port or "None",
                    "ports": available_ports,
                    "x_mm": pose_tracker.x_mm,
                    "y_mm": pose_tracker.y_mm,
                    "yaw_deg": yaw_deg,
                    "match_score": pose_tracker.last_match_score,
                    "match_used": pose_tracker.last_match_used,
                    "pwm_base": pwm_base,
                    "pwm_turn": pwm_turn,
                    "pl": pl,
                    "pr": pr,
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
                }

                draw_panel(screen, font, small, state, buttons, mouse_pos)
                pygame.display.flip()

    except KeyboardInterrupt:
        pass

    finally:
        try:
            if arduino:
                arduino.send_command(0, 0, 0, 0)
        except Exception:
            pass

        try:
            if lidar:
                lidar.close()
        except Exception:
            pass

        try:
            if arduino:
                arduino.close()
        except Exception:
            pass

        pygame.quit()


if __name__ == "__main__":
    main()