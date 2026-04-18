import math

# Calculadas tras definir WHEEL_DIAMETER_MM (al final del bloque de constantes)
# Se definen aquí como forward-declaration; se sobrescriben abajo
WHEEL_CIRCUM_MM  = math.pi * 76.2   # perímetro rueda (actualizar si cambia WHEEL_DIAMETER_MM)
MM_PER_DEG_WHEEL = WHEEL_CIRCUM_MM / 360.0  # mm por grado de giro del eje
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

TOOLBAR_H  = 40                      # barra fina de estado
MARGEN     = 10
MAP_TOP    = TOOLBAR_H + MARGEN
MAP_BOTTOM = ALTO_VENTANA - MARGEN

FPS_OBJETIVO = 30
FPS_DIBUJO = 20
INTERVALO_DIBUJO = 1.0 / FPS_DIBUJO

# ==========================================
# MAPA FIJO 5x5 m
# ==========================================
MAPA_ANCHO_MM = 5000
MAPA_ALTO_MM = 5000
CELDA_MM = 200

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
ROBOT_LARGO_MM = 650
ROBOT_ANCHO_MM = 400
DIST_LIDAR_FRENTE_MM = 150
DIST_LIDAR_ATRAS_MM = ROBOT_LARGO_MM - DIST_LIDAR_FRENTE_MM
MITAD_ANCHO_MM = ROBOT_ANCHO_MM / 2

OFFSET_LIDAR_FISICO = 186.0

# ==========================================
# ODOMETRÍA / ESTIMACIÓN DE POSE
# ==========================================
WHEEL_TRACK_MM     = 395.0   # separación centro-centro ruedas motrices (medida real)

# ── Encoders AS5600 ──────────────────────────────────
# Mide el diámetro real con cinta métrica:
# marca la rueda, hazla rodar 1 vuelta completa → mide la distancia → diámetro = dist/π
WHEEL_DIAMETER_MM  = 76.2    # ← CALIBRAR con cinta métrica
# WHEEL_CIRCUM_MM se calcula después de importar math (ver abajo)

# Umbral mínimo de delta (deg) para considerar que la rueda se movió
ENC_DELTA_THRESHOLD_DEG = 0.5

# Fallback PWM (activo solo si no hay datos de encoder disponibles)
VEL_MM_PER_PWM     = 3.5
PWM_VEL_THRESHOLD  = 30

# Corrección LiDAR (solo cuando el robot está parado)
LIDAR_CORR_ALPHA        = 0.10   # fracción del error que se corrige por tick
LIDAR_CORR_MIN_HITS     = 10     # mínimo de hits para confiar en el centroide
LIDAR_CORR_MAX_MM       = 80.0   # corrección máxima permitida por tick (anti-jump)
LIDAR_CORR_STABLE_S     = 0.30   # segundos quieto antes de activar corrección

# ==========================================
# SCAN MATCHING — localización continua
# ==========================================
MATCH_EVERY_N_TICKS     = 5      # correr matching cada N ticks (~6 Hz a 30 FPS)
MATCH_MAX_CORR_MM       = 250.0  # distancia máx hit↔pared para par válido
MATCH_MIN_PAIRS         = 8      # pares mínimos para confiar en el resultado
MATCH_ALPHA             = 0.40   # fracción de corrección aplicada por llamada
MATCH_MAX_CORRECTION_MM = 120.0  # corrección máxima por llamada (anti-salto)

# ==========================================
# FASE D — CLASIFICADOR GEOMÉTRICO + BLOQUEO FRONTAL
# ==========================================

# Segmentación de hits
SEG_CLUSTER_GAP_MM   = 200.0  # distancia máx entre hits consecutivos del mismo segmento
SEG_MIN_POINTS       = 5      # puntos mínimos para analizar un segmento
SEG_LINE_RESIDUAL_MM = 55.0   # residual máx RMS para considerar segmento "recto" (pared)
                               # por encima → irregular → objeto dinámico

# Cono de detección frontal
FRONT_HALF_ANGLE_DEG = 35.0   # semiángulo del cono (±35° respecto al heading)
FRONT_MAX_DIST_MM    = 400.0  # distancia máxima dentro del cono

# Confirmación / despejado de bloqueo
BLOCK_MIN_IRREG_HITS = 5      # hits irregulares mínimos en el cono por tick
BLOCK_CONFIRM_TICKS  = 12     # ticks consecutivos con obstáculo → BLOCKED
BLOCK_CLEAR_TICKS    = 6      # ticks consecutivos sin obstáculo → despejado

# Colores de visualización
COLOR_SEG_DYNAMIC    = (255,  70,  70)  # rojo   — segmento irregular/dinámico
COLOR_SEG_STATIC     = ( 70, 200, 120)  # verde  — segmento recto (pared)
COLOR_FRONT_CONE     = (255, 160,  30)  # naranja — arco del cono frontal

# ==========================================
# FASE E — GRILLA COMO FILTRO DE SEGURIDAD
# ==========================================
PATH_CHECK_EVERY_N   = 10
COLOR_WP_SKIPPED     = (110, 110, 120)
COLOR_PATH_BLOCKED   = (200,  50,  50)

# ==========================================
# PATRONES DE LIMPIEZA AUTOMÁTICA
# ==========================================
PATRON_NINGUNO   = "NINGUNO"
PATRON_MATRICIAL = "MATRICIAL"
PATRON_ESPIRAL   = "ESPIRAL"
PATRON_BOWTIE    = "BOW-TIE"

PATRONES_CICLO   = [PATRON_NINGUNO, PATRON_MATRICIAL,
                    PATRON_ESPIRAL, PATRON_BOWTIE]

# Paso entre pasadas = diámetro de limpieza exacto (sin solapamiento)
# RADIO_LIMPIEZA_MM = 220 → diámetro = 440 mm
PASO_LIMPIEZA_MM = 440   # mm  (= RADIO_LIMPIEZA_MM * 2)

# Margen desde las paredes para no chocar
MARGEN_PARED_MM  = 450

# Colores de patrón en el panel
COLOR_PATRON = {
    PATRON_NINGUNO:   (130, 130, 140),
    PATRON_MATRICIAL: ( 80, 200, 255),
    PATRON_ESPIRAL:   (160, 120, 255),
    PATRON_BOWTIE:    (255, 180,  60),
}

# ==========================================
# REPLANNING DE LIMPIEZA
# ==========================================
CLEAN_THRESHOLD_DEFAULT = 90.0   # % de cobertura para considerar "limpio"
CLEAN_THRESHOLD_STEP    =  5.0   # paso al ajustar con [ / ]
CLEAN_THRESHOLD_MIN     = 50.0
CLEAN_THRESHOLD_MAX     = 100.0

# Bordes de suciedad para selección de patrón automático
REPLAN_BORDER_RATIO_ESPIRAL  = 0.55  # >55% sucio en bordes → ESPIRAL
REPLAN_ASPECT_RATIO_MATRICIAL = 1.4  # zona alargada → MATRICIAL

COLOR_CLEANABLE   = (220, 235, 255)  # azul muy suave — área limpiable sin limpiar
# COLOR_CLEAN_DONE reutiliza COLOR_CLEANED (definido en sección colores, más abajo)
COLOR_THRESH_OK   = ( 80, 220, 130)  # verde — por encima del umbral
COLOR_THRESH_WARN = (255, 180,  60)  # naranja — por debajo del umbral

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
PWM_BASE = 100
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
MODE_SCAN   = "SCAN"

# ==========================================
# MAPEO INICIAL DE PAREDES
# ==========================================
SCAN_DURATION_S      = 4.0   # segundos acumulando hits
SCAN_MIN_HITS        = 3     # hits mínimos por celda para confirmarla como pared
COLOR_WALL           = ( 90,  50,  50)   # rojo oscuro — celda de pared
COLOR_WALL_BORDER    = (180,  80,  80)   # borde de pared
COLOR_SCAN_PROGRESS  = ( 80, 160, 255)   # barra de progreso del scan
COLOR_MODE_SCAN      = (255, 130,  50)   # naranja — modo scan en panel

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
RADIO_ALERTA_DINAMICA_MM = 400.0
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

# Toolbar
COLOR_TOOLBAR_BG     = (22, 24, 36)
COLOR_TOOLBAR_BORDER = (45, 50, 70)
COLOR_TB_BTN         = (32, 36, 52)
COLOR_TB_BTN_HOVER   = (44, 50, 72)
COLOR_TB_BTN_ACTIVE  = (55, 90, 140)
COLOR_TB_BTN_DANGER  = (100, 28, 28)
COLOR_TB_BTN_OK      = (28, 80, 48)
COLOR_TB_SEP         = (55, 62, 82)
COLOR_TB_GROUP_LABEL = (90, 100, 125)
COLOR_TB_KEY         = (255, 210, 80)

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
        self.last_dang           = 0.0
        self.last_rx_time        = 0.0
        self.last_command        = "CMD:0,0,0,0"

        # ── Encoders AS5600 ───────────────────────────────
        self.enc_acum_izq   = 0.0   # ángulo acumulado rueda izq (grados)
        self.enc_acum_der   = 0.0   # ángulo acumulado rueda der (grados)
        self.enc_delta_izq  = 0.0   # delta desde último reporte (grados)
        self.enc_delta_der  = 0.0   # delta desde último reporte (grados)
        self.enc_available  = False # True después del primer mensaje ENC:

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

                    elif line.startswith("ENC:"):
                        # Formato: ENC:acum_izq,acum_der,delta_izq,delta_der
                        try:
                            parts = line[4:].split(",")
                            if len(parts) == 4:
                                acum_i  = float(parts[0])
                                acum_d  = float(parts[1])
                                delta_i = float(parts[2])
                                delta_d = float(parts[3])
                                with self.lock:
                                    self.enc_acum_izq  = acum_i
                                    self.enc_acum_der  = acum_d
                                    self.enc_delta_izq = delta_i
                                    self.enc_delta_der = delta_d
                                    self.enc_available = True
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

    def get_encoder_deltas(self):
        """Retorna (delta_izq_deg, delta_der_deg, available).
        Los deltas son los acumulados desde el último reporte del Arduino (20ms).
        """
        with self.lock:
            return self.enc_delta_izq, self.enc_delta_der, self.enc_available

    def get_encoder_accum(self):
        """Retorna (acum_izq_deg, acum_der_deg)."""
        with self.lock:
            return self.enc_acum_izq, self.enc_acum_der

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
        self.cleaned    = np.zeros((self.rows, self.cols), dtype=np.uint8)
        self.walls      = np.zeros((self.rows, self.cols), dtype=np.uint8)
        self.cleanable  = np.zeros((self.rows, self.cols), dtype=np.uint8)

    def reset(self):
        self.discovered.fill(0)
        self.cleaned.fill(0)
        # walls y cleanable NO se borran con reset general

    def clear_walls(self):
        self.walls.fill(0)
        self.cleanable.fill(0)   # invalidar máscara al re-escanear

    def compute_cleanable_mask(self, bbox):
        """
        Marca como 'cleanable' todas las celdas dentro del bbox
        de limpieza que no son pared. Llámala tras cada escaneo F1.
        bbox: (x_min, y_min, x_max, y_max) en mm
        """
        self.cleanable.fill(0)
        if bbox is None:
            return

        x_min, y_min, x_max, y_max = bbox
        c_min = max(0, int(x_min // self.cell_mm))
        c_max = min(self.cols - 1, int(x_max // self.cell_mm))
        r_min = max(0, int(y_min // self.cell_mm))
        r_max = min(self.rows - 1, int(y_max // self.cell_mm))

        for r in range(r_min, r_max + 1):
            for c in range(c_min, c_max + 1):
                if not self.walls[r, c]:
                    self.cleanable[r, c] = 1

    def has_cleanable_mask(self):
        return bool(np.any(self.cleanable))

    def get_cleanable_count(self):
        return int(np.sum(self.cleanable))

    def get_clean_coverage_pct(self):
        """
        % de celdas limpias respecto al área limpiable total.
        Retorna 0.0 si no hay máscara cleanable.
        """
        total = int(np.sum(self.cleanable))
        if total == 0:
            return 0.0
        cleaned_in_area = int(np.sum(self.cleaned & self.cleanable))
        return 100.0 * cleaned_in_area / total

    def get_uncleaned_cell_count(self):
        """Celdas limpiabes que todavía no han sido limpiadas."""
        return int(np.sum(self.cleanable & (1 - self.cleaned)))

    def get_uncleaned_bbox(self):
        """
        Bounding box de las celdas cleanable que aún no están limpias.
        Retorna (x_min, y_min, x_max, y_max) en mm, o None si todo limpio.
        """
        dirty = self.cleanable & (1 - self.cleaned)   # (rows, cols)
        idxs  = np.argwhere(dirty)
        if len(idxs) == 0:
            return None

        r_min = int(idxs[:, 0].min())
        r_max = int(idxs[:, 0].max())
        c_min = int(idxs[:, 1].min())
        c_max = int(idxs[:, 1].max())

        # Convertir a mm con margen interno para no ir al filo de celdas
        x_min = c_min * self.cell_mm + MARGEN_PARED_MM
        y_min = r_min * self.cell_mm + MARGEN_PARED_MM
        x_max = (c_max + 1) * self.cell_mm - MARGEN_PARED_MM
        y_max = (r_max + 1) * self.cell_mm - MARGEN_PARED_MM

        if x_min >= x_max or y_min >= y_max:
            return None

        return (x_min, y_min, x_max, y_max)

    def get_uncleaned_cells_array(self):
        """
        Retorna array numpy (N×2) con centros [x_mm, y_mm] de celdas
        cleanable no limpias. Usado por select_best_pattern().
        """
        dirty = self.cleanable & (1 - self.cleaned)
        idxs  = np.argwhere(dirty)
        if len(idxs) == 0:
            return None
        xs = (idxs[:, 1] + 0.5) * self.cell_mm
        ys = (idxs[:, 0] + 0.5) * self.cell_mm
        return np.column_stack([xs, ys]).astype(np.float32)

    def has_wall_map(self):
        """True si ya se hizo al menos un escaneo de paredes."""
        return bool(np.any(self.walls))

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

    def mark_wall(self, x_mm, y_mm):
        idx = self.world_to_cell(x_mm, y_mm)
        if idx is not None:
            r, c = idx
            self.walls[r, c] = 1

    def is_wall_cell(self, x_mm, y_mm):
        idx = self.world_to_cell(x_mm, y_mm)
        if idx is None:
            return False
        r, c = idx
        return bool(self.walls[r, c])

    def nearest_free_cell_mm(self, x_mm, y_mm, max_radius_cells=8):
        """
        Si (x_mm, y_mm) cae en una celda de pared, devuelve las coordenadas
        del centro de la celda libre más cercana usando búsqueda espiral.
        Si ya es libre, devuelve (x_mm, y_mm) sin cambios.
        """
        idx = self.world_to_cell(x_mm, y_mm)
        if idx is None:
            return x_mm, y_mm

        r0, c0 = idx
        if not self.walls[r0, c0]:
            return x_mm, y_mm   # ya es libre

        # Búsqueda espiral por anillos crecientes
        for radius in range(1, max_radius_cells + 1):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if abs(dr) != radius and abs(dc) != radius:
                        continue   # solo el borde del anillo
                    rr, cc = r0 + dr, c0 + dc
                    if 0 <= rr < self.rows and 0 <= cc < self.cols:
                        if not self.walls[rr, cc]:
                            # Centro de celda en mm
                            cx = (cc + 0.5) * self.cell_mm
                            cy = (rr + 0.5) * self.cell_mm
                            return cx, cy

        # Sin celda libre encontrada → devolver original
        return x_mm, y_mm

    def get_wall_count(self):
        return int(np.sum(self.walls))

    def path_crosses_wall(self, x0_mm, y0_mm, x1_mm, y1_mm):
        """
        Rasteriza el segmento (x0,y0)→(x1,y1) con Bresenham y comprueba
        si alguna celda atravesada es una pared.
        Retorna True si el camino está bloqueado por una pared.
        """
        # Convertir extremos a celdas
        def to_cell(x, y):
            return int(x // self.cell_mm), int(y // self.cell_mm)

        c0, r0 = to_cell(x0_mm, y0_mm)
        c1, r1 = to_cell(x1_mm, y1_mm)

        # Bresenham entero
        dc = abs(c1 - c0)
        dr = abs(r1 - r0)
        sc = 1 if c1 > c0 else -1
        sr = 1 if r1 > r0 else -1
        err = dc - dr

        c, r = c0, r0
        while True:
            # Comprobar celda actual
            if 0 <= r < self.rows and 0 <= c < self.cols:
                if self.walls[r, c]:
                    return True
            if c == c1 and r == r1:
                break
            e2 = 2 * err
            if e2 > -dr:
                err -= dr
                c   += sc
            if e2 < dc:
                err += dc
                r   += sr

        return False

    def get_wall_centers_xy(self):
        """
        Retorna un array numpy (N×2) con las coordenadas del centro
        de cada celda de pared, en mm. Listo para usar en scan matching.
        Retorna None si no hay paredes.
        """
        idxs = np.argwhere(self.walls)   # shape (N, 2): [[row, col], ...]
        if len(idxs) == 0:
            return None
        # Centro de celda: (col + 0.5) * cell_mm, (row + 0.5) * cell_mm
        xs = (idxs[:, 1] + 0.5) * self.cell_mm
        ys = (idxs[:, 0] + 0.5) * self.cell_mm
        return np.column_stack([xs, ys]).astype(np.float32)

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
                cw = int(cell_w) + 1
                ch = int(cell_h) + 1

                if self.walls[r, c]:
                    pygame.draw.rect(screen, COLOR_WALL, (rx, ry, cw, ch))
                elif self.cleaned[r, c]:
                    pygame.draw.rect(screen, COLOR_CLEANED, (rx, ry, cw, ch))
                elif self.cleanable[r, c]:
                    pygame.draw.rect(screen, COLOR_CLEANABLE, (rx, ry, cw, ch))
                elif self.discovered[r, c]:
                    pygame.draw.rect(screen, COLOR_DISCOVERED, (rx, ry, cw, ch))
                # else: COLOR_UNDISCOVERED ya pintado arriba

        # Líneas de grilla
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

        # Alert circle centrado en el cuerpo del robot (no en el LiDAR)
        # El centro geométrico del cuerpo está a (FRENTE - ATRAS)/2 desde el LiDAR
        body_offset = (DIST_LIDAR_FRENTE_MM - DIST_LIDAR_ATRAS_MM) / 2.0
        body_cx = lidar_x_mm + body_offset * math.cos(yaw_rad)
        body_cy = lidar_y_mm + body_offset * math.sin(yaw_rad)
        bsx, bsy = self.world_to_screen(area_rect, body_cx, body_cy)
        rx, _ = self.world_to_screen(area_rect, body_cx + RADIO_ALERTA_DINAMICA_MM, body_cy)
        radius_px = int(abs(rx - bsx))
        pygame.draw.circle(screen, COLOR_ALERT if alert else (135, 135, 150),
                           (int(bsx), int(bsy)), radius_px, 1)

    def draw_lidar_hits(self, screen, area_rect, points_xy_mm):
        for x_mm, y_mm in points_xy_mm:
            if 0 <= x_mm <= self.width_mm and 0 <= y_mm <= self.height_mm:
                sx, sy = self.world_to_screen(area_rect, x_mm, y_mm)
                pygame.draw.circle(screen, COLOR_LIDAR, (int(sx), int(sy)), 2)

    def draw_classified_hits(self, screen, area_rect,
                              dynamic_hits, static_hits):
        """Dibuja hits clasificados: verde=pared, rojo=dinámico."""
        for x_mm, y_mm in static_hits:
            if 0 <= x_mm <= self.width_mm and 0 <= y_mm <= self.height_mm:
                sx, sy = self.world_to_screen(area_rect, x_mm, y_mm)
                pygame.draw.circle(screen, COLOR_SEG_STATIC,
                                   (int(sx), int(sy)), 2)
        for x_mm, y_mm in dynamic_hits:
            if 0 <= x_mm <= self.width_mm and 0 <= y_mm <= self.height_mm:
                sx, sy = self.world_to_screen(area_rect, x_mm, y_mm)
                pygame.draw.circle(screen, COLOR_SEG_DYNAMIC,
                                   (int(sx), int(sy)), 3)

    def draw_frontal_cone(self, screen, area_rect,
                          robot_x, robot_y, yaw_deg, blocked=False):
        """
        Dibuja el arco del cono frontal de detección de obstáculos.
        Color naranja normal, rojo pulsante si está bloqueado.
        """
        sx, sy = self.world_to_screen(area_rect, robot_x, robot_y)

        # Radio del cono en píxeles
        rx, _ = self.world_to_screen(area_rect,
                                     robot_x + FRONT_MAX_DIST_MM, robot_y)
        radius_px = max(4, int(abs(rx - sx)))

        color = COLOR_ALERT if blocked else COLOR_FRONT_CONE

        # Dibujar arco con líneas radiales
        yaw_rad  = math.radians(yaw_deg)
        half_rad = math.radians(FRONT_HALF_ANGLE_DEG)

        steps = 18
        prev_pt = None
        for i in range(steps + 1):
            a = yaw_rad - half_rad + (2 * half_rad * i / steps)
            px = int(sx + radius_px * math.cos(a))
            py = int(sy - radius_px * math.sin(a))
            if prev_pt:
                pygame.draw.line(screen, color, prev_pt, (px, py), 1)
            prev_pt = (px, py)

        # Líneas laterales del cono
        for sign in (+1, -1):
            a  = yaw_rad + sign * half_rad
            ex = int(sx + radius_px * math.cos(a))
            ey = int(sy - radius_px * math.sin(a))
            pygame.draw.line(screen, color, (int(sx), int(sy)), (ex, ey), 1)

    def screen_to_world(self, area_rect, sx, sy):
        """Convierte coordenadas de pantalla a coordenadas globales en mm."""
        x0, y0, w, h = area_rect
        x_mm = (sx - x0) / w * self.width_mm
        y_mm = (1.0 - (sy - y0) / h) * self.height_mm
        return x_mm, y_mm

    def is_inside_map_rect(self, area_rect, sx, sy):
        x0, y0, w, h = area_rect
        return x0 <= sx <= x0 + w and y0 <= sy <= y0 + h

    def draw_route(self, screen, area_rect, waypoints, active_idx,
                   fsm_state=FSM_IDLE, skipped_indices=None):
        """
        Dibuja la ruta de waypoints sobre el mapa.
        waypoints       : lista de (x_mm, y_mm)
        active_idx      : índice del waypoint objetivo actual
        fsm_state       : estado actual del follower
        skipped_indices : set de índices de waypoints saltados por pared
        """
        if not waypoints:
            return

        if skipped_indices is None:
            skipped_indices = set()

        pts_screen = [
            self.world_to_screen(area_rect, x, y)
            for x, y in waypoints
        ]

        # ── Líneas entre waypoints ─────────────────────────────
        if len(pts_screen) >= 2:
            for i in range(len(pts_screen) - 1):
                p1 = (int(pts_screen[i][0]),   int(pts_screen[i][1]))
                p2 = (int(pts_screen[i+1][0]), int(pts_screen[i+1][1]))

                if i < active_idx:
                    color_line = COLOR_WP_DONE
                elif (i in skipped_indices or i + 1 in skipped_indices):
                    color_line = COLOR_PATH_BLOCKED   # tramo bloqueado
                else:
                    color_line = COLOR_RUTA_LINE

                # Tramo bloqueado: línea discontinua (guiones)
                if color_line == COLOR_PATH_BLOCKED:
                    dx = p2[0] - p1[0]
                    dy = p2[1] - p1[1]
                    seg_len = math.hypot(dx, dy)
                    if seg_len > 0:
                        steps = max(1, int(seg_len / 8))
                        for s in range(steps):
                            if s % 2 == 0:
                                fa = s / steps
                                fb = (s + 0.8) / steps
                                pa = (int(p1[0] + dx * fa), int(p1[1] + dy * fa))
                                pb = (int(p1[0] + dx * fb), int(p1[1] + dy * fb))
                                pygame.draw.line(screen, color_line, pa, pb, 2)
                else:
                    pygame.draw.line(screen, color_line, p1, p2, 2)

        # ── Color del waypoint activo según FSM ───────────────
        if fsm_state == FSM_BLOCKED:
            active_color = COLOR_WP_BLOCKED
        elif fsm_state == FSM_ALIGN:
            active_color = (255, 230, 80)
        else:
            active_color = COLOR_WP_ACTIVE

        # ── Puntos de waypoints ───────────────────────────────
        lbl_font = pygame.font.SysFont("Consolas", 11)
        for i, (sx, sy) in enumerate(pts_screen):
            if i in skipped_indices:
                color = COLOR_WP_SKIPPED
                r = WP_RADIO_PX - 1
            elif i < active_idx:
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

            # Tachar waypoints saltados con una X
            if i in skipped_indices:
                d = r - 1
                pygame.draw.line(screen, (200, 80, 80),
                    (int(sx)-d, int(sy)-d), (int(sx)+d, int(sy)+d), 2)
                pygame.draw.line(screen, (200, 80, 80),
                    (int(sx)+d, int(sy)-d), (int(sx)-d, int(sy)+d), 2)

            lbl = lbl_font.render(str(i + 1), True, (255, 255, 255))
            screen.blit(lbl, (int(sx) + r + 2, int(sy) - 7))


# ==========================================
# RESET
# ==========================================
def do_replan(grid_map):
    """
    Ejecuta el replanning: genera nueva ruta sobre áreas sin limpiar.
    Retorna (waypoints, patron_elegido, replan_succeeded).
    """
    wps, pat = replan_cleaning(grid_map)
    if not wps:
        return [], PATRON_NINGUNO, False
    return wps, pat, True


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
    if modo == MODE_RUTA:
        modo_color = COLOR_MODE_RUTA
    elif modo == MODE_SCAN:
        modo_color = COLOR_MODE_SCAN
    else:
        modo_color = COLOR_MODE_MANUAL

    scan_progress = state.get("scan_progress", 0.0)   # 0.0 – 1.0
    scan_walls    = state.get("scan_wall_count", 0)
    has_walls     = state.get("has_wall_map", False)

    card_h = 110 if modo == MODE_SCAN else 68
    card = draw_card(screen, cx, y, cw, card_h, "MODO", font, small)
    mode_font = pygame.font.SysFont("Arial", 22, bold=True)
    screen.blit(mode_font.render(f"▶  {modo}", True, modo_color),
                (card.x + 12, card.y + 34))

    if modo == MODE_SCAN:
        # Barra de progreso
        bar_x  = card.x + 12
        bar_y  = card.y + 62
        bar_w  = cw - 24
        bar_h  = 12
        pygame.draw.rect(screen, (40, 44, 60), (bar_x, bar_y, bar_w, bar_h), border_radius=4)
        fill_w = int(bar_w * scan_progress)
        if fill_w > 0:
            pygame.draw.rect(screen, COLOR_SCAN_PROGRESS,
                             (bar_x, bar_y, fill_w, bar_h), border_radius=4)
        pygame.draw.rect(screen, COLOR_BORDER, (bar_x, bar_y, bar_w, bar_h), 1, border_radius=4)
        screen.blit(small.render(
            f"{int(scan_progress * 100)}%  —  {scan_walls} celdas pared",
            True, COLOR_SUBTEXT), (bar_x, bar_y + 16))
    else:
        wall_txt = f"Paredes: {scan_walls} celdas" if has_walls else "Sin mapa de paredes"
        wall_col = COLOR_OK if has_walls else COLOR_WARN
        screen.blit(small.render(wall_txt, True, wall_col),
                    (card.x + 12, card.y + 54))

    y += card_h + 14

    card = draw_card(screen, cx, y, cw, 148, "POSE", font, small)
    corr_mm    = state.get("pose_correction_mm", 0.0)
    match_qual = state.get("match_quality", 0.0)
    has_walls  = state.get("has_wall_map", False)
    enc_ok     = state.get("enc_available", False)

    if has_walls:
        fuente_txt = "SCAN MATCH" + (" + ENC" if enc_ok else "")
        fuente_col = (120, 200, 255)
        corr_col   = COLOR_OK if match_qual >= 50 else COLOR_WARN
    elif enc_ok:
        fuente_txt = "ENCODERS"
        fuente_col = COLOR_OK
        corr_col   = COLOR_SUBTEXT
    else:
        fuente_txt = "CONGELADA (sin sensores)"
        fuente_col = (140, 140, 150)
        corr_col   = COLOR_SUBTEXT

    enc_acum_i = state.get("enc_acum_izq", 0.0)
    enc_acum_d = state.get("enc_acum_der", 0.0)

    info = [
        (f"X: {state['x_mm']:.1f} mm",               COLOR_SUBTEXT),
        (f"Y: {state['y_mm']:.1f} mm",               COLOR_SUBTEXT),
        (f"Yaw: {state['yaw_deg']:.2f} deg",          COLOR_SUBTEXT),
        (f"Fuente: {fuente_txt}",                     fuente_col),
        (f"Match: {match_qual:.0f}%  corr:{corr_mm:.1f}mm", corr_col),
        (f"Enc I:{enc_acum_i:.0f}°  D:{enc_acum_d:.0f}°",
             COLOR_OK if enc_ok else (80, 80, 90)),
    ]
    for i, (s, col) in enumerate(info):
        screen.blit(small.render(s, True, col),
                    (card.x + 12, card.y + 40 + i * 17))

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
    fb    = state.get("frontal_blocked", False)
    fc    = state.get("frontal_count", 0)
    dyn_t = state.get("dyn_count_total", 0)
    map_lines = [
        (f"Descubierto: {state['disc_pct']:.1f}%",   COLOR_SUBTEXT),
        (f"Limpio: {state['clean_pct']:.1f}%",        COLOR_SUBTEXT),
        (f"Hits dinámicos: {dyn_t}",                  COLOR_ALERT if dyn_t > 0 else COLOR_SUBTEXT),
        (f"Cono frontal: {fc} hits  {'BLOQUEADO' if fb else 'libre'}",
             COLOR_ALERT if fb else COLOR_OK),
        (f"Arduino: {'OK' if state['arduino_ok'] else 'DESCONECTADO'}",
             COLOR_OK if state["arduino_ok"] else COLOR_WARN),
        (f"LiDAR: {'OK' if state['lidar_ok'] else 'DESCONECTADO'}",
             COLOR_OK if state["lidar_ok"] else COLOR_WARN),
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

    skipped_n      = state.get("skipped_count", 0)
    patron_now     = state.get("patron_actual", PATRON_NINGUNO)
    pat_color      = COLOR_PATRON.get(patron_now, COLOR_SUBTEXT)
    coverage       = state.get("clean_coverage", 0.0)
    threshold      = state.get("clean_threshold", CLEAN_THRESHOLD_DEFAULT)
    replan_n       = state.get("replan_count", 0)
    clean_done     = state.get("cleaning_complete", False)
    cleanable_n    = state.get("cleanable_count", 0)
    uncleaned_n    = state.get("uncleaned_count", 0)

    card = draw_card(screen, cx, y, cw, 230, "RUTA / LIMPIEZA", font, small)

    # ── Patrón y estado FSM ─────────────────────────────
    screen.blit(small.render(f"Patrón: {patron_now}", True, pat_color),
                (card.x + 12, card.y + 38))
    screen.blit(
        small.render(f"Estado: {fsm}", True,
                     fsm_color if modo_now == MODE_RUTA else COLOR_SUBTEXT),
        (card.x + 12, card.y + 54))
    screen.blit(
        small.render(
            f"WP: {wp_idx+1 if n_wps>0 else '-'}/{n_wps if n_wps>0 else '-'}"
            f"  saltados:{skipped_n}",
            True, COLOR_WARN if skipped_n > 0 else COLOR_SUBTEXT),
        (card.x + 12, card.y + 70))

    # ── Barra de cobertura ──────────────────────────────
    bar_x  = card.x + 12
    bar_y  = card.y + 92
    bar_w  = cw - 24
    bar_h  = 14
    cover_f = clamp(coverage / 100.0, 0.0, 1.0)
    thresh_f = clamp(threshold / 100.0, 0.0, 1.0)

    # Fondo
    pygame.draw.rect(screen, (35, 38, 52), (bar_x, bar_y, bar_w, bar_h), border_radius=4)
    # Relleno de cobertura
    fill_w = int(bar_w * cover_f)
    fill_col = COLOR_THRESH_OK if coverage >= threshold else COLOR_THRESH_WARN
    if fill_w > 0:
        pygame.draw.rect(screen, fill_col, (bar_x, bar_y, fill_w, bar_h), border_radius=4)
    # Línea de umbral
    thresh_x = bar_x + int(bar_w * thresh_f)
    pygame.draw.line(screen, (255, 255, 255),
                     (thresh_x, bar_y - 2), (thresh_x, bar_y + bar_h + 2), 2)
    # Borde
    pygame.draw.rect(screen, COLOR_BORDER, (bar_x, bar_y, bar_w, bar_h), 1, border_radius=4)

    # Texto de cobertura
    cov_txt = "✓ COMPLETA" if clean_done else f"{coverage:.1f}%"
    cov_col = COLOR_THRESH_OK if coverage >= threshold else COLOR_THRESH_WARN
    screen.blit(small.render(
        f"Cobertura: {cov_txt}  (umbral {threshold:.0f}%)", True, cov_col),
        (bar_x, bar_y + bar_h + 4))

    # Celdas
    screen.blit(small.render(
        f"Limpiabl.: {cleanable_n}  Sucias: {uncleaned_n}",
        True, COLOR_SUBTEXT), (bar_x, bar_y + bar_h + 20))

    # Replanning
    rp_col = COLOR_WARN if replan_n > 0 else COLOR_SUBTEXT
    screen.blit(small.render(f"Replans: {replan_n}",
        True, rp_col), (bar_x, bar_y + bar_h + 36))

    # Controles
    screen.blit(small.render(
        "[F2] Patrón  [F3] Replan  [ ] Umbral",
        True, COLOR_SUBTEXT), (bar_x, bar_y + bar_h + 54))
    screen.blit(small.render(
        "[Tab] Activar ruta  [Click] WP manual",
        True, COLOR_SUBTEXT), (bar_x, bar_y + bar_h + 70))


# Rect fijo del botón Controls (guardado entre frames)
_CTRL_BTN_RECT = None


def draw_toolbar(screen, font, small, state, dropdown_open=False):
    """Barra delgada de estado + botón Controls ▾ que abre el dropdown."""
    global _CTRL_BTN_RECT
    tw = MAP_W
    th = TOOLBAR_H
    pygame.draw.rect(screen, COLOR_TOOLBAR_BG, (0, 0, tw, th))
    pygame.draw.line(screen, COLOR_TOOLBAR_BORDER, (0, th-1), (tw, th-1), 1)

    modo       = state.get("modo", MODE_MANUAL)
    fsm        = state.get("fsm_state", FSM_IDLE)
    coverage   = state.get("clean_coverage", 0.0)
    threshold  = state.get("clean_threshold", CLEAN_THRESHOLD_DEFAULT)
    patron_now = state.get("patron_actual", PATRON_NINGUNO)
    enc_ok     = state.get("enc_available", False)
    has_walls  = state.get("has_wall_map", False)
    blocked    = state.get("frontal_blocked", False)
    clean_done = state.get("cleaning_complete", False)
    cy = th // 2

    def pill(text, color, bg, x):
        surf = small.render(text, True, color)
        w = surf.get_rect().width + 14
        h = th - 8
        r = pygame.Rect(x, (th-h)//2, w, h)
        pygame.draw.rect(screen, bg, r, border_radius=5)
        pygame.draw.rect(screen, COLOR_TOOLBAR_BORDER, r, 1, border_radius=5)
        screen.blit(surf, (x+7, cy - surf.get_rect().height//2))
        return x + w + 5

    cx = 8

    # Mode pill
    if   modo == MODE_SCAN: m_col, m_bg = COLOR_MODE_SCAN,   (30,45,70)
    elif modo == MODE_RUTA: m_col, m_bg = COLOR_MODE_RUTA,   (20,35,65)
    else:                   m_col, m_bg = COLOR_MODE_MANUAL, (22,40,28)
    cx = pill(f"  {modo}  ", m_col, m_bg, cx)

    # FSM pill (RUTA only)
    if modo == MODE_RUTA:
        fc = {FSM_ALIGN:(255,220,60), FSM_ADVANCE:(80,220,255),
              FSM_BLOCKED:(255,70,70), FSM_IDLE:COLOR_SUBTEXT,
              FSM_WP_REACHED:(120,255,140)}.get(fsm, COLOR_SUBTEXT)
        cx = pill(fsm, fc, (22,24,38), cx)

    pygame.draw.line(screen, COLOR_TB_SEP, (cx,6), (cx,th-6), 1); cx += 8

    # Coverage bar
    if has_walls:
        bar_w = 100; bar_h = 10
        bx, by = cx, cy - bar_h//2
        pygame.draw.rect(screen, (35,38,52), (bx,by,bar_w,bar_h), border_radius=3)
        fw = int(bar_w * min(coverage/100.0, 1.0))
        fc = COLOR_THRESH_OK if coverage >= threshold else COLOR_THRESH_WARN
        if fw > 0: pygame.draw.rect(screen, fc, (bx,by,fw,bar_h), border_radius=3)
        tx = bx + int(bar_w * threshold/100.0)
        pygame.draw.line(screen, (255,255,255), (tx,by-1), (tx,by+bar_h+1), 1)
        pygame.draw.rect(screen, COLOR_TOOLBAR_BORDER, (bx,by,bar_w,bar_h), 1, border_radius=3)
        cx += bar_w + 4
        cov_txt = "✓ DONE" if clean_done else f"{coverage:.0f}%"
        cs = small.render(cov_txt, True, fc)
        screen.blit(cs, (cx, cy - cs.get_rect().height//2)); cx += cs.get_rect().width + 8
        pygame.draw.line(screen, COLOR_TB_SEP, (cx,6), (cx,th-6), 1); cx += 8

    # Patron pill
    if patron_now != PATRON_NINGUNO:
        ps = {"MATRICIAL":"≡≡","ESPIRAL":"◎","BOW-TIE":"✕"}.get(patron_now, patron_now)
        pc = COLOR_PATRON.get(patron_now, COLOR_SUBTEXT)
        cx = pill(f"{ps} {patron_now}", pc, (22,24,38), cx)

    # Sensor source pill
    if has_walls and enc_ok:   src, sc = "MATCH+ENC", (80,160,255)
    elif enc_ok:               src, sc = "ENCODERS",  COLOR_OK
    else:                      src, sc = "NO SENSORS",(140,140,150)
    cx = pill(src, sc, (22,24,38), cx)

    if blocked: cx = pill("⬛ BLOCKED", COLOR_ALERT, (60,18,18), cx)

    # Controls button — right side
    btn_txt  = "⌨  Controls  ▾" if not dropdown_open else "⌨  Controls  ▴"
    bs = small.render(btn_txt, True, COLOR_TEXT)
    bw = bs.get_rect().width + 20
    bh = th - 8
    bx = tw - bw - 8
    by = (th - bh) // 2
    bg = COLOR_TB_BTN_ACTIVE if dropdown_open else COLOR_TB_BTN
    br = pygame.Rect(bx, by, bw, bh)
    pygame.draw.rect(screen, bg, br, border_radius=6)
    bc = (100,160,255) if dropdown_open else COLOR_TOOLBAR_BORDER
    pygame.draw.rect(screen, bc, br, 1, border_radius=6)
    screen.blit(bs, (bx+10, cy - bs.get_rect().height//2))
    _CTRL_BTN_RECT = br


def draw_controls_dropdown(screen, small):
    """Dropdown flotante con atajos y leyenda de colores."""
    items = [
        ("F1","Scan paredes"),     ("F2","Ciclar patrón"),
        ("F3","Replan forzado"),   ("[ ]","Umbral cobertura ±"),
        ("Tab","Manual / Ruta"),   ("W/S","Adelante / atrás"),
        ("A/D","Girar"),           ("R","Toggle limpieza"),
        ("O","Toggle compresor"),  ("V","Cambiar potencia"),
        ("1/2","PWM avance ±"),    ("3/4","PWM giro ±"),
        ("E","Exportar mapa PNG"), ("P","Reset mapa/yaw"),
        ("BkSp","Limpiar waypoints"),("X","STOP total"),
        ("Click","Añadir waypoint"),("RClick","Borrar último WP"),
    ]
    col_w = 190; row_h = 22; cols = 2
    rows  = math.ceil(len(items)/cols)
    pad   = 12
    dw    = col_w*cols + pad*2
    dh    = rows*row_h + pad*2 + 26
    dx    = MAP_W - dw - 4
    dy    = TOOLBAR_H

    pygame.draw.rect(screen, (18,20,32), (dx,dy,dw,dh), border_radius=8)
    pygame.draw.rect(screen, COLOR_TOOLBAR_BORDER, (dx,dy,dw,dh), 1, border_radius=8)

    legend = [(COLOR_UNDISCOVERED,"Base"),(COLOR_CLEANABLE,"Limpiable"),
              (COLOR_CLEANED,"Limpio"),(COLOR_WALL,"Pared"),(COLOR_LIDAR,"LiDAR")]
    lx = dx+pad; ly = dy+pad
    for col_sw, lbl in legend:
        pygame.draw.rect(screen, col_sw, (lx, ly+2, 10, 10))
        pygame.draw.rect(screen, COLOR_BORDER, (lx, ly+2, 10, 10), 1)
        ls = small.render(lbl, True, COLOR_SUBTEXT)
        screen.blit(ls, (lx+13, ly)); lx += 13+ls.get_rect().width+8
    pygame.draw.line(screen, COLOR_TB_SEP, (dx+pad,dy+pad+18), (dx+dw-pad,dy+pad+18), 1)

    for i, (key, desc) in enumerate(items):
        col = i % cols; row = i // cols
        ix = dx+pad+col*col_w; iy = dy+pad+26+row*row_h
        screen.blit(small.render(key,  True, COLOR_TB_KEY),    (ix,    iy))
        screen.blit(small.render(desc, True, COLOR_SUBTEXT),   (ix+52, iy))


def export_cleaning_map(grid_map, coverage_pct, base_path="."):
    """Exporta el mapa de limpieza como PNG con leyenda y cobertura."""
    import os, datetime
    EXPORT_CELL = 8; LEG_H = 60
    IMG_W = grid_map.cols * EXPORT_CELL
    IMG_H = grid_map.rows * EXPORT_CELL + LEG_H
    surf = pygame.Surface((IMG_W, IMG_H))
    surf.fill((10,10,18))
    for r in range(grid_map.rows):
        for c in range(grid_map.cols):
            px = c * EXPORT_CELL
            py = (grid_map.rows-1-r) * EXPORT_CELL
            if   grid_map.walls[r,c]:     col = COLOR_WALL
            elif grid_map.cleaned[r,c]:   col = COLOR_CLEANED
            elif grid_map.cleanable[r,c]: col = COLOR_CLEANABLE
            elif grid_map.discovered[r,c]:col = COLOR_DISCOVERED
            else:                         col = COLOR_UNDISCOVERED
            pygame.draw.rect(surf, col, (px, py, EXPORT_CELL, EXPORT_CELL))
    ly = grid_map.rows * EXPORT_CELL
    pygame.draw.rect(surf, (18,20,32), (0,ly,IMG_W,LEG_H))
    try:    ef = pygame.font.SysFont("Consolas", 13)
    except: ef = pygame.font.SysFont(None, 13)
    legend = [(COLOR_UNDISCOVERED,"Sin desc."),(COLOR_CLEANABLE,"Limpiable"),
              (COLOR_CLEANED,"Limpiado"),(COLOR_WALL,"Pared"),(COLOR_DISCOVERED,"Descub.")]
    lx = 8
    for col_sw, lbl in legend:
        pygame.draw.rect(surf, col_sw, (lx,ly+8,14,14))
        pygame.draw.rect(surf, (80,80,100),(lx,ly+8,14,14),1)
        ls = ef.render(lbl, True, (200,200,215)); surf.blit(ls,(lx+18,ly+8))
        lx += 18+ls.get_rect().width+14
    fc = COLOR_THRESH_OK if coverage_pct >= 80 else COLOR_THRESH_WARN
    cs = ef.render(f"Cobertura: {coverage_pct:.1f}%", True, fc)
    surf.blit(cs, (IMG_W-cs.get_rect().width-10, ly+8))
    ts = ef.render(datetime.datetime.now().strftime("Exportado: %Y-%m-%d %H:%M:%S"),
                   True, (100,105,120))
    surf.blit(ts, (8, ly+30))
    fname = os.path.join(base_path, f"cleaning_map_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.png")
    pygame.image.save(surf, fname)
    return fname


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
    """Dead reckoning por PWM — fallback cuando no hay encoders."""
    if dt <= 0.0 or dt > 0.25:
        return x_mm, y_mm

    def pwm_to_vel(pwm):
        if abs(pwm) < PWM_VEL_THRESHOLD:
            return 0.0
        return float(pwm) * VEL_MM_PER_PWM

    v_l = pwm_to_vel(pl_cmd)
    v_r = pwm_to_vel(pr_cmd)
    v_linear = (v_l + v_r) * 0.5
    yaw_rad = math.radians(yaw_deg)
    new_x = clamp(x_mm + v_linear * math.cos(yaw_rad) * dt, 0.0, float(MAPA_ANCHO_MM))
    new_y = clamp(y_mm + v_linear * math.sin(yaw_rad) * dt, 0.0, float(MAPA_ALTO_MM))
    return new_x, new_y


def odometry_encoder_step(x_mm, y_mm, yaw_deg,
                           delta_izq_deg, delta_der_deg):
    """
    Odometría diferencial con encoders AS5600.

    Usa el yaw del giroscopio (ya acumulado externamente) como heading
    real — no recalcula el ángulo desde los encoders, lo que evita
    el drift rotacional que tienen los encoders solos en superficies
    irregulares como un corral.

    delta_izq_deg, delta_der_deg : grados girados por cada rueda
                                   desde el último reporte del Arduino

    Retorna (new_x_mm, new_y_mm)
    """
    # Filtrar ruido de quietud
    if (abs(delta_izq_deg) < ENC_DELTA_THRESHOLD_DEG and
            abs(delta_der_deg) < ENC_DELTA_THRESHOLD_DEG):
        return x_mm, y_mm

    # Distancia lineal recorrida por cada rueda
    dist_izq = delta_izq_deg * MM_PER_DEG_WHEEL   # mm, con signo
    dist_der = delta_der_deg * MM_PER_DEG_WHEEL

    # Desplazamiento lineal del centro del robot
    v_linear = (dist_izq + dist_der) * 0.5

    # Proyectar sobre el heading actual del giroscopio
    yaw_rad = math.radians(yaw_deg)
    new_x = clamp(x_mm + v_linear * math.cos(yaw_rad),
                  0.0, float(MAPA_ANCHO_MM))
    new_y = clamp(y_mm + v_linear * math.sin(yaw_rad),
                  0.0, float(MAPA_ALTO_MM))
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
# FASE D — CLASIFICADOR GEOMÉTRICO DE HITS
# ==========================================

def _fit_line_residual(points_xy):
    """
    Ajusta una línea recta a un conjunto de puntos 2D (numpy array Nx2).
    Retorna el RMS del residual perpendicular en mm.
    Un valor bajo → puntos alineados (pared).
    Un valor alto → puntos dispersos/curvos (objeto dinámico).
    """
    if len(points_xy) < 2:
        return 0.0

    xs = points_xy[:, 0]
    ys = points_xy[:, 1]

    # Centrar
    cx, cy = np.mean(xs), np.mean(ys)
    dxs = xs - cx
    dys = ys - cy

    # Dirección principal por PCA (eigenvector del mayor autovalor)
    cov = np.array([[np.dot(dxs, dxs), np.dot(dxs, dys)],
                    [np.dot(dxs, dys), np.dot(dys, dys)]]) / len(points_xy)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    # Normal a la línea = eigenvector del MENOR autovalor
    normal = eigenvectors[:, 0]

    # Distancia perpendicular de cada punto a la línea
    perp_dist = dxs * normal[0] + dys * normal[1]
    rms = float(np.sqrt(np.mean(perp_dist ** 2)))
    return rms


def classify_hits(hits_global, robot_x, robot_y, yaw_deg):
    """
    Clasifica los hits LiDAR en segmentos estáticos (paredes) o
    dinámicos (objetos irregulares como animales).

    Pasos:
      1. Convertir hits a coordenadas relativas al robot
      2. Ordenar por ángulo
      3. Agrupar en segmentos por gap de distancia
      4. Ajustar línea a cada segmento → residual decide si es pared o dinámico

    Retorna
    -------
    dynamic_hits  : list of (x_mm, y_mm) — hits de segmentos dinámicos
    static_hits   : list of (x_mm, y_mm) — hits de segmentos rectos (paredes)
    segments_info : list of dict con metadatos para debug/render
    """
    if len(hits_global) < SEG_MIN_POINTS:
        return [], list(hits_global), []

    arr = np.array(hits_global, dtype=np.float32)   # (M, 2)

    # ── Coordenadas relativas al robot ────────────────────────
    rel = arr - np.array([robot_x, robot_y], dtype=np.float32)

    # Ángulo y distancia de cada hit respecto al robot
    angles  = np.degrees(np.arctan2(rel[:, 1], rel[:, 0]))
    dists   = np.hypot(rel[:, 0], rel[:, 1])

    # Ordenar por ángulo para agrupar hits contiguos
    order   = np.argsort(angles)
    arr_s   = arr[order]
    rel_s   = rel[order]

    # ── Segmentación por gap espacial ─────────────────────────
    # Gap entre hits consecutivos en el espacio global
    gaps     = np.hypot(np.diff(arr_s[:, 0]), np.diff(arr_s[:, 1]))
    split_at = np.where(gaps > SEG_CLUSTER_GAP_MM)[0] + 1
    segments = np.split(arr_s, split_at)

    dynamic_hits  = []
    static_hits   = []
    segments_info = []

    for seg in segments:
        if len(seg) < SEG_MIN_POINTS:
            # Segmento demasiado pequeño → tratar como estático por precaución
            static_hits.extend(seg.tolist())
            continue

        residual = _fit_line_residual(seg)
        is_dynamic = residual > SEG_LINE_RESIDUAL_MM

        info = {
            "points": seg,
            "residual_mm": residual,
            "is_dynamic": is_dynamic,
            "centroid": seg.mean(axis=0).tolist(),
        }
        segments_info.append(info)

        if is_dynamic:
            dynamic_hits.extend(seg.tolist())
        else:
            static_hits.extend(seg.tolist())

    return dynamic_hits, static_hits, segments_info


def check_frontal_obstacle(dynamic_hits, robot_x, robot_y, yaw_deg):
    """
    Cuenta cuántos hits dinámicos caen dentro del cono frontal del robot.

    Cono definido por:
      - semiángulo FRONT_HALF_ANGLE_DEG respecto al heading actual
      - distancia máxima FRONT_MAX_DIST_MM

    Retorna (n_frontal_hits, frontal_hit_list)
    """
    if not dynamic_hits:
        return 0, []

    yaw_rad   = math.radians(yaw_deg)
    half_ang  = math.radians(FRONT_HALF_ANGLE_DEG)

    frontal = []
    for xh, yh in dynamic_hits:
        dx  = xh - robot_x
        dy  = yh - robot_y
        d   = math.hypot(dx, dy)
        if d > FRONT_MAX_DIST_MM:
            continue

        ang_to_hit  = math.atan2(dy, dx)
        ang_diff    = abs(math.atan2(
            math.sin(ang_to_hit - yaw_rad),
            math.cos(ang_to_hit - yaw_rad)
        ))
        if ang_diff <= half_ang:
            frontal.append((xh, yh))

    return len(frontal), frontal


# ==========================================
# SCAN MATCHING — localización por paredes
# ==========================================

def scan_matching_step(x_mm, y_mm, hits_global, wall_centers_xy):
    """
    Nearest-neighbor scan matching vectorizado con numpy.

    Para cada hit LiDAR en coordenadas globales busca la celda de pared
    más cercana. Si la distancia es menor a MATCH_MAX_CORR_MM, forma un
    par válido. El error medio de todos los pares se aplica como corrección
    proporcional a la pose estimada.

    Parámetros
    ----------
    x_mm, y_mm       : pose estimada actual
    hits_global      : lista de (x, y) en mm del scan actual
    wall_centers_xy  : np.array (N×2) de centros de celdas de pared (mm)
                       — None si no hay mapa de paredes

    Retorna
    -------
    new_x, new_y     : pose corregida
    quality_pct      : % de hits que encontraron correspondencia (0–100)
    correction_mm    : magnitud de la corrección aplicada (mm)
    """
    # Sin mapa de paredes o sin hits → no hacer nada
    if wall_centers_xy is None or len(hits_global) < MATCH_MIN_PAIRS:
        return x_mm, y_mm, 0.0, 0.0

    # Convertir hits a array numpy (M×2)
    hits_arr = np.array(hits_global, dtype=np.float32)   # (M, 2)

    # ── Nearest-neighbor vectorizado ──────────────────────────────
    # hits_arr[:, None, :] → (M, 1, 2)
    # wall_centers_xy[None, :, :] → (1, N, 2)
    # diff → (M, N, 2) , dist2 → (M, N)
    diff  = hits_arr[:, None, :] - wall_centers_xy[None, :, :]   # (M, N, 2)
    dist2 = np.sum(diff ** 2, axis=2)                             # (M, N)

    # Para cada hit, índice y distancia de la pared más cercana
    nn_idx  = np.argmin(dist2, axis=1)                            # (M,)
    nn_dist = np.sqrt(dist2[np.arange(len(hits_arr)), nn_idx])    # (M,)

    # Filtrar por umbral de correspondencia
    valid_mask = nn_dist < MATCH_MAX_CORR_MM
    n_valid    = int(np.sum(valid_mask))
    quality    = 100.0 * n_valid / len(hits_arr)

    if n_valid < MATCH_MIN_PAIRS:
        return x_mm, y_mm, quality, 0.0

    # ── Calcular error medio de correspondencias válidas ──────────
    valid_hits  = hits_arr[valid_mask]                        # (K, 2)
    valid_walls = wall_centers_xy[nn_idx[valid_mask]]         # (K, 2)

    # El robot está en una posición tal que sus hits se alinean con las paredes.
    # Si hits están desplazados respecto a las paredes, la pose está desviada
    # en la misma dirección → corregir restando ese error.
    mean_error = np.mean(valid_hits - valid_walls, axis=0)    # (2,) [dx, dy]
    error_mm   = float(np.linalg.norm(mean_error))

    if error_mm < 1.0:
        return x_mm, y_mm, quality, 0.0

    # Escalar corrección
    scale = MATCH_ALPHA
    if error_mm * scale > MATCH_MAX_CORRECTION_MM:
        scale = MATCH_MAX_CORRECTION_MM / error_mm

    new_x = clamp(x_mm - float(mean_error[0]) * scale,
                  0.0, float(MAPA_ANCHO_MM))
    new_y = clamp(y_mm - float(mean_error[1]) * scale,
                  0.0, float(MAPA_ALTO_MM))

    correction_applied = math.hypot(new_x - x_mm, new_y - y_mm)
    return new_x, new_y, quality, correction_applied


# ==========================================
# FOLLOWER DE WAYPOINTS
# ==========================================
def _normalize_angle(deg):
    """Normaliza un ángulo al rango (-180, 180]."""
    while deg >  180.0: deg -= 360.0
    while deg <= -180.0: deg += 360.0
    return deg


# ==========================================
# GENERADORES DE PATRONES DE LIMPIEZA
# ==========================================

def get_cleaning_bbox(grid_map, margin_mm=MARGEN_PARED_MM):
    """
    Calcula el bounding box interior del área limpiable a partir
    de las celdas de pared escaneadas.
    Retorna (x_min, y_min, x_max, y_max) en mm, o None si no hay paredes.
    """
    if not grid_map.has_wall_map():
        return None

    idxs = np.argwhere(grid_map.walls)   # (N, 2) [row, col]
    if len(idxs) == 0:
        return None

    row_min = int(idxs[:, 0].min())
    row_max = int(idxs[:, 0].max())
    col_min = int(idxs[:, 1].min())
    col_max = int(idxs[:, 1].max())

    # Convertir a mm (borde interior de cada celda de pared)
    wall_x_min = (col_min + 1) * grid_map.cell_mm
    wall_y_min = (row_min + 1) * grid_map.cell_mm
    wall_x_max =  col_max      * grid_map.cell_mm
    wall_y_max =  row_max      * grid_map.cell_mm

    x_min = wall_x_min + margin_mm
    y_min = wall_y_min + margin_mm
    x_max = wall_x_max - margin_mm
    y_max = wall_y_max - margin_mm

    if x_min >= x_max or y_min >= y_max:
        return None

    return (x_min, y_min, x_max, y_max)


def generate_matricial(bbox, paso_mm=PASO_LIMPIEZA_MM):
    """
    Genera waypoints en patrón matricial boustrophedon (serpiente).
    Filas paralelas al eje X, alternando dirección.
    """
    x_min, y_min, x_max, y_max = bbox
    waypoints = []
    y = y_min
    row = 0
    while y <= y_max + 1:
        if row % 2 == 0:
            waypoints.append((x_min, y))
            waypoints.append((x_max, y))
        else:
            waypoints.append((x_max, y))
            waypoints.append((x_min, y))
        y += paso_mm
        row += 1
    return waypoints


def generate_espiral(bbox, paso_mm=PASO_LIMPIEZA_MM):
    """
    Genera waypoints en espiral rectangular desde afuera hacia adentro.
    Cada vuelta recorre las 4 esquinas del rectángulo actual,
    luego encoge el rectángulo por paso_mm en cada lado.
    """
    x_min, y_min, x_max, y_max = bbox
    waypoints = []

    while x_min < x_max and y_min < y_max:
        # Vuelta exterior: TL → TR → BR → BL
        waypoints.append((x_min, y_max))   # TL
        waypoints.append((x_max, y_max))   # TR
        waypoints.append((x_max, y_min))   # BR
        waypoints.append((x_min, y_min))   # BL

        x_min += paso_mm
        y_min += paso_mm
        x_max -= paso_mm
        y_max -= paso_mm

    # Punto central si queda espacio
    if x_min < x_max and y_min < y_max:
        cx = (x_min + x_max) / 2
        cy = (y_min + y_max) / 2
        waypoints.append((cx, cy))

    return waypoints


def generate_bowtie(bbox, paso_mm=PASO_LIMPIEZA_MM):
    """
    Genera waypoints en patrón bow-tie (figura-8 en cuadrícula).
    Divide el área en celdas de 2*paso × 2*paso.
    En cada celda: TL → centro → TR → centro → BR → centro → BL → centro
    Esto crea patrones de X superpuestos que dan buena cobertura
    y son similares a los patrones de robots de limpieza comerciales.
    """
    x_min, y_min, x_max, y_max = bbox
    paso2 = paso_mm * 2
    waypoints = []

    y = y_min
    cell_row = 0
    while y < y_max:
        y_top = min(y + paso2, y_max)
        x = x_min
        cell_col = 0

        # Alternar dirección de columnas por fila (boustrophedon de celdas)
        xs_range = []
        cx = x_min
        while cx < x_max:
            xs_range.append(cx)
            cx += paso2

        if cell_row % 2 == 1:
            xs_range = list(reversed(xs_range))

        for x in xs_range:
            x_right = min(x + paso2, x_max)
            cy_cell  = (y + y_top) / 2
            cx_cell  = (x + x_right) / 2

            # Figura-8: 4 esquinas pasando siempre por el centro
            corners = [
                (x,       y_top),   # TL
                (x_right, y_top),   # TR
                (x_right, y),       # BR
                (x,       y),       # BL
            ]
            for corner in corners:
                waypoints.append(corner)
                waypoints.append((cx_cell, cy_cell))  # volver al centro

        y += paso2
        cell_row += 1

    return waypoints


def generate_pattern(patron, grid_map):
    """
    Punto de entrada único. Retorna lista de waypoints o [] si falla.
    """
    bbox = get_cleaning_bbox(grid_map)
    if bbox is None:
        return []

    if patron == PATRON_MATRICIAL:
        return generate_matricial(bbox)
    elif patron == PATRON_ESPIRAL:
        return generate_espiral(bbox)
    elif patron == PATRON_BOWTIE:
        return generate_bowtie(bbox)
    return []


# ==========================================
# REPLANNING INTELIGENTE DE LIMPIEZA
# ==========================================

def select_best_pattern(grid_map):
    """
    Analiza la distribución de celdas sin limpiar y elige el patrón
    más adecuado para limpiarlas eficientemente.

    Lógica de selección:
      1. ESPIRAL  → suciedad concentrada en los bordes del área limpiable
                    (border_ratio > REPLAN_BORDER_RATIO_ESPIRAL)
      2. MATRICIAL → zona sucia alargada (aspect_ratio > umbral)
      3. BOWTIE   → suciedad dispersa por toda el área

    Retorna el nombre del patrón elegido.
    """
    cells = grid_map.get_uncleaned_cells_array()
    if cells is None or len(cells) < 4:
        return PATRON_MATRICIAL   # default seguro

    xs = cells[:, 0]
    ys = cells[:, 1]

    x_min, x_max = float(xs.min()), float(xs.max())
    y_min, y_max = float(ys.min()), float(ys.max())
    width  = x_max - x_min
    height = y_max - y_min

    # ── Aspect ratio de la zona sucia ────────────────────────────
    if height < 1.0:
        aspect = 99.0
    else:
        aspect = width / height

    # ── Border ratio: ¿cuántas celdas sucias están cerca del borde
    #    de la zona limpiable? ─────────────────────────────────────
    cleanable_idxs = np.argwhere(grid_map.cleanable)
    if len(cleanable_idxs) > 0:
        cl_r_min = int(cleanable_idxs[:, 0].min())
        cl_r_max = int(cleanable_idxs[:, 0].max())
        cl_c_min = int(cleanable_idxs[:, 1].min())
        cl_c_max = int(cleanable_idxs[:, 1].max())

        # Profundidad de "borde" = 12% del tamaño del área limpiable
        r_span = cl_r_max - cl_r_min + 1
        c_span = cl_c_max - cl_c_min + 1
        border_depth = max(2, int(min(r_span, c_span) * 0.12))
        dirty = grid_map.cleanable & (1 - grid_map.cleaned)
        border_mask = np.zeros_like(dirty)
        border_mask[cl_r_min:cl_r_min+border_depth, :] = 1
        border_mask[cl_r_max-border_depth+1:cl_r_max+1, :] = 1
        border_mask[:, cl_c_min:cl_c_min+border_depth] = 1
        border_mask[:, cl_c_max-border_depth+1:cl_c_max+1] = 1

        n_border = int(np.sum(dirty & border_mask))
        n_total  = int(np.sum(dirty))
        border_ratio = n_border / n_total if n_total > 0 else 0.0
    else:
        border_ratio = 0.0

    # ── Decisión ─────────────────────────────────────────────────
    if border_ratio >= REPLAN_BORDER_RATIO_ESPIRAL:
        return PATRON_ESPIRAL
    elif aspect >= REPLAN_ASPECT_RATIO_MATRICIAL or aspect <= (1.0 / REPLAN_ASPECT_RATIO_MATRICIAL):
        return PATRON_MATRICIAL
    else:
        return PATRON_BOWTIE


def replan_cleaning(grid_map):
    """
    Genera una nueva ruta de limpieza enfocada en las áreas sin limpiar.

    1. Obtiene el bbox ajustado de celdas sucias (más pequeño que el corral)
    2. Selecciona el mejor patrón para esa distribución
    3. Genera los waypoints dentro de ese bbox

    Retorna (waypoints, patron_elegido) o ([], PATRON_NINGUNO) si todo limpio.
    """
    if not grid_map.has_cleanable_mask():
        return [], PATRON_NINGUNO

    uncleaned_count = grid_map.get_uncleaned_cell_count()
    if uncleaned_count == 0:
        return [], PATRON_NINGUNO

    # Elegir patrón según distribución de suciedad
    patron = select_best_pattern(grid_map)

    # Obtener bbox de zona sucia (puede ser más pequeño que el corral entero)
    dirty_bbox = grid_map.get_uncleaned_bbox()
    if dirty_bbox is None:
        return [], PATRON_NINGUNO

    # Generar waypoints dentro del bbox sucio
    if patron == PATRON_MATRICIAL:
        wps = generate_matricial(dirty_bbox)
    elif patron == PATRON_ESPIRAL:
        wps = generate_espiral(dirty_bbox)
    elif patron == PATRON_BOWTIE:
        wps = generate_bowtie(dirty_bbox)
    else:
        wps = generate_matricial(dirty_bbox)

    return wps, patron


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

    # ── Mapeo inicial de paredes ─────────────────────────
    scan_start_time = None
    scan_hit_counts = np.zeros((GRID_ROWS, GRID_COLS), dtype=np.int16)

    # ── Scan matching ────────────────────────────────────
    wall_centers_xy      = None   # array (N×2) precomputado tras F1
    match_tick_counter   = 0
    last_match_quality   = 0.0
    last_match_corr_mm   = 0.0

    # ── Fase D: clasificador + bloqueo frontal ───────────
    block_ticks_on      = 0
    block_ticks_off     = 0
    frontal_blocked     = False
    last_frontal_count  = 0
    dynamic_hits_render = []
    static_hits_render  = []

    # ── Fase E: grilla como filtro de seguridad ──────────
    skipped_wp_indices  = set()
    path_check_counter  = 0

    # ── Patrones de limpieza ─────────────────────────────
    patron_actual       = PATRON_NINGUNO
    patron_idx          = 0

    # ── Replanning de limpieza ───────────────────────────
    clean_threshold     = CLEAN_THRESHOLD_DEFAULT
    replan_count        = 0
    cleaning_complete   = False

    # ── UI estado ────────────────────────────────────────
    dropdown_open       = False   # Controls dropdown visible
    last_export_file    = None    # último archivo PNG exportado   # True cuando coverage >= threshold

    last_render_time = 0.0
    robot_moving     = False   # inicializar antes del primer render

    map_rect = get_map_rect()
    # No help_rect needed — toolbar is always drawn at y=0

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
                    if e.key == pygame.K_e:
                        # Exportar mapa PNG
                        coverage_now = grid_map.get_clean_coverage_pct()
                        last_export_file = export_cleaning_map(grid_map, coverage_now)
                        print(f"Mapa exportado: {last_export_file}")

                    elif e.key == pygame.K_F1:
                        # Iniciar (o re-iniciar) escaneo de paredes
                        if modo != MODE_SCAN:
                            grid_map.clear_walls()
                            scan_hit_counts.fill(0)
                            scan_start_time = now
                            wall_centers_xy = None
                            last_match_quality = 0.0
                            last_match_corr_mm = 0.0
                            modo = MODE_SCAN
                            pl, pr = 0.0, 0.0
                            arduino.send_command(0, 0, 0, 0)
                    elif e.key == pygame.K_F2:
                        # Ciclar patrón de limpieza y generar waypoints
                        if grid_map.has_wall_map() and modo != MODE_SCAN:
                            patron_idx    = (patron_idx + 1) % len(PATRONES_CICLO)
                            patron_actual = PATRONES_CICLO[patron_idx]
                            if patron_actual != PATRON_NINGUNO:
                                nuevos_wps = generate_pattern(patron_actual, grid_map)
                                if nuevos_wps:
                                    ruta_waypoints     = nuevos_wps
                                    waypoint_idx       = 0
                                    skipped_wp_indices = set()
                                    path_check_counter = 0
                                    fsm_state          = FSM_IDLE
                                    cleaning_complete  = False
                                    if modo == MODE_RUTA:
                                        modo = MODE_MANUAL
                                        pl, pr = 0.0, 0.0
                                        arduino.send_command(0, 0, 0, 0)
                            else:
                                ruta_waypoints     = []
                                waypoint_idx       = 0
                                skipped_wp_indices = set()

                    elif e.key == pygame.K_F3:
                        # Forzar replanning inmediato
                        if grid_map.has_cleanable_mask() and modo != MODE_SCAN:
                            wps, pat, ok = do_replan(grid_map)
                            if ok:
                                ruta_waypoints     = wps
                                patron_actual      = pat
                                waypoint_idx       = 0
                                skipped_wp_indices = set()
                                path_check_counter = 0
                                fsm_state          = FSM_IDLE
                                replan_count      += 1
                                cleaning_complete  = False
                                if modo == MODE_RUTA:
                                    modo = MODE_MANUAL
                                    pl, pr = 0.0, 0.0
                                    arduino.send_command(0, 0, 0, 0)

                    elif e.key == pygame.K_LEFTBRACKET:
                        # Bajar umbral de limpieza
                        clean_threshold = max(CLEAN_THRESHOLD_MIN,
                                              clean_threshold - CLEAN_THRESHOLD_STEP)
                    elif e.key == pygame.K_RIGHTBRACKET:
                        # Subir umbral de limpieza
                        clean_threshold = min(CLEAN_THRESHOLD_MAX,
                                              clean_threshold + CLEAN_THRESHOLD_STEP)

                    elif e.key == pygame.K_TAB:
                        modo = MODE_RUTA if modo == MODE_MANUAL else MODE_MANUAL
                        if modo == MODE_RUTA:
                            pl, pr = 0.0, 0.0
                            arduino.send_command(0, 0, 0, 0)
                            waypoint_idx       = 0
                            fsm_state          = FSM_IDLE
                            skipped_wp_indices = set()
                            path_check_counter = 0
                        else:
                            pl, pr = 0.0, 0.0
                            arduino.send_command(0, 0, 0, 0)
                            fsm_state = FSM_IDLE
                    elif e.key == pygame.K_BACKSPACE:
                        ruta_waypoints.clear()
                        waypoint_idx       = 0
                        skipped_wp_indices = set()
                        path_check_counter = 0
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
                        scan_start_time        = None
                        scan_hit_counts.fill(0)
                        # Nota: las paredes NO se borran con P
                        # Para re-escanear paredes usar F1

                # ── Click del mouse ───────────────────────────────────
                if e.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = e.pos
                    # Toggle Controls dropdown
                    if _CTRL_BTN_RECT and _CTRL_BTN_RECT.collidepoint(mx, my):
                        dropdown_open = not dropdown_open
                    else:
                        # Close dropdown on any click outside button
                        if dropdown_open and my > TOOLBAR_H:
                            dropdown_open = False
                        # Waypoint clicks (only on map area, not toolbar)
                        if my > TOOLBAR_H and grid_map.is_inside_map_rect(map_rect, mx, my):
                            if e.button == 1:   # click izquierdo → añadir
                                wx, wy = grid_map.screen_to_world(map_rect, mx, my)
                                wx = clamp(wx, 0.0, float(MAPA_ANCHO_MM))
                                wy = clamp(wy, 0.0, float(MAPA_ALTO_MM))
                                wx, wy = grid_map.nearest_free_cell_mm(wx, wy)
                                ruta_waypoints.append((wx, wy))
                            elif e.button == 3: # click derecho → borrar último
                                if ruta_waypoints:
                                    ruta_waypoints.pop()
                                    waypoint_idx = clamp(waypoint_idx, 0,
                                        max(0, len(ruta_waypoints) - 1))

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
            lidar_hits_global = transform_scan_to_global(
                ang, dist, lidar_x_mm, lidar_y_mm, yaw_deg
            )

            # ── Fase D: clasificar hits en estáticos / dinámicos ──
            dynamic_hits_render, static_hits_render, _ = classify_hits(
                lidar_hits_global, lidar_x_mm, lidar_y_mm, yaw_deg
            )

            # Conteo dinámico global (mantiene compatibilidad con dyn_alert)
            dyn_count = len(dynamic_hits_render)
            dyn_alert = dyn_count >= UMBRAL_PUNTOS_DINAMICOS

            # Obstáculo frontal persistente con clasificación geométrica
            n_frontal, _ = check_frontal_obstacle(
                dynamic_hits_render, lidar_x_mm, lidar_y_mm, yaw_deg
            )
            last_frontal_count = n_frontal

            if n_frontal >= BLOCK_MIN_IRREG_HITS:
                block_ticks_on  += 1
                block_ticks_off  = 0
            else:
                block_ticks_off += 1
                block_ticks_on   = 0

            if not frontal_blocked and block_ticks_on >= BLOCK_CONFIRM_TICKS:
                frontal_blocked = True
            if frontal_blocked and block_ticks_off >= BLOCK_CLEAR_TICKS:
                frontal_blocked = False

            # ── Decisión de movimiento según modo ─────────────────
            if modo == MODE_SCAN:
                # Robot quieto durante el scan
                pl, pr = 0.0, 0.0

                # Acumular hits LiDAR en scan_hit_counts
                for xp, yp in lidar_hits_global:
                    idx = grid_map.world_to_cell(xp, yp)
                    if idx is not None:
                        r, c = idx
                        scan_hit_counts[r, c] += 1

                # Comprobar si terminó el tiempo de scan
                if scan_start_time is not None and (now - scan_start_time) >= SCAN_DURATION_S:
                    wall_cells = np.argwhere(scan_hit_counts >= SCAN_MIN_HITS)
                    for r, c in wall_cells:
                        grid_map.walls[r, c] = 1
                    wall_centers_xy = grid_map.get_wall_centers_xy()
                    # Computar máscara de área limpiable (post-scan)
                    _bbox = get_cleaning_bbox(grid_map)
                    grid_map.compute_cleanable_mask(_bbox)
                    modo = MODE_MANUAL
                    scan_start_time = None

            elif modo == MODE_MANUAL:
                keys = pygame.key.get_pressed()
                pl, pr = get_manual_drive(keys, pwm_base, pwm_turn)
            else:
                # Modo RUTA: pasar frontal_blocked al follower
                pl, pr, fsm_state, waypoint_idx = follow_route_step(
                    lidar_x_mm, lidar_y_mm, yaw_deg,
                    ruta_waypoints, waypoint_idx,
                    fsm_state, frontal_blocked
                )

                if fsm_state == FSM_ROUTE_DONE:
                    coverage = grid_map.get_clean_coverage_pct()
                    if coverage >= clean_threshold:
                        # Limpieza completa — volver a MANUAL
                        cleaning_complete = True
                        modo      = MODE_MANUAL
                        fsm_state = FSM_IDLE
                        pl, pr    = 0.0, 0.0
                        arduino.send_command(0, 0, 0, 0)
                    else:
                        # Cobertura insuficiente → replan automático
                        wps, pat, ok = do_replan(grid_map)
                        if ok:
                            ruta_waypoints     = wps
                            patron_actual      = pat
                            waypoint_idx       = 0
                            skipped_wp_indices = set()
                            path_check_counter = 0
                            fsm_state          = FSM_IDLE
                            replan_count      += 1
                            # Continuar en modo RUTA directamente
                        else:
                            # No hay más zonas sucias accesibles
                            cleaning_complete = True
                            modo      = MODE_MANUAL
                            fsm_state = FSM_IDLE
                            pl, pr    = 0.0, 0.0
                            arduino.send_command(0, 0, 0, 0)

                # ── Fase E: verificar camino cada N ticks ──────
                if grid_map.has_wall_map():
                    path_check_counter += 1
                    if path_check_counter >= PATH_CHECK_EVERY_N:
                        path_check_counter = 0

                        # Revisar todos los waypoints pendientes
                        i = waypoint_idx
                        while i < len(ruta_waypoints):
                            if i in skipped_wp_indices:
                                i += 1
                                continue
                            wx, wy = ruta_waypoints[i]
                            if grid_map.path_crosses_wall(
                                    lidar_x_mm, lidar_y_mm, wx, wy):
                                skipped_wp_indices.add(i)
                                # Si es el waypoint activo, avanzar al siguiente
                                if i == waypoint_idx:
                                    waypoint_idx += 1
                                    # Buscar el siguiente no saltado
                                    while (waypoint_idx < len(ruta_waypoints)
                                           and waypoint_idx in skipped_wp_indices):
                                        waypoint_idx += 1
                                    if waypoint_idx >= len(ruta_waypoints):
                                        # Todos saltados → fin de ruta
                                        modo      = MODE_MANUAL
                                        fsm_state = FSM_IDLE
                                        pl, pr    = 0.0, 0.0
                                        arduino.send_command(0, 0, 0, 0)
                                    else:
                                        fsm_state = FSM_ALIGN
                            i += 1

            for xp, yp in lidar_hits_global:
                grid_map.mark_discovered(xp, yp)

            pl_cmd = pl * SIGNO_MOTOR_IZQ
            pr_cmd = pr * SIGNO_MOTOR_DER

            aux_pwm = int(255 * NIVELES_LIMPIEZA[pwr_idx]) if clean_sys else 0
            arduino.send_command(pl_cmd, pr_cmd, aux_pwm, comp_on)

            # ── Estimación de pose ─────────────────────────────────
            robot_moving = (abs(pl_cmd) >= PWM_VEL_THRESHOLD or
                            abs(pr_cmd) >= PWM_VEL_THRESHOLD)

            # Leer deltas de encoders (disponibles si Arduino envía ENC:)
            enc_delta_izq, enc_delta_der, enc_available = arduino.get_encoder_deltas()

            if wall_centers_xy is not None:
                # ── NIVEL 1: Scan matching (máxima precisión) ──────
                if enc_available:
                    lidar_x_mm, lidar_y_mm = odometry_encoder_step(
                        lidar_x_mm, lidar_y_mm, yaw_deg,
                        enc_delta_izq, enc_delta_der
                    )
                match_tick_counter += 1
                if match_tick_counter >= MATCH_EVERY_N_TICKS:
                    match_tick_counter = 0
                    (lidar_x_mm, lidar_y_mm,
                     last_match_quality,
                     last_match_corr_mm) = scan_matching_step(
                        lidar_x_mm, lidar_y_mm,
                        lidar_hits_global,
                        wall_centers_xy
                    )
                    last_pose_correction = last_match_corr_mm

            elif enc_available:
                # ── NIVEL 2: Encoders solos (sin mapa aún) ─────────
                lidar_x_mm, lidar_y_mm = odometry_encoder_step(
                    lidar_x_mm, lidar_y_mm, yaw_deg,
                    enc_delta_izq, enc_delta_der
                )
                last_match_quality   = 0.0
                last_match_corr_mm   = 0.0
                last_pose_correction = 0.0

            else:
                # ── Sin encoders ni mapa: pose congelada ───────────
                # No se usa PWM del teclado para mover la pose.
                # El robot se mueve físicamente pero la posición en
                # pantalla no cambia hasta que lleguen datos reales.
                last_match_quality   = 0.0
                last_match_corr_mm   = 0.0
                last_pose_correction = 0.0

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
                grid_map.draw_classified_hits(screen, map_rect,
                    dynamic_hits_render, static_hits_render)
                grid_map.draw_frontal_cone(screen, map_rect,
                    lidar_x_mm, lidar_y_mm, yaw_deg, blocked=frontal_blocked)
                grid_map.draw_route(screen, map_rect, ruta_waypoints,
                                    waypoint_idx, fsm_state,
                                    skipped_indices=skipped_wp_indices)
                grid_map.draw_robot(screen, map_rect, lidar_x_mm, lidar_y_mm, yaw_deg,
                                    alert=frontal_blocked)

                rx_age = now - arduino.last_rx_time if arduino.last_rx_time > 0 else None
                rx_age_txt = f"{rx_age:.2f}s" if rx_age is not None else "--"

                lidar_age = now - lidar.last_rx_time if lidar.last_rx_time > 0 else None
                lidar_rx_txt = f"{lidar_age:.2f}s" if lidar_age is not None else "--"

                state = {
                    "modo": modo,
                    "fsm_state": fsm_state,
                    "n_waypoints": len(ruta_waypoints),
                    "waypoint_idx": waypoint_idx,
                    "scan_progress": clamp((now - scan_start_time) / SCAN_DURATION_S, 0.0, 1.0)
                                     if scan_start_time is not None else 0.0,
                    "scan_wall_count": grid_map.get_wall_count(),
                    "has_wall_map": grid_map.has_wall_map(),
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
                    "match_quality": last_match_quality,
                    "enc_available": enc_available,
                    "enc_acum_izq":  enc_delta_izq,
                    "enc_acum_der":  enc_delta_der,
                    "frontal_blocked": frontal_blocked,
                    "frontal_count": last_frontal_count,
                    "skipped_count": len(skipped_wp_indices),
                    "patron_actual":    patron_actual,
                    "clean_coverage":   grid_map.get_clean_coverage_pct(),
                    "clean_threshold":  clean_threshold,
                    "replan_count":     replan_count,
                    "cleaning_complete": cleaning_complete,
                    "cleanable_count":  grid_map.get_cleanable_count(),
                    "uncleaned_count":  grid_map.get_uncleaned_cell_count(),
                    "dyn_count_total": dyn_count,
                    "clean_on": clean_sys,
                    "comp_on": comp_on,
                    "power": NIVELES_LIMPIEZA[pwr_idx],
                    "aux_pwm": aux_pwm,
                    "disc_pct": grid_map.get_discovered_pct(),
                    "clean_pct": grid_map.get_cleaned_pct(),
                    "last_dang": last_dang,
                    "arduino_ok": arduino_ok,
                    "lidar_ok": lidar_ok,
                    "rx_age_txt": rx_age_txt,
                    "lidar_rx_txt": lidar_rx_txt,
                }

                draw_panel(screen, font, small, state)
                draw_toolbar(screen, font, small, state, dropdown_open=dropdown_open)
                if dropdown_open:
                    draw_controls_dropdown(screen, small)
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
