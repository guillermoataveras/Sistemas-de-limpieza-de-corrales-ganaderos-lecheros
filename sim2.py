"""
sim.py — Simulador del Robot Cleaner de Corrales Ganaderos
===========================================================
Ejecutar con:
    python sim.py
    python sim.py ruta/a/LiDar3_20_route_fix.py

No requiere hardware. Arduino, LiDAR y cámaras son simulados.
Todos los controles y modos del programa real funcionan igual.

Teclas exclusivas de simulación:
    I  → Inyectar obstáculo estático delante del robot
    K  → Eliminar todos los obstáculos inyectados

Al iniciar imprime un banner con los parámetros activos.
"""

import sys, os, math, time, threading, importlib.util, types
import numpy as np
import pygame

# ══════════════════════════════════════════════════════════════════
# LOCALIZAR EL MÓDULO PRINCIPAL
# ══════════════════════════════════════════════════════════════════
_CANDIDATES = [
    sys.argv[1] if len(sys.argv) > 1 else None,
    "LiDar3_20_route_fix.py",
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "LiDar3_20_route_fix.py"),
]
MAIN_FILE = next((c for c in _CANDIDATES if c and os.path.exists(c)), None)
if MAIN_FILE is None:
    print("ERROR: No se encontró LiDar3_20_route_fix.py")
    print("Uso: python sim.py [ruta/al/archivo.py]")
    sys.exit(1)

# ══════════════════════════════════════════════════════════════════
# ★ PARÁMETROS AJUSTABLES — edita aquí para calibrar el simulador ★
# ══════════════════════════════════════════════════════════════════

# ── Posición y orientación inicial ───────────────────────────────
SIM_INITIAL_X_MM    = None      # None = centro del mapa (MAPA_ANCHO_MM / 2)
SIM_INITIAL_Y_MM    = None      # None = centro del mapa
SIM_INITIAL_YAW_DEG = 0.0       # orientación inicial en grados (0 = este)

# ── Corral simulado (mm) ─────────────────────────────────────────
SIM_WALL_X1 = 600.0             # pared oeste (izquierda)
SIM_WALL_Y1 = 600.0             # pared sur   (abajo en mundo)
SIM_WALL_X2 = 4400.0            # pared este  (derecha)
SIM_WALL_Y2 = 4400.0            # pared norte (arriba en mundo)

# ── Velocidad y física ────────────────────────────────────────────
SIM_VEL_MM_PER_PWM  = 3.5       # mm/s por unidad de PWM (calibrar con robot real)
SIM_PWM_THRESHOLD   = 30        # PWM mínimo para que el robot se mueva
SIM_PHYSICS_HZ      = 50        # frecuencia de actualización de física (Hz)

# ── Dirección de los motores ─────────────────────────────────────
# Cambia a True si W mueve el robot hacia atrás en el mapa.
# Esto depende del montaje físico de los motores.
SIM_FLIP_LEFT_MOTOR  = True     # True = motor izq requiere inversión (SIGNO_IZQ=-1)
SIM_FLIP_RIGHT_MOTOR = False    # True = motor der requiere inversión (SIGNO_DER=-1)

# ── Ruido gaussiano ───────────────────────────────────────────────
SIM_LIDAR_NOISE_MM  = 4.0       # ruido en mediciones LiDAR (mm)
SIM_ENC_NOISE_MM    = 0.8       # ruido en encoders (mm/paso de física)
SIM_GYRO_NOISE_DEG  = 0.06      # ruido del giroscopio (°/paso de física)

# ── LiDAR ────────────────────────────────────────────────────────
SIM_LIDAR_HZ        = 15        # frecuencia de barrido (Hz)
SIM_LIDAR_MIN_MM    = 30.0      # distancia mínima válida
SIM_LIDAR_MAX_MM    = 12000.0   # distancia máxima válida

# ── Cámara ───────────────────────────────────────────────────────
# Los frames simulados contienen pixeles "claros" (limpio) y "oscuros" (sucio).
# La detección HSV del main los clasifica correctamente.
SIM_DIRT_BASE       = 0.30      # suciedad media del suelo (0=limpio, 1=sucio)
SIM_DIRT_NOISE      = 0.12      # variabilidad frame a frame (gaussiana)
SIM_DIRT_REAR_BASE  = 0.25      # suciedad base de la cámara trasera
# Para simular zona limpiada de verdad, la cámara trasera puede devolver
# menor suciedad cuando el robot acabó de pasar por ahí.
SIM_DIRT_AFTER_CLEAN= 0.08      # suciedad residual en zona recién limpiada

# ── Obstáculo inyectable (tecla I) ───────────────────────────────
SIM_OBS_RADIUS_MM   = 250.0     # radio del obstáculo circular (mm)
SIM_OBS_FRONT_MM    = 500.0     # distancia frontal al inyectar


# ══════════════════════════════════════════════════════════════════
# ESTADO FÍSICO COMPARTIDO (fuente de verdad del simulador)
# ══════════════════════════════════════════════════════════════════
class _SimState:
    """Posición/orientación real del robot — compartido entre todos los drivers."""
    def __init__(self, x0: float, y0: float, yaw0: float = 0.0):
        self.x    = x0
        self.y    = y0
        self.yaw  = yaw0   # grados
        self.pl   = 0.0    # último pl_cmd recibido por send_command
        self.pr   = 0.0    # último pr_cmd recibido
        self.lock = threading.Lock()


# ══════════════════════════════════════════════════════════════════
# ARDUINO SIMULADO
# ══════════════════════════════════════════════════════════════════
class SimulatedArduino:
    """
    Replica la interfaz de ArduinoSerialController.
    Integra cinemática diferencial a SIM_PHYSICS_HZ Hz.

    Recibe pl_cmd y pr_cmd YA con SIGNO aplicado (igual que en main).
    SIM_FLIP_LEFT_MOTOR = True significa que pl negativo = izq adelante.
    """
    def __init__(self, port, baudrate=115200):
        self._state            = None      # inyectado por bootstrap
        self._vel_mm_per_pwm   = SIM_VEL_MM_PER_PWM
        self._pwm_thr          = SIM_PWM_THRESHOLD
        self._wheel_track      = 395.0     # sobrescrito por bootstrap
        self._mm_per_deg       = math.pi * 76.2 / 360.0  # sobrescrito
        self._flip_l           = SIM_FLIP_LEFT_MOTOR
        self._flip_r           = SIM_FLIP_RIGHT_MOTOR

        self.accumulated_yaw_deg = 0.0
        self.last_dang           = 0.0
        self.last_rx_time        = 0.0
        self.enc_acum_izq        = 0.0
        self.enc_acum_der        = 0.0
        self.enc_delta_izq       = 0.0
        self.enc_delta_der       = 0.0
        self.enc_available       = True
        self.lock                = threading.Lock()
        self._running            = False
        self._thread             = None
        self._dt                 = 1.0 / SIM_PHYSICS_HZ

    def connect(self):
        self._running = True
        self.last_rx_time = time.time()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return True

    def _wheel_vel(self, pwm_cmd, flip):
        """
        Convierte PWM de comando en velocidad de rueda (mm/s).
        flip=True: polaridad invertida (motor montado al revés → negativo PWM = avanzar).
        """
        if abs(pwm_cmd) < self._pwm_thr:
            return 0.0
        vel = float(pwm_cmd) * self._vel_mm_per_pwm
        return -vel if flip else vel

    def _loop(self):
        rng = np.random.default_rng()
        while self._running:
            t0 = time.time()

            with self._state.lock:
                pl = self._state.pl
                pr = self._state.pr

            # Velocidades de hub de cada rueda
            v_l = self._wheel_vel(pl, self._flip_l)
            v_r = self._wheel_vel(pr, self._flip_r)

            # Cinemática diferencial
            v_lin    = (v_l + v_r) * 0.5
            yaw_rate = (v_r - v_l) / self._wheel_track   # rad/s
            yaw_d    = math.degrees(yaw_rate * self._dt)
            yaw_d   += float(rng.normal(0.0, SIM_GYRO_NOISE_DEG))

            with self._state.lock:
                yr = math.radians(self._state.yaw)
                self._state.x   += v_lin * math.cos(yr) * self._dt
                self._state.y   += v_lin * math.sin(yr) * self._dt
                self._state.yaw += yaw_d

            # Encoders: distancia lineal → grados de eje
            noise_l = float(rng.normal(0.0, SIM_ENC_NOISE_MM))
            noise_r = float(rng.normal(0.0, SIM_ENC_NOISE_MM))
            dist_l  = v_l * self._dt + noise_l
            dist_r  = v_r * self._dt + noise_r
            # Convertir a grados de rueda (positivo = avanzar)
            deg_l   = dist_l / self._mm_per_deg if self._mm_per_deg > 0 else 0.0
            deg_r   = dist_r / self._mm_per_deg if self._mm_per_deg > 0 else 0.0

            with self.lock:
                self.accumulated_yaw_deg += yaw_d
                self.last_dang            = yaw_d
                self.enc_acum_izq        += deg_l
                self.enc_acum_der        += deg_r
                self.enc_delta_izq        = deg_l
                self.enc_delta_der        = deg_r
                self.last_rx_time         = time.time()

            elapsed = time.time() - t0
            sleep_t = self._dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    def send_command(self, izq, der, aux_pwm, comp_state):
        """Recibe pl_cmd y pr_cmd con SIGNO ya aplicado."""
        with self._state.lock:
            self._state.pl = float(izq)
            self._state.pr = float(der)

    # ── Interfaz compatible con ArduinoSerialController ──────────
    def is_connected(self):          return True
    def get_yaw(self):
        with self.lock: return self.accumulated_yaw_deg
    def get_last_dang(self):
        with self.lock: return self.last_dang
    def get_encoder_deltas(self):
        with self.lock:
            return self.enc_delta_izq, self.enc_delta_der, self.enc_available
    def get_encoder_accum(self):
        with self.lock:
            return self.enc_acum_izq, self.enc_acum_der
    def reset_yaw_accumulator(self):
        with self.lock:
            self.accumulated_yaw_deg = 0.0
            self.last_dang = 0.0
    def close(self):
        self._running = False


# ══════════════════════════════════════════════════════════════════
# LIDAR SIMULADO — RAYCASTING 2D
# ══════════════════════════════════════════════════════════════════
def _ray_seg_dist(ox, oy, dx, dy, x1, y1, x2, y2):
    """Distancia rayo→segmento. Retorna inf si no hay intersección."""
    ax, ay = x1 - ox, y1 - oy
    bx, by = x2 - x1, y2 - y1
    den    = dx * by - dy * bx
    if abs(den) < 1e-10:
        return float('inf')
    t = (ax * by - ay * bx) / den
    s = (ax * dy - ay * dx) / den
    return t if (t > 0.02 and 0.0 <= s <= 1.0) else float('inf')


class SimulatedLiDAR:
    """
    Replica la interfaz de LiDAR_LD20.
    Genera 360 puntos por barrido mediante raycasting contra el corral y obstáculos.

    ★ OFFSET_LIDAR_FISICO ★
    El módulo principal aplica un offset angular al convertir los ángulos
    del sensor a coordenadas globales (transform_scan_to_global).
    Para que el mapa de paredes quede correctamente orientado, los ángulos
    reportados deben estar en el frame del sensor (incluyendo ese offset).
    El bootstrap inyecta _lidar_angle_offset desde mod.OFFSET_LIDAR_FISICO.
    """
    def __init__(self, port, baudrate=230400):
        self._state  = None    # inyectado por bootstrap
        self.running = False
        self.last_rx_time     = 0.0
        self._lock            = threading.Lock()
        self._pts             = []
        self._thread          = None
        self._lidar_angle_offset = 0.0  # sobrescrito por bootstrap

        x1, y1, x2, y2 = SIM_WALL_X1, SIM_WALL_Y1, SIM_WALL_X2, SIM_WALL_Y2
        self._walls = [
            (x1, y1, x2, y1),   # sur
            (x2, y1, x2, y2),   # este
            (x2, y2, x1, y2),   # norte
            (x1, y2, x1, y1),   # oeste
        ]
        self.obstacles = []    # lista de (cx, cy, radius_mm)

    def connect(self):
        self.running = True
        self.last_rx_time = time.time()
        self._thread = threading.Thread(target=self._scan_loop, daemon=True)
        self._thread.start()
        return True

    def _cast(self, ox, oy, angle_deg):
        """Distancia al obstáculo más cercano en la dirección angle_deg (grados globales)."""
        rad = math.radians(angle_deg)
        dx, dy = math.cos(rad), math.sin(rad)
        min_d  = float('inf')

        for seg in self._walls:
            d = _ray_seg_dist(ox, oy, dx, dy, *seg)
            if d < min_d:
                min_d = d

        for cx, cy, r in self.obstacles:
            ex, ey = cx - ox, cy - oy
            proj   = ex * dx + ey * dy
            if proj <= 0:
                continue
            perp2 = (ex - proj*dx)**2 + (ey - proj*dy)**2
            if perp2 < r * r:
                d_hit = proj - math.sqrt(max(0.0, r*r - perp2))
                if 0 < d_hit < min_d:
                    min_d = d_hit

        return min_d

    def _scan_loop(self):
        rng      = np.random.default_rng(42)
        interval = 1.0 / SIM_LIDAR_HZ
        offset   = self._lidar_angle_offset   # e.g. 186.0

        while self.running:
            with self._state.lock:
                rx, ry, ryaw = self._state.x, self._state.y, self._state.yaw

            pts = []
            for a_sensor in range(0, 360, 1):
                # Ángulo en frame del sensor (a_sensor=0 apunta al frente del sensor)
                # Dirección global: ryaw + a_sensor - offset
                # (el main sumará offset y restará offset, quedando ryaw + a_sensor)
                world_angle = ryaw + a_sensor - offset
                d = self._cast(rx, ry, world_angle)
                d += float(rng.normal(0.0, SIM_LIDAR_NOISE_MM))
                if SIM_LIDAR_MIN_MM < d < SIM_LIDAR_MAX_MM:
                    pts.append((float(a_sensor), float(d)))

            with self._lock:
                self._pts = pts
            self.last_rx_time = time.time()
            time.sleep(interval)

    def obtener_datos_nuevos(self):
        with self._lock:
            if not self._pts:
                return np.empty(0, np.float32), np.empty(0, np.float32)
            datos     = self._pts[:]
            self._pts = []
        angs, dists = zip(*datos)
        return np.array(angs, np.float32), np.array(dists, np.float32)

    def close(self):
        self.running = False


# ══════════════════════════════════════════════════════════════════
# CÁMARA SIMULADA (frontal y trasera)
# ══════════════════════════════════════════════════════════════════
class SimulatedCamera:
    """
    Replica la interfaz de CameraCapture.
    Genera frames numpy BGR con píxeles "limpios" y "sucios".
    La detección HSV del módulo principal los clasifica correctamente:
      Limpio → S bajo, V alto  → BGR = (215, 215, 215)
      Sucio  → S alto, V bajo  → BGR = (55, 85, 115)
    """
    def __init__(self, index=0):
        self._ok    = False
        self._index = index
        self._rng   = np.random.default_rng(index * 137 + 99)
        self._is_rear = (index != 0)
        self._base_dirt = SIM_DIRT_REAR_BASE if self._is_rear else SIM_DIRT_BASE

    def connect(self):
        self._ok = True
        label = "trasera" if self._is_rear else "frontal"
        print(f"[SIM] Cámara {label} (índice {self._index}) conectada.")
        return True

    def get_frame(self):
        H, W   = 480, 640
        dirt   = float(np.clip(
            self._rng.normal(self._base_dirt, SIM_DIRT_NOISE), 0.0, 1.0))
        frame  = np.full((H, W, 3), [215, 215, 215], dtype=np.uint8)
        mask   = self._rng.random((H, W)) < dirt
        frame[mask] = [55, 85, 115]   # color de suciedad (S alto, V bajo en HSV)
        return frame

    def is_ok(self):  return self._ok
    def close(self):  self._ok = False


# ══════════════════════════════════════════════════════════════════
# TECLAS EXCLUSIVAS DE SIMULACIÓN (I / K)
# ══════════════════════════════════════════════════════════════════
_sim_lidar_ref = None
_sim_state_ref = None

def handle_sim_keys(event):
    """Intercepta teclas I/K para inyectar/limpiar obstáculos."""
    if event.type != pygame.KEYDOWN:
        return
    if _sim_lidar_ref is None or _sim_state_ref is None:
        return

    if event.key == pygame.K_i:
        with _sim_state_ref.lock:
            rx, ry, ryaw = _sim_state_ref.x, _sim_state_ref.y, _sim_state_ref.yaw
        rad   = math.radians(ryaw)
        obs_x = rx + math.cos(rad) * SIM_OBS_FRONT_MM
        obs_y = ry + math.sin(rad) * SIM_OBS_FRONT_MM
        _sim_lidar_ref.obstacles.append((obs_x, obs_y, SIM_OBS_RADIUS_MM))
        print(f"[SIM] Obstáculo inyectado en ({obs_x:.0f},{obs_y:.0f})  "
              f"total={len(_sim_lidar_ref.obstacles)}")

    elif event.key == pygame.K_k:
        _sim_lidar_ref.obstacles.clear()
        print("[SIM] Todos los obstáculos eliminados.")


# ══════════════════════════════════════════════════════════════════
# BOOTSTRAP PRINCIPAL
# ══════════════════════════════════════════════════════════════════
def main():
    global _sim_lidar_ref, _sim_state_ref

    print(f"\n[SIM] Cargando módulo principal: {MAIN_FILE}")
    spec = importlib.util.spec_from_file_location("robot_main", MAIN_FILE)
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    # ── Posición inicial ──────────────────────────────────────────
    x0 = SIM_INITIAL_X_MM if SIM_INITIAL_X_MM is not None else mod.LIDAR_X_INICIAL_MM
    y0 = SIM_INITIAL_Y_MM if SIM_INITIAL_Y_MM is not None else mod.LIDAR_Y_INICIAL_MM

    sim_state      = _SimState(x0, y0, SIM_INITIAL_YAW_DEG)
    _sim_state_ref = sim_state

    # ── Arduino simulado ──────────────────────────────────────────
    _arduino = SimulatedArduino.__new__(SimulatedArduino)
    SimulatedArduino.__init__(_arduino, "SIM_ARDUINO")
    _arduino._state          = sim_state
    _arduino._vel_mm_per_pwm = getattr(mod, "VEL_MM_PER_PWM", SIM_VEL_MM_PER_PWM)
    _arduino._pwm_thr        = getattr(mod, "PWM_VEL_THRESHOLD", SIM_PWM_THRESHOLD)
    _arduino._wheel_track    = getattr(mod, "WHEEL_TRACK_MM", 395.0)
    _arduino._mm_per_deg     = getattr(mod, "MM_PER_DEG_WHEEL", math.pi*76.2/360.0)

    # ── LiDAR simulado ────────────────────────────────────────────
    _lidar = SimulatedLiDAR.__new__(SimulatedLiDAR)
    SimulatedLiDAR.__init__(_lidar, "SIM_LIDAR")
    _lidar._state               = sim_state
    # ★ FIX: aplicar el mismo offset que usa transform_scan_to_global
    _lidar._lidar_angle_offset  = getattr(mod, "OFFSET_LIDAR_FISICO", 0.0)
    _sim_lidar_ref              = _lidar

    # ── Fábricas de clases (devuelven los drivers simulados) ──────
    class _ArdFactory:
        def __new__(cls, *a, **kw): return _arduino
    class _LidFactory:
        def __new__(cls, *a, **kw): return _lidar
    class _CamFactory:
        def __new__(cls, index=0, **kw): return SimulatedCamera(index)

    mod.LiDAR_LD20              = _LidFactory
    mod.ArduinoSerialController = _ArdFactory
    mod.CameraCapture           = _CamFactory

    # ── StateManager redirigido a archivo de sesión de sim ────────
    class _SimStateManager(mod.StateManager):
        def __init__(self):
            super().__init__(filepath="sim_session.json")
    mod.StateManager = _SimStateManager

    # ── Interceptar eventos de pygame para teclas I/K ─────────────
    _orig_get = pygame.event.get
    def _patched_get():
        evs = _orig_get()
        for e in evs:
            handle_sim_keys(e)
        return evs
    pygame.event.get = _patched_get

    # ── Banner ────────────────────────────────────────────────────
    w_mm = SIM_WALL_X2 - SIM_WALL_X1
    h_mm = SIM_WALL_Y2 - SIM_WALL_Y1
    flip_l = "invertido" if SIM_FLIP_LEFT_MOTOR  else "normal"
    flip_r = "invertido" if SIM_FLIP_RIGHT_MOTOR else "normal"
    offset = getattr(mod, "OFFSET_LIDAR_FISICO", 0.0)

    print("═" * 66)
    print("  MODO SIMULACIÓN — Robot Cleaner de Corrales Ganaderos")
    print(f"  Módulo:  {os.path.basename(MAIN_FILE)}")
    print(f"  Corral:  {w_mm/1000:.1f} × {h_mm/1000:.1f} m  "
          f"({SIM_WALL_X1:.0f},{SIM_WALL_Y1:.0f})→({SIM_WALL_X2:.0f},{SIM_WALL_Y2:.0f}) mm")
    print(f"  Pose:    x={x0:.0f} y={y0:.0f} yaw={SIM_INITIAL_YAW_DEG:.0f}°")
    print(f"  Vel:     {SIM_VEL_MM_PER_PWM} mm/s·PWM  |  Física {SIM_PHYSICS_HZ} Hz  "
          f"|  LiDAR {SIM_LIDAR_HZ} Hz")
    print(f"  Motores: izq={flip_l}  der={flip_r}")
    print(f"  LiDAR offset aplicado: {offset:.1f}°")
    print(f"  Suciedad cámara: {SIM_DIRT_BASE*100:.0f}% ± {SIM_DIRT_NOISE*100:.0f}%")
    print(f"  Ruido:   LiDAR {SIM_LIDAR_NOISE_MM}mm  "
          f"| Enc {SIM_ENC_NOISE_MM}mm  | Gyro {SIM_GYRO_NOISE_DEG:.2f}°")
    print("  I → inyectar obstáculo  |  K → eliminar obstáculos")
    print("  Si W va al revés: cambia SIM_FLIP_LEFT_MOTOR / SIM_FLIP_RIGHT_MOTOR")
    print("═" * 66)

    mod.main()


if __name__ == "__main__":
    main()
