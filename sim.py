"""
sim.py — Simulador del Robot Cleaner de Corrales Ganaderos
===========================================================
Ejecutar con:
    python sim.py
    python sim.py ruta/a/LiDar3_19_final_improvements.py

No requiere hardware: Arduino, LiDAR ni cámaras son simulados.
Todos los controles y modos del sistema real funcionan igual.

Teclas exclusivas de simulación:
    I  → Inyectar obstáculo estático delante del robot
    K  → Eliminar todos los obstáculos inyectados

Ambiente simulado por defecto:
    Corral rectangular 4 × 4 m dentro del mapa 5 × 5 m
    Suciedad base del suelo: 30 % (ajustar SIM_DIRT_BASE)
    Física diferencial a 50 Hz con ruido gaussiano
"""

import sys, os, math, time, threading, importlib.util, types
import numpy as np
import pygame

# ══════════════════════════════════════════════════════════════════
# LOCALIZAR EL MÓDULO PRINCIPAL
# ══════════════════════════════════════════════════════════════════
_CANDIDATES = [
    sys.argv[1] if len(sys.argv) > 1 else None,
    "LiDar3_19_final_improvements.py",
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "LiDar3_19_final_improvements.py"),
]
MAIN_FILE = next((c for c in _CANDIDATES if c and os.path.exists(c)), None)
if MAIN_FILE is None:
    print("ERROR: No se encontró LiDar3_19_final_improvements.py")
    print("Uso: python sim.py [ruta/al/archivo.py]")
    sys.exit(1)

# ══════════════════════════════════════════════════════════════════
# PARÁMETROS DE SIMULACIÓN (ajustables)
# ══════════════════════════════════════════════════════════════════
SIM_DT_S          = 0.020   # paso de física (50 Hz)
SIM_SCAN_HZ       = 15      # frecuencia del LiDAR simulado
SIM_NOISE_MM      = 4.0     # ruido gaussiano en lecturas LiDAR (mm)
SIM_ENC_NOISE_MM  = 0.8     # ruido en encoders (mm por paso de física)
SIM_GYRO_NOISE_DEG= 0.06    # ruido del giroscopio (grados por paso)

# Corral simulado (en mm)
SIM_WALL_X1 = 600.0
SIM_WALL_Y1 = 600.0
SIM_WALL_X2 = 4400.0
SIM_WALL_Y2 = 4400.0

# Suciedad sintética de la cámara
SIM_DIRT_BASE  = 0.30   # suciedad media del suelo (0=limpio, 1=sucio)
SIM_DIRT_NOISE = 0.12   # variabilidad gaussiana frame a frame

# Obstáculo inyectable
SIM_OBS_RADIUS_MM = 250.0   # radio del obstáculo (mm)
SIM_OBS_FRONT_MM  = 500.0   # distancia frontal al inyectar (mm)


# ══════════════════════════════════════════════════════════════════
# ESTADO FÍSICO COMPARTIDO
# ══════════════════════════════════════════════════════════════════
class _SimState:
    """Verdad del suelo del robot — compartida entre todos los drivers."""
    def __init__(self, x0: float, y0: float):
        self.x   = x0
        self.y   = y0
        self.yaw = 0.0   # grados
        self.pl  = 0.0   # PWM izq recibido en send_command (post-SIGNO)
        self.pr  = 0.0   # PWM der recibido
        self.lock= threading.Lock()


# ══════════════════════════════════════════════════════════════════
# ARDUINO SIMULADO
# ══════════════════════════════════════════════════════════════════
class SimulatedArduino:
    """
    Replica la interfaz de ArduinoSerialController.
    Integra cinemática diferencial y genera datos de encoders y giroscopio.
    """
    def __init__(self, port, baudrate=115200):
        self._state = None   # inyectado por la fábrica
        self.accumulated_yaw_deg = 0.0
        self.last_dang     = 0.0
        self.last_rx_time  = 0.0
        self.enc_acum_izq  = 0.0
        self.enc_acum_der  = 0.0
        self.enc_delta_izq = 0.0
        self.enc_delta_der = 0.0
        self.enc_available = True
        self.lock          = threading.Lock()
        self._running      = False
        self._thread       = None
        # Constantes físicas (sobreescritas por la fábrica)
        self._vel_mm_per_pwm= 3.5
        self._pwm_thr       = 30
        self._wheel_track   = 395.0
        self._mm_per_deg    = math.pi * 76.2 / 360.0

    def connect(self):
        self._running = True
        self._thread  = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return True

    def _loop(self):
        rng = np.random.default_rng()
        while self._running:
            # Leer comandos PWM actuales
            with self._state.lock:
                pl = self._state.pl
                pr = self._state.pr

            # Velocidades lineales (deshacer la inversión SIGNO_MOTOR_IZQ=-1)
            def v(pwm, flip=False):
                if abs(pwm) < self._pwm_thr:
                    return 0.0
                vel = float(pwm) * self._vel_mm_per_pwm
                return -vel if flip else vel

            v_l = v(pl, flip=True)   # izq: flip por SIGNO=-1
            v_r = v(pr, flip=False)

            # Cinemática diferencial
            v_lin     = (v_l + v_r) * 0.5
            yaw_rate  = (v_r - v_l) / self._wheel_track   # rad/s
            yaw_delta = math.degrees(yaw_rate * SIM_DT_S)
            yaw_delta+= float(rng.normal(0.0, SIM_GYRO_NOISE_DEG))

            # Actualizar posición física
            with self._state.lock:
                yaw_rad    = math.radians(self._state.yaw)
                self._state.x   += v_lin * math.cos(yaw_rad) * SIM_DT_S
                self._state.y   += v_lin * math.sin(yaw_rad) * SIM_DT_S
                self._state.yaw += yaw_delta

            # Encoders: distancia lineal → grados de eje
            noise_l = float(rng.normal(0.0, SIM_ENC_NOISE_MM))
            noise_r = float(rng.normal(0.0, SIM_ENC_NOISE_MM))
            dist_l  = v_l * SIM_DT_S + noise_l
            dist_r  = v_r * SIM_DT_S + noise_r
            deg_l   = dist_l / self._mm_per_deg if self._mm_per_deg > 0 else 0.0
            deg_r   = dist_r / self._mm_per_deg if self._mm_per_deg > 0 else 0.0

            with self.lock:
                self.accumulated_yaw_deg += yaw_delta
                self.last_dang            = yaw_delta
                self.enc_acum_izq        += deg_l
                self.enc_acum_der        += deg_r
                self.enc_delta_izq        = deg_l
                self.enc_delta_der        = deg_r
                self.last_rx_time         = time.time()

            time.sleep(SIM_DT_S)

    def send_command(self, izq, der, aux_pwm, comp_state):
        with self._state.lock:
            self._state.pl = float(izq)
            self._state.pr = float(der)

    def is_connected(self):
        return True

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
def _ray_seg(ox, oy, dx, dy, x1, y1, x2, y2):
    """Distancia desde (ox,oy) en dir (dx,dy) al segmento. Retorna inf si no cruza."""
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
    Genera escaneos 360° mediante raycasting contra las paredes del corral
    y obstáculos circulares dinámicos.
    """
    def __init__(self, port, baudrate=230400):
        self._state  = None   # inyectado por la fábrica
        self.running = False
        self.last_rx_time = 0.0
        self._lock   = threading.Lock()
        self._pts    = []
        self._thread = None

        # Paredes del corral
        x1, y1, x2, y2 = SIM_WALL_X1, SIM_WALL_Y1, SIM_WALL_X2, SIM_WALL_Y2
        self._walls = [
            (x1, y1, x2, y1),   # sur
            (x2, y1, x2, y2),   # este
            (x2, y2, x1, y2),   # norte
            (x1, y2, x1, y1),   # oeste
        ]
        # Lista de obstáculos circulares: [(cx, cy, radius), ...]
        self.obstacles = []

    def connect(self):
        self.running = True
        self._thread = threading.Thread(target=self._scan_loop, daemon=True)
        self._thread.start()
        return True

    def _cast(self, ox, oy, angle_deg):
        rad = math.radians(angle_deg)
        dx, dy = math.cos(rad), math.sin(rad)
        min_d  = float('inf')

        for seg in self._walls:
            d = _ray_seg(ox, oy, dx, dy, *seg)
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
        rng = np.random.default_rng(42)
        interval = 1.0 / SIM_SCAN_HZ
        while self.running:
            with self._state.lock:
                rx, ry, ryaw = self._state.x, self._state.y, self._state.yaw

            pts = []
            for a in range(0, 360, 1):
                d = self._cast(rx, ry, ryaw + a)
                d += float(rng.normal(0.0, SIM_NOISE_MM))
                if 30.0 < d < 12000.0:
                    pts.append((float(a), float(d)))

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
# CÁMARA SIMULADA
# ══════════════════════════════════════════════════════════════════
class SimulatedCamera:
    """
    Replica la interfaz de CameraCapture.
    Genera frames numpy (H×W×3 BGR) con suciedad sintética.
    La detección HSV del main verá píxeles "limpios" o "sucios" según
    SIM_DIRT_BASE y SIM_DIRT_NOISE.
    """
    def __init__(self, index=0):
        self._ok  = False
        self._rng = np.random.default_rng(index * 137 + 1)
        self._label = "trasera" if index != 0 else "frontal"

    def connect(self):
        self._ok = True
        return True

    def get_frame(self):
        H, W = 480, 640
        dirt = float(np.clip(
            self._rng.normal(SIM_DIRT_BASE, SIM_DIRT_NOISE), 0.0, 1.0))

        # Limpio: S bajo, V alto → BGR ≈ (215, 215, 215)
        # Sucio:  S alto, V bajo → BGR ≈ (55, 85, 115) (marrón)
        frame = np.full((H, W, 3), [215, 215, 215], dtype=np.uint8)
        mask  = self._rng.random((H, W)) < dirt
        frame[mask] = [55, 85, 115]
        return frame

    def is_ok(self):  return self._ok
    def close(self):  self._ok = False


# ══════════════════════════════════════════════════════════════════
# EVENTO DE TECLADO EXTRA: INYECTAR OBSTÁCULO (I) / LIMPIAR (K)
# ══════════════════════════════════════════════════════════════════
_sim_lidar_ref = None   # referencia global al SimulatedLiDAR activo
_sim_state_ref = None   # referencia global al _SimState activo

def handle_sim_keys(event):
    """Procesa teclas exclusivas de simulación. Llamar desde el event loop."""
    if event.type != pygame.KEYDOWN:
        return
    if _sim_lidar_ref is None or _sim_state_ref is None:
        return

    if event.key == pygame.K_i:
        # Inyectar obstáculo estático a SIM_OBS_FRONT_MM delante del robot
        with _sim_state_ref.lock:
            rx, ry, ryaw = _sim_state_ref.x, _sim_state_ref.y, _sim_state_ref.yaw
        rad  = math.radians(ryaw)
        obs_x = rx + math.cos(rad) * SIM_OBS_FRONT_MM
        obs_y = ry + math.sin(rad) * SIM_OBS_FRONT_MM
        _sim_lidar_ref.obstacles.append((obs_x, obs_y, SIM_OBS_RADIUS_MM))
        print(f"[SIM] Obstáculo inyectado en ({obs_x:.0f},{obs_y:.0f})  "
              f"total={len(_sim_lidar_ref.obstacles)}")

    elif event.key == pygame.K_k:
        _sim_lidar_ref.obstacles.clear()
        print("[SIM] Obstáculos eliminados.")


# ══════════════════════════════════════════════════════════════════
# BOOTSTRAP PRINCIPAL
# ══════════════════════════════════════════════════════════════════
def main():
    global _sim_lidar_ref, _sim_state_ref

    print(f"\n[SIM] Cargando: {MAIN_FILE}")
    spec = importlib.util.spec_from_file_location("robot_main", MAIN_FILE)
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    # Estado físico compartido — inicializar en el centro del mapa
    sim_state = _SimState(x0=mod.LIDAR_X_INICIAL_MM, y0=mod.LIDAR_Y_INICIAL_MM)
    _sim_state_ref = sim_state

    # Instanciar drivers con acceso al estado compartido
    _arduino = SimulatedArduino.__new__(SimulatedArduino)
    SimulatedArduino.__init__(_arduino, "SIM_ARDUINO")
    _arduino._state        = sim_state
    _arduino._vel_mm_per_pwm = mod.VEL_MM_PER_PWM
    _arduino._pwm_thr      = mod.PWM_VEL_THRESHOLD
    _arduino._wheel_track  = mod.WHEEL_TRACK_MM
    _arduino._mm_per_deg   = mod.MM_PER_DEG_WHEEL

    _lidar = SimulatedLiDAR.__new__(SimulatedLiDAR)
    SimulatedLiDAR.__init__(_lidar, "SIM_LIDAR")
    _lidar._state = sim_state
    _sim_lidar_ref = _lidar

    # Fábricas que devuelven los mismos objetos (main los crea como new)
    class _ArdFactory:
        def __new__(cls, port, baudrate=115200):
            return _arduino
    class _LidFactory:
        def __new__(cls, port, baudrate=230400):
            return _lidar
    class _CamFactory:
        def __new__(cls, index=0):
            return SimulatedCamera(index)

    # Parchear clases de hardware en el módulo
    mod.LiDAR_LD20             = _LidFactory
    mod.ArduinoSerialController = _ArdFactory
    mod.CameraCapture           = _CamFactory

    # Usar archivo de sesión separado para no interferir con datos reales
    class _SimStateManager(mod.StateManager):
        def __init__(self):
            super().__init__(filepath="sim_session.json")
    mod.StateManager = _SimStateManager

    # Parchear el event loop del main para interceptar teclas I/K
    _orig_event_get = pygame.event.get
    def _patched_event_get():
        events = _orig_event_get()
        for e in events:
            handle_sim_keys(e)
        return events
    pygame.event.get = _patched_event_get

    # Banner de inicio
    print("═" * 62)
    print("  MODO SIMULACIÓN  —  Robot Cleaner de Corrales Ganaderos")
    print(f"  Corral: ({SIM_WALL_X1:.0f},{SIM_WALL_Y1:.0f})–"
          f"({SIM_WALL_X2:.0f},{SIM_WALL_Y2:.0f}) mm  "
          f"({(SIM_WALL_X2-SIM_WALL_X1)/1000:.1f}×"
          f"{(SIM_WALL_Y2-SIM_WALL_Y1)/1000:.1f} m)")
    print(f"  Suciedad: {SIM_DIRT_BASE*100:.0f}% ± {SIM_DIRT_NOISE*100:.0f}%  |  "
          f"LiDAR {SIM_SCAN_HZ}Hz  |  Física {int(1/SIM_DT_S)}Hz")
    print("  I → inyectar obstáculo  |  K → eliminar obstáculos")
    print("  F1 → escanear paredes  |  Tab → activar ruta")
    print("═" * 62)

    # Ejecutar el main real con hardware parcheado
    mod.main()


if __name__ == "__main__":
    main()
