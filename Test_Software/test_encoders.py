"""
test_encoders.py — Test rápido de encoders AS5600 + giroscopio
===============================================================
Script corto para verificar:
  - Comunicación serial con el Arduino
  - Encoders izquierdo y derecho reportan cambios al mover las ruedas
  - Giroscopio reporta DANG al girar el robot

Ejecutar:
    python test_encoders.py
    python test_encoders.py --port COM3
    python test_encoders.py --port /dev/ttyACM0

Protocolo esperado (el Arduino debe enviar):
    ENC:acum_izq,acum_der,delta_izq,delta_der\\n
    DANG:delta_deg\\n

Teclas:
    R      → reset acumuladores locales
    P      → pausar lectura
    ESC    → salir
"""

import sys
import time
import math
import argparse
import threading
from collections import deque

import pygame
import serial
import serial.tools.list_ports as list_ports

# ── Configuración por defecto ─────────────────────────────────
DEFAULT_PORT     = "COM7"
DEFAULT_BAUD     = 115200
WINDOW_W         = 1000
WINDOW_H         = 640
FPS              = 30

# Colores
BG       = (14, 16, 24)
PANEL    = (24, 28, 40)
BORDER   = (60, 70, 90)
TEXT     = (230, 230, 240)
SUBTEXT  = (140, 150, 170)
OK       = (90, 220, 140)
WARN     = (255, 180, 70)
ERR      = (255, 80, 80)
ACCENT   = (100, 180, 255)
ENC_L    = (255, 200, 100)    # amarillo — encoder izquierdo
ENC_R    = (100, 200, 255)    # azul     — encoder derecho
GYRO_COL = (220, 130, 255)    # morado   — giroscopio


# ══════════════════════════════════════════════════════════════
# LECTOR SERIAL EN HILO
# ══════════════════════════════════════════════════════════════

class SerialReader:
    """Hilo que lee líneas del Arduino y acumula encoders + yaw."""

    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.ser  = None

        self.acum_izq   = 0.0
        self.acum_der   = 0.0
        self.delta_izq  = 0.0   # último delta recibido
        self.delta_der  = 0.0
        self.yaw_accum  = 0.0
        self.last_dang  = 0.0

        self.last_rx_time = 0.0
        self.msg_count    = 0
        self.enc_count    = 0
        self.dang_count   = 0
        self.last_raw     = deque(maxlen=10)   # últimas 10 líneas

        self.running  = False
        self.paused   = False
        self.error    = None
        self.lock     = threading.Lock()
        self._thread  = None

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(0.2)   # dar tiempo al Arduino de arrancar
            self.running = True
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()
            return True
        except Exception as e:
            self.error = str(e)
            return False

    def _loop(self):
        buf = b""
        while self.running:
            try:
                if self.paused:
                    time.sleep(0.05); continue
                if self.ser.in_waiting:
                    buf += self.ser.read(self.ser.in_waiting)
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        self._parse(line.decode("utf-8", errors="ignore").strip())
                else:
                    time.sleep(0.005)
            except Exception as e:
                with self.lock:
                    self.error = str(e)
                time.sleep(0.5)

    def _parse(self, line):
        if not line:
            return
        with self.lock:
            self.last_raw.append(line)
            self.msg_count   += 1
            self.last_rx_time = time.time()

        if line.startswith("ENC:"):
            try:
                parts = line[4:].split(",")
                if len(parts) == 4:
                    ai, ad, di, dd = map(float, parts)
                    with self.lock:
                        self.acum_izq  = ai
                        self.acum_der  = ad
                        self.delta_izq = di
                        self.delta_der = dd
                        self.enc_count += 1
            except Exception:
                pass
        elif line.startswith("DANG:"):
            try:
                v = float(line.split(":", 1)[1])
                with self.lock:
                    self.yaw_accum += v
                    self.last_dang  = v
                    self.dang_count += 1
            except Exception:
                pass

    def reset_accum(self):
        with self.lock:
            self.acum_izq  = 0.0
            self.acum_der  = 0.0
            self.yaw_accum = 0.0

    def toggle_pause(self):
        self.paused = not self.paused

    def close(self):
        self.running = False
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass

    def snapshot(self):
        """Copia thread-safe del estado actual para el render."""
        with self.lock:
            return {
                "acum_izq":     self.acum_izq,
                "acum_der":     self.acum_der,
                "delta_izq":    self.delta_izq,
                "delta_der":    self.delta_der,
                "yaw_accum":    self.yaw_accum,
                "last_dang":    self.last_dang,
                "msg_count":    self.msg_count,
                "enc_count":    self.enc_count,
                "dang_count":   self.dang_count,
                "last_rx":      self.last_rx_time,
                "last_raw":     list(self.last_raw),
                "error":        self.error,
            }


# ══════════════════════════════════════════════════════════════
# RENDER — WIDGETS GRÁFICOS
# ══════════════════════════════════════════════════════════════

def draw_wheel_gauge(screen, font, small, x, y, r,
                     label, color, acum_deg, delta_deg):
    """
    Círculo con una flecha que rota según el acumulado.
    Muestra acum_deg (total) y delta_deg (cambio instantáneo).
    """
    # Círculo base
    pygame.draw.circle(screen, (36, 40, 58), (x, y), r, 0)
    pygame.draw.circle(screen, BORDER, (x, y), r, 2)

    # Marcas cada 45°
    for a in range(0, 360, 45):
        rad = math.radians(a)
        x1 = x + (r - 8) * math.cos(rad)
        y1 = y + (r - 8) * math.sin(rad)
        x2 = x + r       * math.cos(rad)
        y2 = y + r       * math.sin(rad)
        pygame.draw.line(screen, (70, 80, 100),
                         (int(x1), int(y1)), (int(x2), int(y2)), 1)

    # Aguja según acumulado (mod 360 para visualización)
    vis_angle = acum_deg % 360.0
    rad = math.radians(vis_angle - 90)   # -90 para que 0° apunte arriba
    hx = x + (r - 14) * math.cos(rad)
    hy = y + (r - 14) * math.sin(rad)
    pygame.draw.line(screen, color, (x, y), (int(hx), int(hy)), 4)
    pygame.draw.circle(screen, color, (x, y), 6)

    # Indicador visual de delta: flecha curva en el borde
    if abs(delta_deg) > 0.05:
        # Arco del último delta
        arc_sign = 1 if delta_deg > 0 else -1
        pygame.draw.circle(screen, color, (x, y),
                           r + 6, 3 if abs(delta_deg) > 2 else 1)

    # Label centrado debajo
    lbl = font.render(label, True, color)
    screen.blit(lbl, (x - lbl.get_rect().width // 2, y + r + 8))

    # Acumulado
    acc_txt = f"{acum_deg:+8.1f}°"
    acc = font.render(acc_txt, True, TEXT)
    screen.blit(acc, (x - acc.get_rect().width // 2, y + r + 32))

    # Delta (velocidad instantánea)
    delta_col = OK if abs(delta_deg) > 0.1 else SUBTEXT
    delta_txt = f"Δ {delta_deg:+6.2f}°"
    ds = small.render(delta_txt, True, delta_col)
    screen.blit(ds, (x - ds.get_rect().width // 2, y + r + 58))


def draw_gyro_dial(screen, font, small, x, y, r, yaw_deg, last_dang):
    """Dial grande del giroscopio con norte arriba."""
    # Fondo circular
    pygame.draw.circle(screen, (28, 24, 40), (x, y), r, 0)
    pygame.draw.circle(screen, GYRO_COL, (x, y), r, 2)

    # Marcas cardinales
    for a, txt in [(0, "N"), (90, "E"), (180, "S"), (270, "W")]:
        rad = math.radians(a - 90)
        tx = x + (r - 20) * math.cos(rad)
        ty = y + (r - 20) * math.sin(rad)
        t = font.render(txt, True, SUBTEXT)
        screen.blit(t, (int(tx) - t.get_rect().width // 2,
                        int(ty) - t.get_rect().height // 2))

    # Marcas cada 30°
    for a in range(0, 360, 30):
        rad = math.radians(a - 90)
        r1  = r - 10
        r2  = r - 4
        x1, y1 = x + r1*math.cos(rad), y + r1*math.sin(rad)
        x2, y2 = x + r2*math.cos(rad), y + r2*math.sin(rad)
        pygame.draw.line(screen, (90, 100, 130),
                         (int(x1), int(y1)), (int(x2), int(y2)), 1)

    # Aguja del yaw
    vis = yaw_deg % 360.0
    rad = math.radians(vis - 90)
    hx = x + (r - 30) * math.cos(rad)
    hy = y + (r - 30) * math.sin(rad)
    pygame.draw.line(screen, GYRO_COL, (x, y), (int(hx), int(hy)), 5)
    pygame.draw.circle(screen, GYRO_COL, (x, y), 8)
    pygame.draw.circle(screen, (14, 16, 24), (x, y), 4)

    # Label y valor
    lbl = font.render("GYRO / YAW", True, GYRO_COL)
    screen.blit(lbl, (x - lbl.get_rect().width // 2, y + r + 10))

    val_col = OK if abs(last_dang) > 0.05 else TEXT
    val_txt = f"{yaw_deg:+9.1f}°"
    val_font = pygame.font.SysFont("Consolas", 22, bold=True)
    val = val_font.render(val_txt, True, val_col)
    screen.blit(val, (x - val.get_rect().width // 2, y + r + 34))

    # Último DANG (delta)
    dang_col = OK if abs(last_dang) > 0.05 else SUBTEXT
    dang_txt = f"Δ DANG: {last_dang:+6.3f}°"
    d = small.render(dang_txt, True, dang_col)
    screen.blit(d, (x - d.get_rect().width // 2, y + r + 68))


def draw_velocity_bar(screen, small, x, y, w, h, delta_deg, max_deg=10.0, color=ACCENT):
    """Barra horizontal centrada que muestra velocidad instantánea."""
    # Fondo
    pygame.draw.rect(screen, (30, 34, 48), (x, y, w, h), border_radius=3)
    pygame.draw.rect(screen, BORDER, (x, y, w, h), 1, border_radius=3)

    # Línea de cero
    cx = x + w // 2
    pygame.draw.line(screen, (80, 90, 110), (cx, y), (cx, y + h), 1)

    # Barra proporcional
    ratio = max(-1.0, min(1.0, delta_deg / max_deg))
    bar_w = int(abs(ratio) * (w // 2 - 2))
    if bar_w > 0:
        bx = cx if ratio > 0 else cx - bar_w
        pygame.draw.rect(screen, color,
                         (bx, y + 2, bar_w, h - 4), border_radius=2)


def draw_status_card(screen, font, small, x, y, w, h, snap, port, baud):
    """Card de conexión + contadores."""
    pygame.draw.rect(screen, PANEL, (x, y, w, h), border_radius=8)
    pygame.draw.rect(screen, BORDER, (x, y, w, h), 1, border_radius=8)

    screen.blit(font.render("CONEXIÓN", True, TEXT), (x + 12, y + 8))

    # Puerto y baud
    screen.blit(small.render(f"Puerto: {port}", True, SUBTEXT),
                (x + 12, y + 36))
    screen.blit(small.render(f"Baud:   {baud}", True, SUBTEXT),
                (x + 12, y + 54))

    # Estado de conexión
    now = time.time()
    age = now - snap["last_rx"] if snap["last_rx"] > 0 else 999
    if snap["error"]:
        status, col = f"ERROR: {snap['error'][:30]}", ERR
    elif age < 0.5:
        status, col = "RX OK (activo)", OK
    elif age < 3.0:
        status, col = f"RX lento ({age:.1f}s)", WARN
    else:
        status, col = "SIN DATOS", ERR
    screen.blit(small.render(status, True, col), (x + 12, y + 76))

    # Contadores
    screen.blit(small.render(f"Msgs: {snap['msg_count']}", True, SUBTEXT),
                (x + 12, y + 100))
    screen.blit(small.render(f"ENC:  {snap['enc_count']}", True, ENC_L),
                (x + 12, y + 118))
    screen.blit(small.render(f"DANG: {snap['dang_count']}", True, GYRO_COL),
                (x + 12, y + 136))


def draw_raw_log(screen, font, small, x, y, w, h, raw_lines):
    """Log raw de las últimas líneas del serial."""
    pygame.draw.rect(screen, PANEL, (x, y, w, h), border_radius=8)
    pygame.draw.rect(screen, BORDER, (x, y, w, h), 1, border_radius=8)
    screen.blit(font.render("RAW SERIAL", True, TEXT), (x + 12, y + 8))

    line_y = y + 34
    mono = pygame.font.SysFont("Consolas", 12)
    for line in raw_lines[-10:]:
        if line.startswith("ENC:"):
            col = ENC_L
        elif line.startswith("DANG:"):
            col = GYRO_COL
        else:
            col = SUBTEXT
        text = line if len(line) < 60 else line[:57] + "..."
        screen.blit(mono.render(text, True, col), (x + 12, line_y))
        line_y += 15


def draw_controls_hint(screen, small, x, y):
    """Recordatorio de teclas."""
    hints = [
        ("R",    "Reset acumuladores"),
        ("P",    "Pausar lectura"),
        ("ESC",  "Salir"),
    ]
    for key, desc in hints:
        k = small.render(f"[{key}]", True, ACCENT)
        d = small.render(desc, True, SUBTEXT)
        screen.blit(k, (x, y))
        screen.blit(d, (x + 48, y))
        y += 18


# ══════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="Test encoders AS5600 + giroscopio")
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help=f"Puerto serial (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                        help=f"Baudrate (default: {DEFAULT_BAUD})")
    parser.add_argument("--list", action="store_true",
                        help="Listar puertos disponibles y salir")
    args = parser.parse_args()

    if args.list:
        print("Puertos disponibles:")
        for p in list_ports.comports():
            print(f"  {p.device}: {p.description}")
        return

    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption(f"Test Encoders — {args.port} @ {args.baud}")
    clock = pygame.time.Clock()

    title_font = pygame.font.SysFont("Arial", 22, bold=True)
    font       = pygame.font.SysFont("Arial", 15, bold=True)
    small      = pygame.font.SysFont("Consolas", 13)

    # Conectar serial
    reader = SerialReader(args.port, args.baud)
    print(f"[TEST] Conectando a {args.port} @ {args.baud}...")
    ok = reader.connect()
    if not ok:
        print(f"[TEST] ERROR: {reader.error}")
        print("[TEST] Listando puertos disponibles:")
        for p in list_ports.comports():
            print(f"  {p.device}: {p.description}")
        # Continuar de todos modos para mostrar el error en la UI

    try:
        while True:
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    return
                if e.type == pygame.KEYDOWN:
                    if e.key == pygame.K_ESCAPE:
                        return
                    elif e.key == pygame.K_r:
                        reader.reset_accum()
                        print("[TEST] Acumuladores reseteados")
                    elif e.key == pygame.K_p:
                        reader.toggle_pause()
                        print(f"[TEST] Pausa: {reader.paused}")

            snap = reader.snapshot()

            # ── Render ─────────────────────────────────────
            screen.fill(BG)

            # Título
            title = title_font.render(
                "TEST DE ENCODERS + GIROSCOPIO", True, TEXT)
            screen.blit(title, (20, 12))

            pause_hint = ""
            if reader.paused: pause_hint = " — PAUSADO"
            subtitle = small.render(
                f"{args.port} @ {args.baud}{pause_hint}",
                True, WARN if reader.paused else SUBTEXT)
            screen.blit(subtitle, (20, 40))

            # Gyro dial (centro-izquierda)
            draw_gyro_dial(screen, font, small,
                           x=200, y=260, r=120,
                           yaw_deg=snap["yaw_accum"],
                           last_dang=snap["last_dang"])

            # Encoder izquierdo
            draw_wheel_gauge(screen, font, small,
                             x=500, y=240, r=80,
                             label="ENC IZQUIERDO",
                             color=ENC_L,
                             acum_deg=snap["acum_izq"],
                             delta_deg=snap["delta_izq"])
            # Barra de velocidad izq
            draw_velocity_bar(screen, small,
                              x=420, y=380, w=160, h=14,
                              delta_deg=snap["delta_izq"],
                              color=ENC_L)

            # Encoder derecho
            draw_wheel_gauge(screen, font, small,
                             x=720, y=240, r=80,
                             label="ENC DERECHO",
                             color=ENC_R,
                             acum_deg=snap["acum_der"],
                             delta_deg=snap["delta_der"])
            draw_velocity_bar(screen, small,
                              x=640, y=380, w=160, h=14,
                              delta_deg=snap["delta_der"],
                              color=ENC_R)

            # Status card
            draw_status_card(screen, font, small,
                             x=20, y=440, w=280, h=170,
                             snap=snap,
                             port=args.port, baud=args.baud)

            # Raw log
            draw_raw_log(screen, font, small,
                         x=320, y=440, w=460, h=170,
                         raw_lines=snap["last_raw"])

            # Controles
            draw_controls_hint(screen, small, x=820, y=460)

            pygame.display.flip()
            clock.tick(FPS)

    finally:
        reader.close()
        pygame.quit()


if __name__ == "__main__":
    main()
