import serial
import struct
import threading
import time
import numpy as np
import pygame
import math
from collections import deque

# ==========================================
# 0. TABLA CRC
# ==========================================
CRC_TABLE = [
    0x00,0x4D,0x9A,0xD7,0x79,0x34,0xE3,0xAE,0xF2,0xBF,0x68,0x25,0x8B,0xC6,0x11,0x5C,
    0xA9,0xE4,0x33,0x7E,0xD0,0x9D,0x4A,0x07,0x5B,0x16,0xC1,0x8C,0x22,0x6F,0xB8,0xF5,
    0x1F,0x52,0x85,0xC8,0x66,0x2B,0xFC,0xB1,0xED,0xA0,0x77,0x3A,0x94,0xD9,0x0E,0x43,
    0xB6,0xFB,0x2C,0x61,0xCF,0x82,0x55,0x18,0x44,0x09,0xDE,0x93,0x3D,0x70,0xA7,0xEA,
    0x3E,0x73,0xA4,0xE9,0x47,0x0A,0xDD,0x90,0xCC,0x81,0x56,0x1B,0xB5,0xF8,0x2F,0x62,
    0x97,0xDA,0x0D,0x40,0xEE,0xA3,0x74,0x39,0x65,0x28,0xFF,0xB2,0x1C,0x51,0x86,0xCB,
    0x21,0x6C,0xBB,0xF6,0x58,0x15,0xC2,0x8F,0xD3,0x9E,0x49,0x04,0xAA,0xE7,0x30,0x7D,
    0x88,0xC5,0x12,0x5F,0xF1,0xBC,0x6B,0x26,0x7A,0x37,0xE0,0xAD,0x03,0x4E,0x99,0xD4,
    0x7C,0x31,0xE6,0xAB,0x05,0x48,0x9F,0xD2,0x8E,0xC3,0x14,0x59,0xF7,0xBA,0x6D,0x20,
    0xD5,0x98,0x4F,0x02,0xAC,0xE1,0x36,0x7B,0x27,0x6A,0xBD,0xF0,0x5E,0x13,0xC4,0x89,
    0x63,0x2E,0xF9,0xB4,0x1A,0x57,0x80,0xCD,0x91,0xDC,0x0B,0x46,0xE8,0xA5,0x72,0x3F,
    0xCA,0x87,0x50,0x1D,0xB3,0xFE,0x29,0x64,0x38,0x75,0xA2,0xEF,0x41,0x0C,0xDB,0x96,
    0x42,0x0F,0xD8,0x95,0x3B,0x76,0xA1,0xEC,0xB0,0xFD,0x2A,0x67,0xC9,0x84,0x53,0x1E,
    0xEB,0xA6,0x71,0x3C,0x92,0xDF,0x08,0x45,0x19,0x54,0x83,0xCE,0x60,0x2D,0xFA,0xB7,
    0x5D,0x10,0xC7,0x8A,0x24,0x69,0xBE,0xF3,0xAF,0xE2,0x35,0x78,0xD6,0x9B,0x4C,0x01,
    0xF4,0xB9,0x6E,0x23,0x8D,0xC0,0x17,0x5A,0x06,0x4B,0x9C,0xD1,0x7F,0x32,0xE5,0xA8,
]

# ==========================================
# CONFIGURACIÓN
# ==========================================
PUERTO_LIDAR = 'COM8'    # <--- VERIFICA TU PUERTO
BAUD_RATE = 230400

# Pantalla
ANCHO_VENTANA = 900
ALTO_VENTANA = 700
ESCALA_ZOOM = 0.25
MAX_PUNTOS_VISUALES = 2500 

# Dimensiones Físicas del Robot
ROBOT_LARGO_MM = 800  
ROBOT_ANCHO_MM = 600
DISTANCIA_LIDAR_FRENTE_MM = 300 

# Offset visual (270° para que el frente del robot sea ARRIBA)
OFFSET_ROTACION = 270 

# Rangos a medir
RANGOS_MEDICION = [
    ("TRASERO (Lidar 0°)", 355, 10),
    ("DERECHA (Lidar 90°)", 80, 100),
    ("FRONTAL (Lidar 180°)", 170, 190), 
    ("IZQUIERDA (Lidar 270°)", 260, 290)
]

# ==========================================
# 1. DRIVER LIDAR 
# ==========================================
class LiDAR_LD20:
    def __init__(self, port, baudrate=230400):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        self.buffer_puntos = []
        self.lock = threading.Lock()
        self.total_packets = 0
        self.crc_errors = 0

    def connect(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            self.running = True
            return True
        except: return False

    def calcular_crc8(self, data):
        crc = 0
        for byte in data:
            crc = CRC_TABLE[(crc ^ byte) & 0xFF]
        return crc

    def read_loop(self):
        buffer = b''
        while self.running:
            try:
                if self.serial.in_waiting:
                    buffer += self.serial.read(self.serial.in_waiting)
                else:
                    time.sleep(0.001)
                    continue
                
                if len(buffer) > 4000: buffer = buffer[-2000:]

                while len(buffer) >= 47:
                    idx = buffer.find(b'\x54')
                    if idx == -1:
                        buffer = b''
                        break
                    if idx > 0: buffer = buffer[idx:]
                    if len(buffer) < 47: break
                    
                    packet = buffer[:47]
                    buffer = buffer[47:]
                    
                    if packet[1] != 0x2C: continue

                    if self.calcular_crc8(packet[:-1]) != packet[-1]:
                        self.crc_errors += 1
                        continue
                    
                    self.total_packets += 1
                    start_angle = struct.unpack('<H', packet[4:6])[0] / 100.0
                    end_angle = struct.unpack('<H', packet[42:44])[0] / 100.0
                    if end_angle < start_angle: end_angle += 360
                    step = (end_angle - start_angle) / 11.0
                    
                    nuevos = []
                    for i in range(12):
                        dist = struct.unpack('<H', packet[6+(i*3):8+(i*3)])[0]
                        if dist > 0 and packet[8+(i*3)] > 100:
                            ang = (start_angle + step * i) % 360
                            nuevos.append((ang, dist))
                    
                    if nuevos:
                        with self.lock:
                            self.buffer_puntos.extend(nuevos)
            except: self.running = False

    def obtener_datos(self):
        with self.lock:
            datos = self.buffer_puntos[:]
            self.buffer_puntos = []
        return datos
    
    def obtener_stats(self):
        return self.total_packets, self.crc_errors

    def close(self):
        self.running = False
        if self.serial: self.serial.close()

# ==========================================
# 2. PROMEDIOS
# ==========================================
def calcular_promedios(nube_puntos):
    resultados = {}
    for etiqueta, _, _ in RANGOS_MEDICION:
        resultados[etiqueta] = {'suma': 0, 'count': 0, 'avg': 0}
        
    for angulo, dist in nube_puntos:
        for etiqueta, min_a, max_a in RANGOS_MEDICION:
            pertenece = False
            if min_a < max_a:
                if min_a <= angulo <= max_a: pertenece = True
            else:
                if angulo >= min_a or angulo <= max_a: pertenece = True
            
            if pertenece:
                resultados[etiqueta]['suma'] += dist
                resultados[etiqueta]['count'] += 1
    
    for k in resultados:
        if resultados[k]['count'] > 0:
            resultados[k]['avg'] = resultados[k]['suma'] / resultados[k]['count']
            
    return resultados

# ==========================================
# 3. MAIN
# ==========================================
def main():
    pygame.init()
    screen = pygame.display.set_mode((ANCHO_VENTANA, ALTO_VENTANA))
    pygame.display.set_caption("LiDAR LD20: Visualización Completa + Rectángulo")
    clock = pygame.time.Clock()
    font_med = pygame.font.SysFont("Consolas", 20, bold=True)
    font_small = pygame.font.SysFont("Arial", 14)

    lidar = LiDAR_LD20(PUERTO_LIDAR, BAUD_RATE)
    if not lidar.connect(): 
        print(f"No se pudo conectar a {PUERTO_LIDAR}")
        return

    threading.Thread(target=lidar.read_loop, daemon=True).start()

    nube_datos = deque(maxlen=MAX_PUNTOS_VISUALES)
    cx, cy = ANCHO_VENTANA // 2, ALTO_VENTANA // 2
    
    running = True
    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT: running = False

            nuevos = lidar.obtener_datos()
            total_pkts, errores_crc = lidar.obtener_stats()

            if nuevos: nube_datos.extend(nuevos)
            stats = calcular_promedios(nube_datos)

            # --- DIBUJO ---
            screen.fill((10, 10, 20))
            
            # Guías
            pygame.draw.line(screen, (30, 30, 30), (cx, 0), (cx, ALTO_VENTANA))
            pygame.draw.line(screen, (30, 30, 30), (0, cy), (ANCHO_VENTANA, cy))
            pygame.draw.circle(screen, (25, 25, 25), (cx, cy), int(1000 * ESCALA_ZOOM), 1)
            
            # --- AGREGADO: DIBUJAR RECTÁNGULO DEL ROBOT ---
            # 1. Convertir dimensiones a pixeles
            w_px = ROBOT_ANCHO_MM * ESCALA_ZOOM
            l_px = ROBOT_LARGO_MM * ESCALA_ZOOM
            
            # 2. Calcular posición (Offset)
            # El LiDAR (centro rojo) está a 300mm del frente
            dist_frente_px = DISTANCIA_LIDAR_FRENTE_MM * ESCALA_ZOOM
            
            # Coordenada superior izquierda
            rect_x = cx - (w_px / 2)
            rect_y = cy - dist_frente_px
            
            rect_robot = pygame.Rect(rect_x, rect_y, w_px, l_px)
            
            # Dibujar Caja (Amarillo)
            pygame.draw.rect(screen, (255, 255, 0), rect_robot, 2)
            # Línea de frente (Grosor extra)
            pygame.draw.line(screen, (255, 255, 0), rect_robot.topleft, rect_robot.topright, 4)

            # --- DIBUJAR PUNTOS ---
            for ang, dist in nube_datos:
                ang_visual_rad = math.radians(ang + OFFSET_ROTACION)
                px = cx + (dist * math.cos(ang_visual_rad) * ESCALA_ZOOM)
                py = cy - (dist * math.sin(ang_visual_rad) * ESCALA_ZOOM)
                try:
                    screen.set_at((int(px), int(py)), (0, 255, 128))
                except: pass

            pygame.draw.circle(screen, (255, 50, 50), (cx, cy), 4) # Punto Rojo LiDAR

            # --- INTERFAZ ---
            pygame.draw.rect(screen, (30, 30, 40), (10, 10, 360, 180))
            pygame.draw.rect(screen, (100, 100, 100), (10, 10, 360, 180), 1)

            screen.blit(font_small.render(f"CRC Err: {errores_crc} | Puntos: {len(nube_datos)}", True, (200, 200, 200)), (20, 15))
            screen.blit(font_small.render("--- DISTANCIAS PROMEDIO ---", True, (255, 255, 255)), (20, 35))

            y_pos = 60
            for etiqueta, min_a, max_a in RANGOS_MEDICION:
                val = stats[etiqueta]['avg']
                
                # --- ALERTA DINÁMICA ---
                limite_fisico = 0
                if "FRONTAL" in etiqueta:
                    limite_fisico = DISTANCIA_LIDAR_FRENTE_MM
                elif "TRASERO" in etiqueta:
                    limite_fisico = ROBOT_LARGO_MM - DISTANCIA_LIDAR_FRENTE_MM
                else: 
                    limite_fisico = ROBOT_ANCHO_MM / 2
                
                color_texto = (255, 255, 255)
                # Alerta si entra dentro de la caja amarilla + 10cm
                if 0 < val < (limite_fisico + 100):
                    color_texto = (255, 50, 50) 

                screen.blit(font_med.render(f"{etiqueta}: {val:.0f} mm", True, color_texto), (20, y_pos))
                y_pos += 30

            pygame.display.flip()
            clock.tick(60)

    except KeyboardInterrupt: pass
    finally:
        lidar.close()
        pygame.quit()

if __name__ == "__main__":
    main()