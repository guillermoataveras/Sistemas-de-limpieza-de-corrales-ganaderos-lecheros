import math
import time
import struct
import threading
import numpy as np
import pygame
import serial
import json
import zlib
import base64
import os
import datetime

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
CELDA_MM = 50

GRID_COLS = MAPA_ANCHO_MM // CELDA_MM
GRID_ROWS = MAPA_ALTO_MM // CELDA_MM

# Referencia global = centro del LiDAR
# Se coloca al centro del mapa para que toda la cuadrícula inicial
# del robot esté siempre contenida dentro del mapa visible.
LIDAR_X_INICIAL_MM = MAPA_ANCHO_MM / 2
LIDAR_Y_INICIAL_MM = MAPA_ALTO_MM / 2
LIDAR_YAW_INICIAL_DEG = 0.0

# ═══════════════════════════════════════════════════════
# ESPEJO EN X DE LA NUBE LIDAR
# ═══════════════════════════════════════════════════════
# La nube del LiDAR aparece espejada en X respecto al marco real.
# Este flag la refleja al depositarse en transform_scan_*. Pon False
# si no tienes el problema de espejo.
LIDAR_MIRROR_X = False # No 

# ═══════════════════════════════════════════════════════
# ROTACIÓN INICIAL DEL FRAME (rota TODO al arrancar)
# ═══════════════════════════════════════════════════════
# ★ Esta es la ÚNICA constante para rotar el mundo del robot.
# Edita este valor y reinicia: todo arranca rotado por este offset.
#
# Qué afecta:
#   - Orientación inicial del robot dibujado
#   - Dirección "adelante" para navegación y ROIs
#   - Depósito de la nube LiDAR en el mapa
#   - Centro del cuadrado de referencia
#
# Qué NO afecta:
#   - Rotación visual del grid (esa se controla con F5)
#   - Yaw relativo detectado por el giroscopio (siempre parte en 0)
#
# Ejemplos: 0.0 = el robot mira hacia +Y del mapa (norte/arriba)
#          90.0 = el robot mira hacia +X del mapa (este/derecha)
#         180.0 = el robot mira hacia -Y del mapa (sur/abajo)
#         -90.0 = el robot mira hacia -X del mapa (oeste/izquierda)
STARTUP_YAW_OFFSET_DEG = 90.0


# Build simplificada activa: navegación basada en LiDAR + giroscopio.
SIMPLIFIED_ROUTE_BUILD = True

# ==========================================
# GEOMETRÍA DEL ROBOT
# Referencia = centro del LiDAR
# ==========================================
ROBOT_LARGO_MM = 640
ROBOT_ANCHO_MM = 460
DIST_LIDAR_FRENTE_MM = 180
DIST_LIDAR_ATRAS_MM = ROBOT_LARGO_MM - DIST_LIDAR_FRENTE_MM
MITAD_ANCHO_MM = ROBOT_ANCHO_MM / 2

OFFSET_LIDAR_FISICO = 186.0

# ═══════════════════════════════════════════════════════
# CUADRADO DE REFERENCIA (DIAGNÓSTICO)
# ═══════════════════════════════════════════════════════
# Cuadrado fijo en el mapa que representa un "corral de prueba" real,
# centrado en la posición inicial del robot. Sirve para:
#   - Verificar que la nube LiDAR calza con las paredes reales conocidas
#   - Validar visualmente la rotación de pose (tecla G)
#   - A futuro: ser el mapa de referencia para el scan matching (Fase 3)
#
# Está centrado en el CENTRO DEL CUERPO del robot en pose inicial
# (no en el LiDAR, que está descentrado). Esto se calcula al arrancar
# en main() y queda fijo para toda la sesión.
#
# Tecla F4: toggle visibilidad
REFERENCE_SQUARE_ENABLED   = True       # mostrar por defecto
REFERENCE_SQUARE_SIDE_MM   = 2200.0     # lado del cuadrado (2.2 m)
COLOR_REF_SQUARE           = (80, 255, 255)    # cian brillante
COLOR_REF_SQUARE_FILL      = (80, 255, 255, 10)  # relleno semitransparente

# ═══════════════════════════════════════════════════════
# SCAN MATCH CONTRA EL CUADRADO (localización XY + yaw)
# ═══════════════════════════════════════════════════════
# El matching corre cada tick y corrige la pose de forma continua.
# XY: se promedian errores perpendiculares por dirección de pared.
# Yaw: se ajusta una línea a los hits de cada pared y se mide la
#      desviación angular. El yaw resultante corrige saved_yaw_offset.
REF_MATCH_MAX_DIST_MM      = 350.0   # hit más lejos que esto se ignora
REF_MATCH_MIN_PAIRS        = 20      # pares mínimos para aceptar match
REF_MATCH_AUTO_EVERY_TICKS = 1       # match automático cada N ticks (1 = cada tick)

# Yaw match (sub-rutina del scan match)
REF_YAW_MIN_HITS_PER_WALL  = 6       # hits mínimos en una pared para estimar su ángulo
REF_YAW_MAX_CORRECTION_DEG = 5.0     # límite de corrección por llamada (clamp)
REF_YAW_ALPHA              = 0.30    # damping: fracción aplicada por llamada

# Corrección XY con damping para evitar oscilaciones al ir tick-a-tick
REF_XY_ALPHA               = 0.50    # fracción aplicada por llamada (XY)

# Giro conservador: no matchear durante giros fuertes. El LiDAR LD20
# está barriendo de forma continua y durante un giro rápido la nube
# se distorsiona. |pl - pr| >= este umbral → pausar match.
REF_MATCH_TURN_PAUSE_PWM   = 60.0    # PWM diferencial a partir del cual pausar

# Confianza de pose: si no hay match exitoso en X ticks, la pose se
# considera incierta y en modo RUTA el robot se detiene automáticamente.
POSE_CONFIDENCE_MAX_TICKS  = 60      # ~2s a 30 FPS

# ==========================================
# ODOMETRÍA / ESTIMACIÓN DE POSE
# ==========================================
WHEEL_TRACK_MM     = 400.0   # separación centro-centro ruedas motrices (medida real)

# Umbral para clasificar "robot se está moviendo" según PWM de comando
PWM_VEL_THRESHOLD  = 20

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

# Nota: el yaw NUNCA se corrige por scan matching. Siempre viene del
# giroscopio (MPU9250 vía Arduino). El matching solo corrige posición XY.

# ═══════════════════════════════════════════════════════
# MATCHING PAUSE DURANTE GIROS
# ═══════════════════════════════════════════════════════
# Durante giros (PWM diferencial >=50) el LiDAR todavía está
# integrando el barrido — matchear produce saltos erráticos.
# Pausamos y damos settle de varios ticks antes de re-matchear.
MATCH_TURN_PWM_DIFF_THR     = 50.0     # si |pl-pr| >= este valor: en giro
MATCH_TURN_SETTLE_TICKS     = 2       # ticks de espera tras parar giro

# ==========================================
# FASE D — CLASIFICADOR GEOMÉTRICO + BLOQUEO FRONTAL
# ==========================================

# Segmentación de hits
SEG_CLUSTER_GAP_MM   = 200.0  # distancia máx entre hits consecutivos del mismo segmento
SEG_MIN_POINTS       = 8      # puntos mínimos para analizar un segmento
SEG_LINE_RESIDUAL_MM = 55.0   # residual máx RMS para considerar segmento "recto" (pared)
                               # por encima → irregular → objeto dinámico

# Cono de detección frontal
FRONT_HALF_ANGLE_DEG = 35.0   # semiángulo del cono (±35° respecto al heading)
FRONT_MAX_DIST_MM    = 500.0  # distancia máxima dentro del cono (+100mm buffer para escape)

# Confirmación / despejado de bloqueo
BLOCK_MIN_IRREG_HITS = 5      # hits irregulares mínimos en el cono por tick
BLOCK_CONFIRM_TICKS  = 12     # ticks consecutivos con obstáculo → BLOCKED
BLOCK_CLEAR_TICKS    = 6      # ticks consecutivos sin obstáculo → despejado

# ═══════════════════════════════════════════════════════
# ESCAPE POR ROTACIÓN 90° (cuando cono detecta bloqueo)
# ═══════════════════════════════════════════════════════
# Si el cono frontal detecta obstáculo, el robot rota 90° en un sentido
# fijo (derecha por defecto) hasta encontrar camino despejado. Nunca
# retrocede. Si tras 4 rotaciones (360°) sigue bloqueado, marca ese WP
# como inalcanzable.
ESCAPE_ROTATION_DEG     = 90.0   # cuánto rota en cada intento
ESCAPE_ROTATION_SIGN    = +1.0   # +1 = derecha (CW), -1 = izquierda (CCW)
ESCAPE_ROTATION_TOL_DEG = 5.0    # tolerancia de alineación al target de rotación
ESCAPE_MAX_ATTEMPTS     = 4      # 4 × 90° = giro completo 360°

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
PATRON_BOWTIE    = "BOW-TIE" # Me gustaria simplificar este

PATRONES_CICLO   = [PATRON_NINGUNO, PATRON_MATRICIAL,
                    PATRON_ESPIRAL, PATRON_BOWTIE]

# Paso entre pasadas = diámetro de limpieza exacto (sin solapamiento)
# RADIO_LIMPIEZA_MM = 220 → diámetro = 440 mm
PASO_LIMPIEZA_MM = 440   # mm  (= RADIO_LIMPIEZA_MM * 2)

# Margen desde las paredes para no chocar
MARGEN_PARED_MM  = 250

# Radio de giro efectivo para filtrado de waypoints:
# el robot necesita girar al final de cada fila sin que su cono
# frontal choque con la pared → usar el mayor entre medio ancho del
# cuerpo y mitad del cono de detección frontal.
WP_TURN_RADIUS_MM = int(MARGEN_PARED_MM
                        + max(ROBOT_ANCHO_MM // 2, int(FRONT_MAX_DIST_MM) // 2))

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
CLEAN_THRESHOLD_DEFAULT = 70.0   # % de cobertura para considerar "limpio"
CLEAN_THRESHOLD_STEP    =  5.0   # paso al ajustar con [ / ]
CLEAN_THRESHOLD_MIN     = 25.0
CLEAN_THRESHOLD_MAX     = 100.0

# Bordes de suciedad para selección de patrón automático
REPLAN_BORDER_RATIO_ESPIRAL  = 0.55  # >55% sucio en bordes → ESPIRAL
REPLAN_ASPECT_RATIO_MATRICIAL = 1.4  # zona alargada → MATRICIAL

COLOR_CLEANABLE   = (220, 235, 255)
COLOR_THRESH_OK   = ( 80, 220, 130)
COLOR_THRESH_WARN = (255, 180,  60)

# ==========================================
# CÁMARA FRONTAL — ROI Y VISIÓN
# ==========================================
CAM_INDEX            = 1        # índice de la cámara (0 = primera disponible)
CAM_WIDTH_PX         = 640
CAM_HEIGHT_PX        = 480
CAM_FPS              = 30

# ROI proyectado en el mapa (rectángulo delante del robot)
CAM_ROI_OFFSET_MM    = 30.0    # distancia desde el frente del robot al borde del ROI
CAM_ROI_LARGO_MM     = 50.0    # largo del rectángulo (dirección de avance)
CAM_ROI_ANCHO_MM     = 110.0    # ancho del rectángulo (perpendicular)

# Umbrales de suciedad (lógica invertida: blanco = sucio)
CAM_WHITE_THRESH     = 200      # valor de gris: píxel "blanco/sucio" si > este
CAM_CLEAN_RATIO      = 0.70     # clean_ratio > 0.70 → tile LIMPIO (poco blanco)
CAM_MEDIUM_RATIO     = 0.40     # 0.40–0.70 → MEDIO  /  <0.40 → SUCIO (muy blanco)

# EMA para estabilizar dirt_ratio entre frames
CAM_EMA_ALPHA        = 0.30

# PWM auxiliar adaptativo por nivel de suciedad
CAM_PWM_CLEAN        = 60
CAM_PWM_MEDIUM       = 100
CAM_PWM_DIRTY        = 180

# Etiquetas (texto visible al usuario)
CAM_LABEL_CLEAN  = "LIMPIO"
CAM_LABEL_MEDIUM = "MEDIO"
CAM_LABEL_DIRTY  = "SUCIO"

# Colores del ROI en el mapa
COLOR_ROI_BORDER      = (  0, 230, 100)   # verde — ROI frontal
COLOR_REAR_ROI_BORDER = (200, 100, 255)   # morado — ROI trasero

# ── Cámara trasera ────────────────────────────────────
CAM_REAR_INDEX              = 2
CAM_REAR_ROI_OFFSET_MM      = 40.0
CAM_REAR_ROI_LARGO_MM       = 50.0
CAM_REAR_ROI_ANCHO_MM       = 110.0
CAM_REAR_DIRT_FEEDBACK_THR  = 0.45   # dirt_ratio trasero mínimo para marcar celda

# ── Detección HSV (reemplaza umbral de grises) ────────
# Limpio = S bajo + V alto  |  Sucio = S alto o V bajo
CAM_HSV_S_MAX  = 45    # saturación máxima de pixel "limpio"
CAM_HSV_V_MIN  = 155   # luminosidad mínima de pixel "limpio"

# ── Panel colapsable ──────────────────────────────────
PANEL_SECTIONS     = ["POSE", "RUTA", "CAMARA", "SISTEMA"]
# Mapeo tecla → sección (pygame.K_7 etc. se buscan en el evento)
CAM_PREVIEW_W      = 158
CAM_PREVIEW_H      = 118
CAM_REAR_PREVIEW_H = 56

# ==========================================
# LIDAR
# ==========================================
LIDAR_MIN_MM = 30
LIDAR_MAX_MM = 8000

LIDAR_BIN_DEG = 1.0
LIDAR_TTL_S = 0.45
LIDAR_MAX_JUMP_MM = 250
LIDAR_CONFIRM_NEEDED = 2
LIDAR_EMA_ALPHA = 0.35

# ==========================================
# CONTROL MANUAL
# ==========================================
PWM_BASE = 150
PWM_GIRO = 90
PWM_STEP = 5
PWM_MIN = 35
PWM_MAX = 180
SIGNO_MOTOR_IZQ = -1
SIGNO_MOTOR_DER = -1

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
WP_REACH_MM        = 200  # distancia para considerar waypoint alcanzado
COLOR_RUTA_LINE    = (255, 200,  50)
COLOR_WP_NORMAL    = (255, 160,  30)
COLOR_WP_ACTIVE    = ( 80, 220, 255)
COLOR_WP_DONE      = (100, 200, 120)
COLOR_WP_BLOCKED   = (255,  70,  70)
COLOR_MODE_MANUAL  = (120, 255, 140)
COLOR_MODE_RUTA    = ( 80, 200, 255)

# ==========================================
# FOLLOWER — MÁQUINA DE ESTADOS (rediseñada)
# ==========================================
# Filosofía: el robot NUNCA retrocede. Siempre rota en sitio hasta
# apuntar al WP (o escape por 90°), y luego avanza de frente.
#
# Flujo normal (ir a un WP):
#   IDLE → ROTATE → ROTATE_SETTLE → ROTATE → ... → ADVANCE → WP_REACHED → ...
#
# Flujo bloqueo frontal:
#   ADVANCE → BLOCKED_ROTATE → ROTATE_SETTLE → BLOCKED_ROTATE → ...
#     (rota en ESCAPE_ROTATION_SIGN × 90° hasta encontrar despejado)
FSM_IDLE           = "IDLE"
FSM_ROTATE         = "ROTATE"          # rotando en sitio hacia el WP
FSM_ROTATE_SETTLE  = "ROTATE_SETTLE"   # pausa tras burst de giro, esperando match
FSM_ADVANCE        = "ADVANCE"         # avanzando recto hacia WP
FSM_WP_REACHED     = "WP_REACHED"
FSM_ROUTE_DONE     = "ROUTE_DONE"
FSM_BLOCKED_ROTATE = "BLOCKED_ROTATE"  # escape por rotación 90° ante bloqueo
FSM_SLOWDOWN_OBS   = "SLOWDOWN"        # frenando proporcionalmente
FSM_RETURN_HOME    = "RETURN_HOME"     # regresando al home (usa ROTATE+ADVANCE)

# Legacy — aliases para compatibilidad con código que aún pueda referenciarlos.
# Todos apuntan a los nuevos estados equivalentes para que un cambio
# incremental no rompa el archivo mientras se refactoriza.
FSM_ALIGN   = FSM_ROTATE            # ALIGN (girar) → ROTATE
FSM_BLOCKED = FSM_BLOCKED_ROTATE    # BLOCKED ahora rota para escapar

# Parámetros de giro con pausas para match
ROTATE_BURST_MS       = 300       # duración del burst de giro antes de pausa (ms)
ROTATE_SETTLE_MS      = 300       # duración de la pausa para que el match converja (ms)

# Umbrales de alineación
ALIGN_START_DEG       = 10.0      # legacy — no usado con la nueva FSM
ALIGN_FINE_DEG        =  3.0      # nuevo — error angular que permite pasar ROTATE → ADVANCE
ALIGN_DONE_DEG        =  4.0      # legacy — mantenido para follow_route_step si lo usa

# PWM del follower
PWM_FOLLOWER_ALIGN   = 65
PWM_FOLLOWER_ADVANCE = 85
PWM_FOLLOWER_MIN     = 35
K_HEADING            =  1.6   # ganancia proporcional (fallback sin PID)

# ── PID de heading ───────────────────────────────────
PID_KP          =  1.6    # proporcional
PID_KI          =  0.018  # integral
PID_KD          =  0.35   # derivativo
PID_MAX_I       = 25.0    # saturación del integral (grados)

# ── Detección de deslizamiento ───────────────────────
# ── ICP iterativo ────────────────────────────────────
ICP_ITERATIONS        = 4
ICP_CONV_THRESH_MM    = 0.8

# ── Espaciado adaptativo entre filas ─────────────────
ADAPTIVE_PASO_MIN_MM  = 260
ADAPTIVE_PASO_STEP_MM =  40
ADAPTIVE_DIRTY_CELLS  =   3

# ── Auto-reconexión ──────────────────────────────────
RECONNECT_TIMEOUT_S   =  4.0
RECONNECT_RETRY_S     =  1.5

# ── Log de sesión ────────────────────────────────────
LOG_EVERY_N_TICKS     = 30

# ── A* — búsqueda de camino sobre celdas limpiables ──
ASTAR_MAX_EXPANSIONS  = 5000    # límite de nodos expandidos (seguridad)
ASTAR_WALL_CLEARANCE  = MARGEN_PARED_MM   # celdas con pared más cerca son inválidas

# ── Migas de pan (breadcrumb trail) ──────────────────
BREADCRUMB_SPACING_MM = 200.0   # cada cuántos mm guardar una posición
BREADCRUMB_MAX        = 200     # longitud máxima del trail (rolling)

# ==========================================
# OBSTÁCULOS — DESACELERACIÓN PROPORCIONAL
# ==========================================
# Zona dinámica (animales, personas): frena antes y espera más
SLOW_ZONE_DYN_MM = 600.0   # empieza a frenar al detectar dinámico a esta distancia
STOP_ZONE_DYN_MM = 250.0   # para completamente dentro de esta distancia

# Zona estática (paredes, objetos fijos): distancias menores
SLOW_ZONE_STA_MM = 400.0
STOP_ZONE_STA_MM = 180.0

# ═══════════════════════════════════════════════════════
# MODO OPERATIVO LiDAR-ONLY
# ═══════════════════════════════════════════════════════
# El robot se mueve más despacio para que el LiDAR pueda seguir
# la pose con precisión. Estos valores sustituyen directamente a
# los defaults genéricos cuando se aplican al follower / matching.

OP_PWM_BASE        = 50      # PWM base de avance en ruta
OP_MATCH_EVERY     = 1       # matching cada N ticks (1 = cada tick)
OP_ICP_ITERS       = 4       # iteraciones de ICP por llamada
OP_SLOW_MULT       = 1.3     # zonas de slowdown 30% más amplias
OP_FOLLOW_ALIGN    = 70      # follower PWM align
OP_FOLLOW_ADVANCE  = 85      # follower PWM advance

# ═══════════════════════════════════════════════════════
# POSE DE CONTROL (PRUEBA) — LiDAR solo observa, no recoloca
# ═══════════════════════════════════════════════════════
# Para probar una navegación físicamente más coherente, la pose usada
# por el robot se integra solo con gyro + modelo simple por PWM.
# La nube LiDAR sigue usándose para mapa, visualización y obstáculos,
# pero NO corrige la pose XY ni el yaw operativo del robot.
USE_LIDAR_POSE_CORRECTION = False
USE_LIDAR_YAW_CORRECTION  = False

# ═══════════════════════════════════════════════════════
# GRID DE SUCIEDAD — COLOREADO POR CELDAS
# ═══════════════════════════════════════════════════════
# Cada celda del grid tiene un nivel de suciedad 0-1 actualizado por la cámara.
# Se muestra visualmente con 3 colores: limpio/medio/sucio.

DIRT_THRESHOLD_CLEAN   = 0.25     # dirt_ratio < este = tile LIMPIO
DIRT_THRESHOLD_MEDIUM  = 0.60     # dirt_ratio < este = tile MEDIO (resto = SUCIO)
DIRT_EMA_ALPHA         = 0.35     # factor de suavizado de mediciones de cámara
DIRT_GRID_ENABLED      = True     # habilita el grid de suciedad

COLOR_TILE_CLEAN       = (80, 200, 120)    # verde claro
COLOR_TILE_MEDIUM      = (230, 180, 70)    # amarillo/naranja
COLOR_TILE_DIRTY       = (220, 90, 70)     # rojo

# ═══════════════════════════════════════════════════════
# VELOCIDAD ADAPTATIVA POR SUCIEDAD DEL TILE DE ADELANTE
# ═══════════════════════════════════════════════════════
SPEED_MULT_CLEAN       = 1.00    # tile limpio → velocidad normal
SPEED_MULT_MEDIUM      = 0.75    # tile medio → 75% velocidad
SPEED_MULT_DIRTY       = 0.50    # tile sucio → 50% velocidad
LOOKAHEAD_CELLS_DIRT   = 2       # celdas hacia adelante para sampling

# ═══════════════════════════════════════════════════════
# SPACING MÍNIMO ENTRE WAYPOINTS GENERADOS
# ═══════════════════════════════════════════════════════
WP_MIN_SPACING_X_CELLS = 5       # mínimo 5 celdas (≈500mm) entre waypoints en X
WP_MIN_SPACING_Y_CELLS = 5       # mínimo 5 celdas entre waypoints en Y
ROUTE_MARGIN_CELLS     = 5       # margen configurable desde paredes (celdas)

# ═══════════════════════════════════════════════════════
# REPLAN POR FILAS SUCIAS
# ═══════════════════════════════════════════════════════
DIRT_REPLAN_MIN_TILES_PER_ROW = 3  # filas con ≥N tiles sucios/medios → limpiar

# ═══════════════════════════════════════════════════════

# ANTI-STUCK (detección de atascamiento y skip de WP)
# ═══════════════════════════════════════════════════════
STUCK_ALIGN_TICKS        = 240      # ticks en ROTATE sin éxito (≈8s a 30FPS)
                                     # Subido para acomodar el ciclo burst+settle:
                                     # cada giro real dura 300ms + 300ms pausa,
                                     # así que el robot sólo gira ~50% del tiempo.
STUCK_POSE_CHANGE_MM     = 50.0     # mm mínimos de movimiento en ventana
STUCK_POSE_WINDOW_TICKS  = 300      # ventana de observación (~10s)
STUCK_MAX_ATTEMPTS       = 6        # intentos por WP antes de skip permanente
STUCK_BYPASS_DIST_MM     = 2000.0   # saltar WP si siguiente está a esta dist máxima

# ═══════════════════════════════════════════════════════
# CATTLE TRACKING (vacas como objetos con velocidad)
# ═══════════════════════════════════════════════════════
CATTLE_CLUSTER_RADIUS_MM = 100.0   # hits dentro de este radio → mismo cluster
CATTLE_MIN_HITS          = 3       # mínimo de hits dinámicos para formar track
CATTLE_MATCH_RADIUS_MM   = 300.0   # distancia máx para asociar entre frames
CATTLE_TRACK_TTL_TICKS   = 45      # ticks sin match → drop track (~1.5s)
CATTLE_VEL_EMA_ALPHA     = 0.4     # suavizado de velocidad
CATTLE_PREDICT_HORIZON_S = 1.5     # horizonte de predicción (segundos)
CATTLE_AVOID_RADIUS_MM   = 350.0   # expand buffer para evasión
CATTLE_MAX_SPEED_MM_S    = 1500.0  # velocidad máxima plausible (filtra ruido)

# ═══════════════════════════════════════════════════════
# ZONAS NO-GO (polígonos prohibidos dibujados por el usuario)
# ═══════════════════════════════════════════════════════
NOGO_MIN_AREA_MM2        = 90000.0   # área mínima para registrar (≈30cm×30cm)

# ═══════════════════════════════════════════════════════
# RECOVERY / WATCHDOGS DE ESTADO
# ═══════════════════════════════════════════════════════
STATE_BACKUP_ROTATIONS      = 3       # copias rotativas de state.json
MAP_CORRUPT_COV_THRESHOLD   = 95.0    # coverage sin progreso ⇒ rescan sugerido
MAP_CORRUPT_STUCK_TICKS     = 300     # ticks quieto con coverage alto

# ═══════════════════════════════════════════════════════
# CATTLE EVASION (desvío lateral con waypoint temporal)
# ═══════════════════════════════════════════════════════
CATTLE_EVADE_LATERAL_MM    = 800.0   # desplazamiento lateral del bypass WP
CATTLE_EVADE_COOLDOWN_TICKS = 60      # ticks mínimos entre detonaciones
CATTLE_EVADE_CLEAR_DIST_MM = 400.0   # distancia al bypass WP para limpiarlo
CATTLE_EVASION_ENABLED     = False   # DESHABILITADO: centrarse en ICP primero

# ═══════════════════════════════════════════════════════
# MAPA CORRUPTO (disparar F1 auto)
# ═══════════════════════════════════════════════════════
MAP_CORRUPT_COV_MIN_PCT    = 95.0    # coverage > este valor
MAP_CORRUPT_NO_PROGRESS_TICKS = 5     # ticks consecutivos sin pose >20mm
MAP_CORRUPT_POSE_THR_MM    = 20.0    # umbral de "pose cambió"

# Color del waypoint de retorno a casa
COLOR_HOME_WP = (255, 100, 220)

# ==========================================
# ALERTA DINÁMICA
# ==========================================
RADIO_ALERTA_DINAMICA_MM = 400.0
UMBRAL_PUNTOS_DINAMICOS = 8

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
# CÁMARA FRONTAL — CAPTURA Y PROCESAMIENTO
# ==========================================

# Intentar importar OpenCV; si no está disponible operar en modo fallback.
try:
    import cv2 as _cv2
    _CV2_AVAILABLE = True
except ImportError:
    _cv2 = None
    _CV2_AVAILABLE = False
    print("[Camera] OpenCV no encontrado — operando en modo fallback (sin cámara).")


class CameraCapture:
    """
    Wrapper robusto para cámara OpenCV.
    - Apertura segura con reintento.
    - Lectura del último frame disponible.
    - Fallback explícito si la cámara no está disponible.
    - Compatible con Python 3.10+.

    Instalar dependencia:
        pip install opencv-python
    """

    def __init__(self, index=CAM_INDEX,
                 width=CAM_WIDTH_PX, height=CAM_HEIGHT_PX, fps=CAM_FPS):
        self.index   = index
        self.width   = width
        self.height  = height
        self.fps     = fps
        self._cap    = None
        self._ok     = False
        self._last_frame = None   # último frame BGR capturado
        self._lock   = threading.Lock()
        self._thread = None
        self._running= False

    def connect(self):
        """Abre la cámara y arranca el hilo de captura continua."""
        if not _CV2_AVAILABLE:
            print("[Camera] OpenCV no disponible — modo fallback.")
            return False
        try:
            cap = _cv2.VideoCapture(self.index)
            if not cap.isOpened():
                print(f"[Camera] No se pudo abrir cámara índice {self.index}.")
                return False
            cap.set(_cv2.CAP_PROP_FRAME_WIDTH,  self.width)
            cap.set(_cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(_cv2.CAP_PROP_FPS,          self.fps)
            self._cap     = cap
            self._ok      = True
            self._running = True
            self._thread  = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            print(f"[Camera] Cámara {self.index} abierta ({self.width}×{self.height}).")
            return True
        except Exception as e:
            print(f"[Camera] Error al abrir: {e}")
            return False

    def _read_loop(self):
        """Hilo daemon: lee frames continuamente para no bloquear el main loop."""
        while self._running and self._cap and self._cap.isOpened():
            ret, frame = self._cap.read()
            if ret:
                with self._lock:
                    self._last_frame = frame
            else:
                self._ok = False
                break
        self._ok = False

    def get_frame(self):
        """Retorna el último frame BGR, o None si no hay cámara."""
        with self._lock:
            return self._last_frame.copy() if self._last_frame is not None else None

    def is_ok(self):
        return self._ok

    def close(self):
        self._running = False
        if self._cap:
            try:
                self._cap.release()
            except Exception:
                pass
        self._ok = False


# ── Funciones de análisis de imagen ──────────────────────────────

def analyze_frame_dirt(frame,
                        hsv_s_max=CAM_HSV_S_MAX,
                        hsv_v_min=CAM_HSV_V_MIN):
    """
    Analiza un frame BGR usando espacio HSV.

    LÓGICA INVERTIDA (corrales lecheros con piso oscuro):
      "Sucio" : S < hsv_s_max  Y  V > hsv_v_min  (pixel blanco/claro =
                residuos de leche, espuma, cal, deposiciones claras)
      "Limpio": cualquier otra condición (piso oscuro, variado, con
                texturas y colores propios del concreto/goma)

    Retorna (clean_ratio, dirt_ratio) — 0.0–1.0 cada uno.
      clean_ratio = fracción del frame que se ve "limpia" (no blanca)
      dirt_ratio  = fracción del frame que se ve "sucia" (blanca)
    """
    if frame is None or not _CV2_AVAILABLE:
        return 1.0, 0.0   # fallback: asumir limpio

    hsv      = _cv2.cvtColor(frame, _cv2.COLOR_BGR2HSV)
    s_ch     = hsv[:, :, 1].astype(np.float32)
    v_ch     = hsv[:, :, 2].astype(np.float32)

    # Ahora el "white_mask" identifica SUCIEDAD (residuos claros)
    dirt_mask  = (s_ch < hsv_s_max) & (v_ch > hsv_v_min)
    total_px   = dirt_mask.size
    dirt_px    = int(np.sum(dirt_mask))
    dirt_ratio = dirt_px / total_px if total_px > 0 else 0.0
    clean_ratio = 1.0 - dirt_ratio
    return clean_ratio, dirt_ratio


def classify_tile(clean_ratio,
                  clean_thr=CAM_CLEAN_RATIO,
                  medium_thr=CAM_MEDIUM_RATIO):
    """
    Clasifica el tile observado según clean_ratio (fracción no-blanca).

    clean_ratio alto  → tile LIMPIO (poco blanco visible)
    clean_ratio medio → tile MEDIO
    clean_ratio bajo  → tile SUCIO (mucho blanco = residuos)

    Retorna una de las etiquetas: CAM_LABEL_CLEAN / MEDIUM / DIRTY.
    """
    if clean_ratio >= clean_thr:
        return CAM_LABEL_CLEAN
    elif clean_ratio >= medium_thr:
        return CAM_LABEL_MEDIUM
    else:
        return CAM_LABEL_DIRTY


def dirt_to_aux_pwm(dirt_ratio, comp_on=False, comp_pwm=0):
    """
    Mapeo continuo de suciedad a potencia auxiliar.
    Si el compresor ya estaba activo, se toma el máximo entre ambos.

    dirt_ratio : 0.0 (limpio) → 1.0 (muy sucio)
    Retorna    : int 0–255
    """
    # Interpolación lineal entre CAM_PWM_CLEAN y CAM_PWM_DIRTY
    pwm = int(CAM_PWM_CLEAN + (CAM_PWM_DIRTY - CAM_PWM_CLEAN) * dirt_ratio)
    pwm = max(0, min(255, pwm))
    if comp_on:
        pwm = max(pwm, comp_pwm)
    return pwm


# ── Convención angular del robot ─────────────────────────────────────
# En este proyecto el heading 0° apunta hacia ARRIBA del mapa (+Y).
# Por eso el vector forward usa sin para X y cos para Y.
def heading_forward_lateral(yaw_deg):
    yaw_rad = math.radians(yaw_deg)
    sin_y = math.sin(yaw_rad)
    cos_y = math.cos(yaw_rad)
    fwd = (sin_y, cos_y)
    lat = (-cos_y, sin_y)
    return fwd, lat

def heading_from_vector_deg(dx, dy):
    return math.degrees(math.atan2(dx, dy))

# ── Funciones de ROI en mapa ──────────────────────────────────────

def compute_roi_corners(lidar_x_mm, lidar_y_mm, yaw_deg,
                         front_offset_mm = DIST_LIDAR_FRENTE_MM + CAM_ROI_OFFSET_MM,
                         largo_mm        = CAM_ROI_LARGO_MM,
                         ancho_mm        = CAM_ROI_ANCHO_MM):
    """
    Calcula las 4 esquinas del rectángulo ROI en coordenadas globales (mm).
    El rectángulo está centrado lateralmente respecto al robot y
    proyectado hacia adelante.

    Retorna lista de 4 tuplas (x_mm, y_mm) en orden: TL, TR, BR, BL.
    """
    # Vector unitario adelante y lateral usando convención 0° = arriba
    fwd, lat = heading_forward_lateral(yaw_deg)

    # Centro frontal del ROI (borde delantero)
    cx = lidar_x_mm + fwd[0] * front_offset_mm
    cy = lidar_y_mm + fwd[1] * front_offset_mm

    # Offset de profundidad (hacia atrás desde el borde frontal)
    half_l = largo_mm / 2.0
    half_w = ancho_mm / 2.0

    # 4 esquinas
    corners = [
        (cx + fwd[0]*half_l + lat[0]*half_w,
         cy + fwd[1]*half_l + lat[1]*half_w),   # TL
        (cx + fwd[0]*half_l - lat[0]*half_w,
         cy + fwd[1]*half_l - lat[1]*half_w),   # TR
        (cx - fwd[0]*half_l - lat[0]*half_w,
         cy - fwd[1]*half_l - lat[1]*half_w),   # BR
        (cx - fwd[0]*half_l + lat[0]*half_w,
         cy - fwd[1]*half_l + lat[1]*half_w),   # BL
    ]
    return corners


def compute_rear_roi_corners(lidar_x_mm, lidar_y_mm, yaw_deg,
                              rear_offset_mm = DIST_LIDAR_ATRAS_MM + CAM_REAR_ROI_OFFSET_MM,
                              largo_mm       = CAM_REAR_ROI_LARGO_MM,
                              ancho_mm       = CAM_REAR_ROI_ANCHO_MM):
    """
    Calcula las 4 esquinas del rectángulo ROI trasero en coordenadas globales.
    Apunta en dirección opuesta al heading — observa lo que ya pasó el robot.
    Retorna lista de 4 tuplas (x_mm, y_mm): TL, TR, BR, BL.
    """
    # Dirección trasera = -heading, misma convención angular del robot
    fwd, lat = heading_forward_lateral(yaw_deg + 180.0)

    cx = lidar_x_mm + fwd[0] * rear_offset_mm
    cy = lidar_y_mm + fwd[1] * rear_offset_mm

    half_l = largo_mm / 2.0
    half_w = ancho_mm / 2.0

    return [
        (cx + fwd[0]*half_l + lat[0]*half_w,
         cy + fwd[1]*half_l + lat[1]*half_w),
        (cx + fwd[0]*half_l - lat[0]*half_w,
         cy + fwd[1]*half_l - lat[1]*half_w),
        (cx - fwd[0]*half_l - lat[0]*half_w,
         cy - fwd[1]*half_l - lat[1]*half_w),
        (cx - fwd[0]*half_l + lat[0]*half_w,
         cy - fwd[1]*half_l + lat[1]*half_w),
    ]


def _point_in_polygon(px, py, poly_xy):
    """Ray-casting para test punto-en-polígono. poly_xy: lista de (x,y)."""
    n = len(poly_xy)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly_xy[i]
        xj, yj = poly_xy[j]
        if ((yi > py) != (yj > py) and
                px < (xj - xi) * (py - yi) / (yj - yi + 1e-9) + xi):
            inside = not inside
        j = i
    return inside


# ==========================================
# ==========================================
# PID DE HEADING + DETECCIÓN DE SLIP + LOG
# ==========================================

class HeadingPID:
    """
    Controlador PID para corrección de heading en el follower.
    Evita el sobreimpulso del control proporcional puro al girar.
    Se resetea al iniciar cada nueva alineación (ALIGN state).
    """
    def __init__(self, kp=PID_KP, ki=PID_KI, kd=PID_KD, max_i=PID_MAX_I):
        self.kp    = kp
        self.ki    = ki
        self.kd    = kd
        self.max_i = max_i
        self._integral   = 0.0
        self._prev_error = 0.0

    def compute(self, error_deg, dt=0.033):
        """Calcula la corrección PID dado el error de heading en grados."""
        dt = max(dt, 0.001)
        self._integral   = clamp(self._integral + error_deg * dt,
                                  -self.max_i, self.max_i)
        derivative       = (error_deg - self._prev_error) / dt
        self._prev_error = error_deg
        return (self.kp * error_deg +
                self.ki * self._integral +
                self.kd * derivative)

    def reset(self):
        self._integral   = 0.0
        self._prev_error = 0.0


def astar_path(grid_map, start_mm, goal_mm,
                clearance_mm=ASTAR_WALL_CLEARANCE,
                max_expansions=ASTAR_MAX_EXPANSIONS,
                nogo_zones=None):
    """
    Busca el camino más corto entre start_mm y goal_mm sobre el grid,
    respetando el mapa de paredes con un clearance dado.

    Usa 8-conectividad (permite diagonales) y heurística de Chebyshev.
    nogo_zones: NoGoZones opcional. Si se provee, las celdas bloqueadas
                por sus rectángulos se tratan como paredes virtuales.
    Retorna lista de tuplas (x_mm, y_mm) incluyendo start y goal,
    o [] si no hay camino.
    """
    import heapq

    cell = grid_map.cell_mm
    rows = grid_map.rows
    cols = grid_map.cols

    def mm_to_cell(x, y):
        return (int(y / cell), int(x / cell))   # (row, col)

    def cell_to_mm(rc):
        r, c = rc
        return ((c + 0.5) * cell, (r + 0.5) * cell)

    start_cell = mm_to_cell(*start_mm)
    goal_cell  = mm_to_cell(*goal_mm)

    # Si el goal está bloqueado, buscar la celda libre más cercana
    if grid_map.has_wall_within(goal_mm[0], goal_mm[1], clearance_mm):
        # Spiral search por celda libre alrededor del goal
        best = None
        for radius in range(1, 15):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if abs(dr) != radius and abs(dc) != radius:
                        continue
                    r, c = goal_cell[0] + dr, goal_cell[1] + dc
                    if not (0 <= r < rows and 0 <= c < cols):
                        continue
                    wx, wy = cell_to_mm((r, c))
                    if not grid_map.has_wall_within(wx, wy, clearance_mm):
                        best = (r, c)
                        break
                if best: break
            if best:
                goal_cell = best
                break
        if best is None:
            return []

    # Si start está bloqueado, relajar el clearance para arrancar
    start_blocked = grid_map.has_wall_within(start_mm[0], start_mm[1],
                                              clearance_mm)

    # A* con heap
    open_heap = [(0.0, start_cell)]
    came_from = {}
    g_score   = {start_cell: 0.0}
    expansions = 0

    NEIGHBORS = [
        (-1,-1, 1.4142), (-1, 0, 1.0), (-1, 1, 1.4142),
        ( 0,-1, 1.0),                  ( 0, 1, 1.0),
        ( 1,-1, 1.4142), ( 1, 0, 1.0), ( 1, 1, 1.4142),
    ]

    def heuristic(a, b):
        dr = abs(a[0] - b[0])
        dc = abs(a[1] - b[1])
        return cell * (max(dr, dc) + 0.4142 * min(dr, dc))

    while open_heap:
        if expansions >= max_expansions:
            break
        _, current = heapq.heappop(open_heap)
        expansions += 1

        if current == goal_cell:
            # Reconstruir camino
            path_cells = [current]
            while current in came_from:
                current = came_from[current]
                path_cells.append(current)
            path_cells.reverse()
            return [cell_to_mm(c) for c in path_cells]

        cr, cc = current
        for dr, dc, step_cost in NEIGHBORS:
            nr, nc = cr + dr, cc + dc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            wx, wy = cell_to_mm((nr, nc))
            # Skip if wall too close (unless this is the start cell)
            if not start_blocked or (nr, nc) != start_cell:
                if grid_map.has_wall_within(wx, wy, clearance_mm):
                    continue
            # Skip if inside no-go zone
            if nogo_zones is not None and nogo_zones.is_blocked(wx, wy):
                continue

            tentative = g_score[current] + step_cost * cell
            if tentative < g_score.get((nr, nc), float('inf')):
                came_from[(nr, nc)] = current
                g_score[(nr, nc)]   = tentative
                f = tentative + heuristic((nr, nc), goal_cell)
                heapq.heappush(open_heap, (f, (nr, nc)))

    return []   # sin camino encontrado


def build_route_with_astar(pattern_waypoints, grid_map,
                             clearance_mm=ASTAR_WALL_CLEARANCE,
                             nogo_zones=None):
    """
    Dada una lista de waypoints del patrón (matricial/espiral/bowtie),
    construye la ruta final conectando waypoints consecutivos con A*.

    Si el camino directo entre dos waypoints es seguro, se usa directamente.
    Si no, A* encuentra una desviación que respeta el mapa de paredes.

    Retorna lista de waypoints enriquecida con puntos intermedios de A*.
    """
    if not pattern_waypoints or not grid_map.has_wall_map():
        return list(pattern_waypoints)

    result = []
    prev   = None

    for wp in pattern_waypoints:
        if prev is None:
            result.append(wp)
            prev = wp
            continue

        # Camino directo: si es seguro y corto, añadir solo el destino
        direct_ok = not grid_map.path_crosses_wall(prev[0], prev[1],
                                                    wp[0], wp[1])
        # Verificar que el segmento directo no pase por zona no-go
        if direct_ok and nogo_zones is not None:
            # Muestrear el segmento cada 50mm
            dx = wp[0] - prev[0]; dy = wp[1] - prev[1]
            L = math.hypot(dx, dy)
            if L > 1:
                steps = max(1, int(L / 50))
                for s in range(1, steps):
                    t = s / steps
                    if nogo_zones.is_blocked(prev[0] + dx*t, prev[1] + dy*t):
                        direct_ok = False
                        break

        if (direct_ok
                and not grid_map.has_wall_within(wp[0], wp[1], clearance_mm)
                and (nogo_zones is None or not nogo_zones.is_blocked(wp[0], wp[1]))):
            result.append(wp)
        else:
            # A* para encontrar ruta segura
            path = astar_path(grid_map, prev, wp,
                               clearance_mm=clearance_mm,
                               nogo_zones=nogo_zones)
            if path:
                # Omitir el primer punto (ya está en result como prev)
                # y simplificar: saltar puntos colineales
                simplified = _simplify_path(path[1:])
                result.extend(simplified)
            else:
                # A* falló: saltar este waypoint
                continue
        prev = result[-1] if result else wp

    return result


def _simplify_path(path, angle_thresh=0.985):
    """
    Elimina puntos colineales de un camino (reduce waypoints redundantes).
    Conserva puntos donde el ángulo cambia significativamente.
    """
    if len(path) < 3:
        return list(path)
    result = [path[0]]
    for i in range(1, len(path) - 1):
        px, py = result[-1]
        cx, cy = path[i]
        nx, ny = path[i + 1]
        v1x, v1y = cx - px, cy - py
        v2x, v2y = nx - cx, ny - cy
        m1 = math.hypot(v1x, v1y)
        m2 = math.hypot(v2x, v2y)
        if m1 < 1.0 or m2 < 1.0:
            continue
        cos_a = (v1x*v2x + v1y*v2y) / (m1 * m2)
        if cos_a < angle_thresh:   # cambio de dirección significativo
            result.append((cx, cy))
    result.append(path[-1])
    return result


# ==========================================
# BREADCRUMBS — MIGAS DE PAN PARA RETORNO
# ==========================================

def compute_lateral_bypass(robot_x, robot_y, target_x, target_y,
                              cow_x, cow_y,
                              lateral_mm=CATTLE_EVADE_LATERAL_MM,
                              grid_map=None, nogo_zones=None):
    """
    Calcula un waypoint lateral para rodear a una vaca.

    El lado (izquierda/derecha del segmento robot→target) se elige como
    el opuesto al lado donde está la vaca — eso la deja detrás mientras
    el robot se desvía.

    Si grid_map se provee, valida que el bypass no choque con paredes/no-go;
    si choca, intenta el lado contrario.

    Retorna (bypass_x, bypass_y) o None si ambos lados son inviables.
    """
    # Vector del path
    dx = target_x - robot_x
    dy = target_y - robot_y
    L = math.hypot(dx, dy)
    if L < 1.0:
        return None

    # Vector unitario perpendicular (izquierda del path): (-dy, dx)/L
    perp_x = -dy / L
    perp_y =  dx / L

    # Determinar en qué lado está la vaca (producto cruz)
    cow_dx = cow_x - robot_x
    cow_dy = cow_y - robot_y
    cross = dx * cow_dy - dy * cow_dx   # >0: vaca a la izquierda; <0: derecha

    # Waypoint lateral en el lado OPUESTO al de la vaca
    sign = -1.0 if cross > 0 else 1.0   # si vaca izq, desviarse a derecha

    # Punto de bypass: a mitad del path + desplazamiento perpendicular
    mid_x = robot_x + 0.5 * dx
    mid_y = robot_y + 0.5 * dy

    def try_side(s):
        bx = mid_x + s * perp_x * lateral_mm
        by = mid_y + s * perp_y * lateral_mm
        # Clamp al mapa
        bx = max(0.0, min(MAPA_ANCHO_MM, bx))
        by = max(0.0, min(MAPA_ALTO_MM, by))
        # Validar contra paredes y no-go
        if grid_map is not None and grid_map.has_wall_map():
            if grid_map.has_wall_within(bx, by, MARGEN_PARED_MM):
                return None
        if nogo_zones is not None and nogo_zones.is_blocked(bx, by):
            return None
        return (bx, by)

    # Intentar lado preferido primero, luego el otro
    bp = try_side(sign)
    if bp is not None:
        return bp
    bp = try_side(-sign)
    return bp


class NoGoZones:
    """
    Lista de rectángulos prohibidos dibujados por el usuario.
    Cada zona es (x_min, y_min, x_max, y_max) en mm.
    """
    def __init__(self):
        self.zones = []
    
    def add_rect(self, x1, y1, x2, y2):
        xmin, xmax = min(x1, x2), max(x1, x2)
        ymin, ymax = min(y1, y2), max(y1, y2)
        area = (xmax - xmin) * (ymax - ymin)
        if area < NOGO_MIN_AREA_MM2:
            return False
        self.zones.append((xmin, ymin, xmax, ymax))
        return True
    
    def is_blocked(self, x_mm, y_mm):
        for (xmin, ymin, xmax, ymax) in self.zones:
            if xmin <= x_mm <= xmax and ymin <= y_mm <= ymax:
                return True
        return False
    
    def clear(self):
        self.zones.clear()
    
    def remove_last(self):
        if self.zones:
            self.zones.pop()
    
    def to_list(self):
        return [list(z) for z in self.zones]
    
    def from_list(self, data):
        self.zones = [tuple(z) for z in data if len(z) == 4]


class StateBackupManager:
    """Rotación de copias de seguridad: state.json → .bak → .bak2 → .bak3."""
    
    def __init__(self, base_path, rotations=STATE_BACKUP_ROTATIONS):
        self.base = base_path
        self.rotations = rotations
    
    def rotate(self):
        """Rota los backups antes de guardar el archivo principal."""
        for i in range(self.rotations, 0, -1):
            src = self.base + ("" if i == 1 else f".bak{i-1}")
            dst = self.base + f".bak{i}"
            if os.path.exists(src):
                try:
                    if os.path.exists(dst):
                        os.remove(dst)
                    os.rename(src, dst)
                except Exception as ex:
                    print(f"[BACKUP] Rotación falló {src}→{dst}: {ex}")
    
    def get_fallback_paths(self):
        """Retorna backups en orden: [.bak, .bak2, .bak3]."""
        return [self.base + f".bak{i}" for i in range(1, self.rotations + 1)]


class BreadcrumbTrail:
    """
    Registra la trayectoria del robot cada BREADCRUMB_SPACING_MM recorridos.
    Usado como fallback del retorno a casa cuando A* no encuentra camino directo.
    """
    def __init__(self, max_len=BREADCRUMB_MAX,
                  spacing_mm=BREADCRUMB_SPACING_MM):
        from collections import deque
        self.trail      = deque(maxlen=max_len)
        self.spacing    = spacing_mm
        self._last_x    = None
        self._last_y    = None

    def update(self, x_mm, y_mm):
        """Llamar cada tick — solo guarda si avanzó > spacing_mm."""
        if self._last_x is None:
            self.trail.append((x_mm, y_mm))
            self._last_x = x_mm
            self._last_y = y_mm
            return
        d = math.hypot(x_mm - self._last_x, y_mm - self._last_y)
        if d >= self.spacing:
            self.trail.append((x_mm, y_mm))
            self._last_x = x_mm
            self._last_y = y_mm

    def get_return_path(self):
        """Retorna los breadcrumbs en orden inverso (desde el actual al origen)."""
        return list(reversed(self.trail))

    def clear(self):
        self.trail.clear()
        self._last_x = None
        self._last_y = None


# ==========================================
# MEMORIA DE SESIÓN — PERSISTENCIA DE ESTADO
# ==========================================
STATE_FILE = "robot_session.json"

# Umbrales para detectar cambios relevantes
SAVE_POSE_DELTA_MM  = 50.0   # mm de movimiento mínimo para guardar
SAVE_YAW_DELTA_DEG  =  2.0   # grados de giro mínimos para guardar


class StateManager:
    """
    Guarda y restaura el estado completo del robot entre sesiones.
    Usa JSON con arrays numpy serializados como base64+zlib.
    Solo escribe cuando hay cambios relevantes (pose, mapa).
    """

    def __init__(self, filepath=STATE_FILE):
        self.filepath     = filepath
        self._last_x      = None
        self._last_y      = None
        self._last_yaw    = None
        self._last_walls_sum   = None
        self._last_cleaned_sum = None

    # ── Serialización de arrays numpy ────────────────────────────
    @staticmethod
    def _encode_array(arr):
        """numpy uint8 array → base64 string (con compresión zlib)."""
        raw  = arr.astype(np.uint8).tobytes()
        comp = zlib.compress(raw, level=6)
        return base64.b64encode(comp).decode("ascii")

    @staticmethod
    def _decode_array(b64str, shape):
        """base64 string → numpy uint8 array con shape dado."""
        comp = base64.b64decode(b64str.encode("ascii"))
        raw  = zlib.decompress(comp)
        return np.frombuffer(raw, dtype=np.uint8).reshape(shape)

    # ── Guardado ─────────────────────────────────────────────────
    def save(self, snap):
        """
        snap es un dict con:
          x_mm, y_mm, yaw_deg, patron_actual, clean_threshold,
          replan_count, grid_map (GridMap object)
        """
        gm = snap.get("grid_map")
        shape = (gm.rows, gm.cols) if gm is not None else (0, 0)

        doc = {
            "version":        2,
            "timestamp":      datetime.datetime.now().isoformat(timespec="seconds"),
            "clean_exit":     False,
            "x_mm":           float(snap.get("x_mm", LIDAR_X_INICIAL_MM)),
            "y_mm":           float(snap.get("y_mm", LIDAR_Y_INICIAL_MM)),
            "yaw_deg":        float(snap.get("yaw_deg", LIDAR_YAW_INICIAL_DEG)),
            "patron_actual":  snap.get("patron_actual", PATRON_NINGUNO),
            "clean_threshold":float(snap.get("clean_threshold", CLEAN_THRESHOLD_DEFAULT)),
            "replan_count":   int(snap.get("replan_count", 0)),
            "mission_home_x": float(snap.get("mission_home_x", LIDAR_X_INICIAL_MM)),
            "mission_home_y": float(snap.get("mission_home_y", LIDAR_Y_INICIAL_MM)),
            "cam_dirt_ema":   float(snap.get("cam_dirt_ema",   0.0)),
            "grid_rows":      shape[0],
            "grid_cols":      shape[1],
        }

        if gm is not None:
            doc["walls"]     = self._encode_array(gm.walls)
            doc["cleaned"]   = self._encode_array(gm.cleaned)
            doc["cleanable"] = self._encode_array(gm.cleanable)
            doc["discovered"]= self._encode_array(gm.discovered)

        try:
            with open(self.filepath, "w", encoding="utf-8") as f:
                json.dump(doc, f, indent=2)
        except Exception as e:
            print(f"[StateManager] Error al guardar: {e}")

    def mark_clean_exit(self):
        """Marca el archivo como cierre limpio para info en el modal."""
        try:
            if not os.path.exists(self.filepath):
                return
            with open(self.filepath, "r", encoding="utf-8") as f:
                doc = json.load(f)
            doc["clean_exit"] = True
            with open(self.filepath, "w", encoding="utf-8") as f:
                json.dump(doc, f, indent=2)
        except Exception:
            pass

    # ── Carga ─────────────────────────────────────────────────────
    def load(self):
        """
        Retorna dict con el estado guardado, o None si no existe
        o si el archivo está corrupto.
        """
        if not os.path.exists(self.filepath):
            return None
        try:
            with open(self.filepath, "r", encoding="utf-8") as f:
                doc = json.load(f)
            if doc.get("version", 0) < 2:
                return None   # formato antiguo incompatible
            return doc
        except Exception as e:
            print(f"[StateManager] Error al leer: {e}")
            return None

    def restore_grids(self, doc, grid_map):
        """
        Restaura walls/cleaned/cleanable/discovered en un GridMap existente.
        Solo actúa si las dimensiones coinciden.
        """
        rows = doc.get("grid_rows", 0)
        cols = doc.get("grid_cols", 0)
        if rows != grid_map.rows or cols != grid_map.cols:
            print(f"[StateManager] Dimensiones no coinciden: "
                  f"guardado {rows}×{cols} vs actual {grid_map.rows}×{grid_map.cols}")
            return False

        shape = (rows, cols)
        try:
            if "walls"     in doc: grid_map.walls      = self._decode_array(doc["walls"],     shape)
            if "cleaned"   in doc: grid_map.cleaned    = self._decode_array(doc["cleaned"],   shape)
            if "cleanable" in doc: grid_map.cleanable  = self._decode_array(doc["cleanable"], shape)
            if "discovered"in doc: grid_map.discovered = self._decode_array(doc["discovered"],shape)
            return True
        except Exception as e:
            print(f"[StateManager] Error restaurando grillas: {e}")
            return False

    def delete(self):
        """Borra el archivo de estado (al elegir Nueva sesión)."""
        try:
            if os.path.exists(self.filepath):
                os.remove(self.filepath)
        except Exception:
            pass

    # ── Detección de cambios ──────────────────────────────────────
    def is_dirty(self, x_mm, y_mm, yaw_deg, grid_map):
        """
        Retorna True si el estado actual difiere del último guardado
        lo suficiente como para justificar un nuevo save.
        """
        # Primera vez
        if self._last_x is None:
            self._update_refs(x_mm, y_mm, yaw_deg, grid_map)
            return True

        pose_moved = (abs(x_mm - self._last_x) > SAVE_POSE_DELTA_MM or
                      abs(y_mm - self._last_y) > SAVE_POSE_DELTA_MM or
                      abs(yaw_deg - self._last_yaw) > SAVE_YAW_DELTA_DEG)

        walls_sum   = int(np.sum(grid_map.walls))
        cleaned_sum = int(np.sum(grid_map.cleaned))
        map_changed = (walls_sum   != self._last_walls_sum or
                       cleaned_sum != self._last_cleaned_sum)

        if pose_moved or map_changed:
            self._update_refs(x_mm, y_mm, yaw_deg, grid_map)
            return True
        return False

    def _update_refs(self, x_mm, y_mm, yaw_deg, grid_map):
        self._last_x           = x_mm
        self._last_y           = y_mm
        self._last_yaw         = yaw_deg
        self._last_walls_sum   = int(np.sum(grid_map.walls))
        self._last_cleaned_sum = int(np.sum(grid_map.cleaned))


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
    """
    Deposita los hits del LiDAR en coordenadas globales del mapa (mm).

    El punto local (xl, yl) se interpreta como (adelante, lateral_izq) del
    sensor y se rota al mapa usando la convención de brújula:
        fwd = (sin_y, cos_y)   → con yaw=0 'adelante' apunta a +Y (arriba)
        lat = (-cos_y, sin_y)  → 'lateral izquierdo'
    Misma convención que compute_roi_corners y draw_robot (Fase 2 fix).
    """
    if len(distances_mm) == 0:
        return []

    ang_rel = np.radians(angles_deg - OFFSET_LIDAR_FISICO)
    yaw_rad = math.radians(yaw_deg)

    # Calcular puntos locales primero para poder aplicar espejo X
    xl = distances_mm * np.cos(ang_rel)
    yl = distances_mm * np.sin(ang_rel)
    if LIDAR_MIRROR_X:
        xl = -xl   # refleja en X

    # Rotar al marco global con convención de brújula (fwd=sin, cos)
    sin_y, cos_y = np.sin(yaw_rad), np.cos(yaw_rad)
    xg = lidar_x_mm + xl * sin_y + yl * (-cos_y)
    yg = lidar_y_mm + xl * cos_y + yl * sin_y

    return [(float(x), float(y)) for x, y in zip(xg, yg)]


def transform_scan_to_local(angles_deg, distances_mm):
    """Puntos en marco del sensor (antes de aplicar rotación de yaw).
    Útil para yaw ICP que necesita rotar por sí mismo."""
    if len(distances_mm) == 0:
        return []
    ang_rel = np.radians(angles_deg - OFFSET_LIDAR_FISICO)
    xl = distances_mm * np.cos(ang_rel)
    yl = distances_mm * np.sin(ang_rel)
    if LIDAR_MIRROR_X:
        xl = -xl
    return [(float(x), float(y)) for x, y in zip(xl, yl)]


def is_point_inside_robot_body_rel(xr, yr):
    return (-DIST_LIDAR_ATRAS_MM <= xr <= DIST_LIDAR_FRENTE_MM) and (-MITAD_ANCHO_MM <= yr <= MITAD_ANCHO_MM)


def detect_dynamic_obstacles(angles_deg, distances_mm):
    if len(distances_mm) == 0:
        return False, 0

    ang_rel = np.radians(angles_deg - OFFSET_LIDAR_FISICO)
    xs = distances_mm * np.cos(ang_rel)
    ys = distances_mm * np.sin(ang_rel)
    if LIDAR_MIRROR_X:
        xs = -xs

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
                # Watchdog/reconexión automática eliminados por solicitud.
                # Si la conexión cae, el usuario reinicia manualmente.

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
                        buffer = b''; break
                    if idx > 0:
                        buffer = buffer[idx:]
                    if len(buffer) < 47:
                        break

                    packet = buffer[:47]
                    buffer = buffer[47:]

                    if packet[1] != 0x2C:
                        continue

                    start_angle = struct.unpack('<H', packet[4:6])[0] / 100.0
                    end_angle   = struct.unpack('<H', packet[42:44])[0] / 100.0
                    if end_angle < start_angle:
                        end_angle += 360
                    step = (end_angle - start_angle) / 11.0

                    nuevos = []
                    for i in range(12):
                        dist = struct.unpack('<H', packet[6+(i*3):8+(i*3)])[0]
                        if dist > 0:
                            ang = start_angle + step * i
                            if ang >= 360: ang -= 360
                            nuevos.append((ang, dist))

                    if nuevos:
                        with self.lock:
                            self.buffer_puntos.extend(nuevos)

            except Exception:
                # Sin reconexión automática — solo evitar crash del hilo.
                time.sleep(0.05)
                buffer = b''

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
        self.last_command        = "CMD:0,0,0,0,0,0"

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
                time.sleep(0.05)

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

    def send_command(self, izq, der, aux_pwm,
                     bomba_enable=False, cep_enable=False,
                     relay_open=False):
        if not self.serial or not self.serial.is_open:
            return

        try:
            izq = max(min(int(izq), 255), -255)
            der = max(min(int(der), 255), -255)
            aux_pwm = max(min(int(aux_pwm), 255), 0)
            bomba = 1 if bomba_enable else 0
            cep = 1 if cep_enable else 0
            relay = 1 if relay_open else 0

            cmd = f"CMD:{izq},{der},{aux_pwm},{bomba},{cep},{relay}\n"
            self.last_command = cmd.strip()
            self.serial.write(cmd.encode("utf-8"))
        except Exception:
            pass

    def close(self):
        self.running = False
        try:
            if self.serial and self.serial.is_open:
                self.serial.write(b"CMD:0,0,0,0,0,0\n")
                self.serial.close()
        except Exception:
            pass


# ==========================================
# MAPA
# ==========================================
def _point_in_poly(px, py, corners):
    """Test ray-casting: si (px,py) está dentro del polígono corners (lista (x,y))."""
    n = len(corners)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = corners[i]
        xj, yj = corners[j]
        if ((yi > py) != (yj > py)) and \
                (px < (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi):
            inside = not inside
        j = i
    return inside


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

        # Grid de suciedad (float 0.0-1.0) + flag de visto por cámara
        self.dirt       = np.zeros((self.rows, self.cols), dtype=np.float32)
        self.cam_seen   = np.zeros((self.rows, self.cols), dtype=np.uint8)

        # Rotación de vista (grados) — solo afecta el render, no los datos
        self.view_rotation_deg = 0.0

    def reset(self):
        self.discovered.fill(0)
        self.cleaned.fill(0)
        self.dirt.fill(0.0)
        self.cam_seen.fill(0)
        # walls y cleanable NO se borran con reset general

    # ── Grid de suciedad ────────────────────────────────────
    def update_dirt_cell(self, x_mm, y_mm, measured_dirt, alpha=DIRT_EMA_ALPHA):
        """Actualiza la celda con EMA del dirt_ratio medido por cámara."""
        c = int(x_mm / self.cell_mm)
        r = int(y_mm / self.cell_mm)
        if not (0 <= r < self.rows and 0 <= c < self.cols):
            return
        if self.cam_seen[r, c]:
            self.dirt[r, c] = alpha * float(measured_dirt) + (1.0 - alpha) * self.dirt[r, c]
        else:
            self.dirt[r, c] = float(measured_dirt)
            self.cam_seen[r, c] = 1

    def update_dirt_roi(self, corners_mm, measured_dirt):
        """
        Actualiza todas las celdas dentro del polígono ROI (4 esquinas en mm)
        con el dirt_ratio medido. Usa bounding box + test de punto en polígono.
        """
        if not corners_mm or len(corners_mm) < 3:
            return
        xs = [c[0] for c in corners_mm]
        ys = [c[1] for c in corners_mm]
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        c_min = max(0, int(x_min / self.cell_mm))
        c_max = min(self.cols - 1, int(x_max / self.cell_mm))
        r_min = max(0, int(y_min / self.cell_mm))
        r_max = min(self.rows - 1, int(y_max / self.cell_mm))

        alpha = DIRT_EMA_ALPHA
        val   = float(measured_dirt)
        for r in range(r_min, r_max + 1):
            for c in range(c_min, c_max + 1):
                cx = (c + 0.5) * self.cell_mm
                cy = (r + 0.5) * self.cell_mm
                if _point_in_poly(cx, cy, corners_mm):
                    if self.cam_seen[r, c]:
                        self.dirt[r, c] = alpha * val + (1.0 - alpha) * self.dirt[r, c]
                    else:
                        self.dirt[r, c] = val
                        self.cam_seen[r, c] = 1

    def get_dirt_cell(self, x_mm, y_mm):
        """Retorna el dirt_ratio de la celda en (x,y) o 0.0 si no vista."""
        c = int(x_mm / self.cell_mm)
        r = int(y_mm / self.cell_mm)
        if not (0 <= r < self.rows and 0 <= c < self.cols):
            return 0.0
        return float(self.dirt[r, c]) if self.cam_seen[r, c] else 0.0

    def dirty_tiles_by_row(self, threshold=DIRT_THRESHOLD_CLEAN):
        """
        Retorna diccionario {row: [col1, col2, ...]} de celdas cuya suciedad
        supera threshold (es decir, MEDIO o SUCIO). Solo incluye filas que
        tienen al menos DIRT_REPLAN_MIN_TILES_PER_ROW celdas sucias.
        """
        result = {}
        if not self.cam_seen.any():
            return result
        for r in range(self.rows):
            cols_dirty = []
            for c in range(self.cols):
                if (self.cam_seen[r, c]
                        and self.dirt[r, c] >= threshold
                        and self.cleanable[r, c]):
                    cols_dirty.append(c)
            if len(cols_dirty) >= DIRT_REPLAN_MIN_TILES_PER_ROW:
                result[r] = cols_dirty
        return result

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
        """
        Convierte coord real (x_mm, y_mm) a píxeles de pantalla.
        Aplica rotación de vista alrededor del centro del mapa si
        self.view_rotation_deg != 0.
        """
        x0, y0, w, h = area_rect
        if getattr(self, 'view_rotation_deg', 0.0) != 0.0:
            # Rotar alrededor del centro del mapa real
            cx = self.width_mm * 0.5
            cy = self.height_mm * 0.5
            dx = x_mm - cx
            dy = y_mm - cy
            ang = math.radians(self.view_rotation_deg)
            cs, sn = math.cos(ang), math.sin(ang)
            x_mm = cx + dx * cs - dy * sn
            y_mm = cy + dx * sn + dy * cs
        sx = x0 + (x_mm / self.width_mm) * w
        sy = y0 + h - (y_mm / self.height_mm) * h
        return sx, sy

    def draw(self, screen, area_rect):
        x0, y0, w, h = area_rect
        cell_w = w / self.cols
        cell_h = h / self.rows
        rotated = (getattr(self, 'view_rotation_deg', 0.0) != 0.0)

        pygame.draw.rect(screen, COLOR_UNDISCOVERED, area_rect)

        for r in range(self.rows):
            for c in range(self.cols):
                # Escoger color
                color = None
                if self.walls[r, c]:
                    color = COLOR_WALL
                elif DIRT_GRID_ENABLED and self.cam_seen[r, c]:
                    d = self.dirt[r, c]
                    if d >= DIRT_THRESHOLD_MEDIUM:
                        color = COLOR_TILE_DIRTY
                    elif d >= DIRT_THRESHOLD_CLEAN:
                        color = COLOR_TILE_MEDIUM
                    else:
                        color = COLOR_TILE_CLEAN
                elif self.cleaned[r, c]:
                    color = COLOR_CLEANED
                elif self.cleanable[r, c]:
                    color = COLOR_CLEANABLE
                elif self.discovered[r, c]:
                    color = COLOR_DISCOVERED

                if color is None:
                    continue

                if rotated:
                    # Dibujar como polígono rotado (4 esquinas en world→screen)
                    x_mm = c * self.cell_mm
                    y_mm = r * self.cell_mm
                    corners_world = [
                        (x_mm, y_mm),
                        (x_mm + self.cell_mm, y_mm),
                        (x_mm + self.cell_mm, y_mm + self.cell_mm),
                        (x_mm, y_mm + self.cell_mm),
                    ]
                    pts = [tuple(map(int, self.world_to_screen(area_rect, wx, wy)))
                           for wx, wy in corners_world]
                    pygame.draw.polygon(screen, color, pts)
                else:
                    # Dibujo rápido rectangular axis-aligned
                    rx = int(x0 + c * cell_w)
                    ry = int(y0 + (self.rows - 1 - r) * cell_h)
                    cw = int(cell_w) + 1
                    ch = int(cell_h) + 1
                    pygame.draw.rect(screen, color, (rx, ry, cw, ch))

        # Líneas de grilla (solo en modo no-rotado — son ruido visual si se rotan)
        if not rotated:
            for c in range(self.cols + 1):
                gx = int(x0 + c * cell_w)
                pygame.draw.line(screen, COLOR_GRID, (gx, y0), (gx, y0 + h), 1)
            for r in range(self.rows + 1):
                gy = int(y0 + r * cell_h)
                pygame.draw.line(screen, COLOR_GRID, (x0, gy), (x0 + w, gy), 1)

        pygame.draw.rect(screen, COLOR_BORDER, area_rect, 2)

    def draw_robot(self, screen, area_rect, lidar_x_mm, lidar_y_mm, yaw_deg,
                    alert=False, uncertainty_mm=0.0,
                    speed_scale=1.0, pose_source=None,
                    breadcrumbs=None):
        """
        Dibuja el avatar del robot con información extendida:
          - Rectángulo orientado (cuerpo real del robot)
          - Halo de incertidumbre (radio = uncertainty_mm de pose)
          - Cono frontal color-codeado por speed_scale (verde/naranja/rojo)
          - Dot de sensor activo (esquina trasera del robot)
          - Migas de pan del trail (si breadcrumbs se provee)
        """
        # ── Trail de breadcrumbs (detrás de todo) ──────────────
        if breadcrumbs is not None and len(breadcrumbs.trail) > 1:
            pts = []
            for bx, by in breadcrumbs.trail:
                psx, psy = self.world_to_screen(area_rect, bx, by)
                pts.append((int(psx), int(psy)))
            if len(pts) >= 2:
                pygame.draw.lines(screen, (140, 180, 255), False, pts, 1)
                for p in pts:
                    pygame.draw.circle(screen, (140, 180, 255), p, 1)

        # ── Halo de incertidumbre de pose ──────────────────────
        if uncertainty_mm > 0.5:
            sx0, sy0 = self.world_to_screen(area_rect, lidar_x_mm, lidar_y_mm)
            rx, _    = self.world_to_screen(area_rect,
                                             lidar_x_mm + uncertainty_mm,
                                             lidar_y_mm)
            halo_r   = int(max(3, abs(rx - sx0)))
            # Halo semitransparente
            try:
                halo_surf = pygame.Surface((halo_r*2+2, halo_r*2+2),
                                            pygame.SRCALPHA)
                pygame.draw.circle(halo_surf, (255, 220, 80, 35),
                                    (halo_r+1, halo_r+1), halo_r, 0)
                pygame.draw.circle(halo_surf, (255, 220, 80, 120),
                                    (halo_r+1, halo_r+1), halo_r, 1)
                screen.blit(halo_surf,
                             (int(sx0)-halo_r-1, int(sy0)-halo_r-1))
            except Exception:
                pass

        # ── Cuerpo del robot ───────────────────────────────────
        # corners_local[i] = (coord_adelante, coord_lateral)
        #   adelante > 0  → hacia el frente del robot
        #   lateral > 0   → hacia la izquierda del robot
        corners_local = [
            (+DIST_LIDAR_FRENTE_MM, +MITAD_ANCHO_MM),
            (+DIST_LIDAR_FRENTE_MM, -MITAD_ANCHO_MM),
            (-DIST_LIDAR_ATRAS_MM,  -MITAD_ANCHO_MM),
            (-DIST_LIDAR_ATRAS_MM,  +MITAD_ANCHO_MM),
        ]
        # FIX Fase 2: usar la misma convención que heading_forward_lateral
        # y compute_roi_corners para que chasis y ROIs roten coherentemente.
        #   fwd = (sin, cos)     → con yaw=0, "adelante" apunta a +Y (arriba)
        #   lat = (-cos, sin)    → lateral izquierdo
        # Antes se usaba (cos, sin) para adelante, que iba 90° desfasado
        # respecto a los ROI y al cono frontal — ese era el bug que causaba
        # que los ROI se quedaran "en otro ángulo" cuando el robot rotaba.
        yaw_rad = math.radians(yaw_deg)
        sin_y = math.sin(yaw_rad)
        cos_y = math.cos(yaw_rad)
        fwd = (sin_y, cos_y)
        lat = (-cos_y, sin_y)

        pts_screen = []
        for a_fwd, a_lat in corners_local:
            xw = lidar_x_mm + a_fwd * fwd[0] + a_lat * lat[0]
            yw = lidar_y_mm + a_fwd * fwd[1] + a_lat * lat[1]
            sx, sy = self.world_to_screen(area_rect, xw, yw)
            pts_screen.append((int(sx), int(sy)))

        if len(pts_screen) >= 3:
            pygame.draw.polygon(screen, COLOR_ROBOT_FILL, pts_screen, 0)
            pygame.draw.polygon(screen, COLOR_ROBOT,      pts_screen, 2)

        # ── Centro LiDAR + flecha de heading ───────────────────
        sx, sy = self.world_to_screen(area_rect, lidar_x_mm, lidar_y_mm)
        pygame.draw.circle(screen, COLOR_LIDAR_CENTER, (int(sx), int(sy)), 5)
        # Flecha en la dirección de avance del robot (usa fwd de arriba)
        hx = lidar_x_mm + 260.0 * fwd[0]
        hy = lidar_y_mm + 260.0 * fwd[1]
        hsx, hsy = self.world_to_screen(area_rect, hx, hy)
        # Color del heading arrow según speed_scale
        if   speed_scale >= 0.95: head_col = (120, 255, 140)  # verde
        elif speed_scale >= 0.5:  head_col = (255, 180,  60)  # naranja
        elif speed_scale > 0.01:  head_col = (255, 110,  50)  # naranja oscuro
        else:                      head_col = (255,  70,  70)  # rojo (stop)
        pygame.draw.line(screen, head_col,
                          (int(sx), int(sy)), (int(hsx), int(hsy)), 3)

        # ── Alert circle centrado en el cuerpo ─────────────────
        body_offset = (DIST_LIDAR_FRENTE_MM - DIST_LIDAR_ATRAS_MM) / 2.0
        body_cx = lidar_x_mm + body_offset * fwd[0]
        body_cy = lidar_y_mm + body_offset * fwd[1]
        bsx, bsy = self.world_to_screen(area_rect, body_cx, body_cy)
        rx, _ = self.world_to_screen(area_rect,
                                      body_cx + RADIO_ALERTA_DINAMICA_MM,
                                      body_cy)
        radius_px = int(abs(rx - bsx))
        pygame.draw.circle(screen, COLOR_ALERT if alert else (135, 135, 150),
                           (int(bsx), int(bsy)), radius_px, 1)

        # ── Dot de sensor activo (esquina trasera izquierda) ────
        if pose_source is not None:
            src_colors = {
                "LIDAR+MATCH": (120, 255, 140),
                "LIDAR+PWM":   (255, 200,  80),
                "MATCH+GYRO":  (255, 200,  80),
                "GYRO":        (255, 150,  80),
                "NO SENSORS":  (180, 180, 180),
            }
            src_col = src_colors.get(pose_source, (180, 180, 180))
            # Esquina trasera izquierda del robot = -fwd·atrás + lat·medioAncho
            rear_x = lidar_x_mm + (-DIST_LIDAR_ATRAS_MM) * fwd[0] + MITAD_ANCHO_MM * lat[0]
            rear_y = lidar_y_mm + (-DIST_LIDAR_ATRAS_MM) * fwd[1] + MITAD_ANCHO_MM * lat[1]
            rsx, rsy = self.world_to_screen(area_rect, rear_x, rear_y)
            pygame.draw.circle(screen, src_col, (int(rsx), int(rsy)), 4)
            pygame.draw.circle(screen, (20, 20, 30), (int(rsx), int(rsy)), 4, 1)

    def draw_lidar_hits(self, screen, area_rect, points_xy_mm):
        for x_mm, y_mm in points_xy_mm:
            if 0 <= x_mm <= self.width_mm and 0 <= y_mm <= self.height_mm:
                sx, sy = self.world_to_screen(area_rect, x_mm, y_mm)
                pygame.draw.circle(screen, COLOR_LIDAR, (int(sx), int(sy)), 2)

    def has_wall_within(self, x_mm, y_mm, radius_mm):
        """
        Retorna True si hay alguna celda de pared dentro de radius_mm
        del punto (x_mm, y_mm). Usa una búsqueda en ventana numpy — O(1)
        en celdas ≈ rápido incluso para radios grandes.
        """
        c = int(x_mm / self.cell_mm)
        r = int(y_mm / self.cell_mm)
        cell_r = int(math.ceil(radius_mm / self.cell_mm))

        r_min = max(0, r - cell_r)
        r_max = min(self.rows - 1, r + cell_r)
        c_min = max(0, c - cell_r)
        c_max = min(self.cols - 1, c + cell_r)

        if r_min > r_max or c_min > c_max:
            return False
        return bool(np.any(self.walls[r_min:r_max + 1, c_min:c_max + 1]))

    def is_cleanable_cell(self, x_mm, y_mm):
        """Retorna True si la celda en (x_mm, y_mm) está marcada como cleanable."""
        c = int(x_mm / self.cell_mm)
        r = int(y_mm / self.cell_mm)
        if 0 <= r < self.rows and 0 <= c < self.cols:
            return bool(self.cleanable[r, c])
        return False

    def unmark_cleaned_in_roi(self, corners_mm):
        """
        Desmarca celdas 'cleaned' cuyo centro cae dentro del polígono del ROI.
        Usado por la cámara trasera cuando detecta que la zona no quedó limpia.
        Retorna el número de celdas desmarcadas.
        """
        if len(corners_mm) < 3:
            return 0
        count = 0
        for r in range(self.rows):
            for c in range(self.cols):
                if not self.cleaned[r, c]:
                    continue
                cx_mm = (c + 0.5) * self.cell_mm
                cy_mm = (r + 0.5) * self.cell_mm
                if _point_in_polygon(cx_mm, cy_mm, corners_mm):
                    self.cleaned[r, c] = 0
                    count += 1
        return count

    def draw_reference_square(self, screen, area_rect,
                               center_x_mm, center_y_mm,
                               side_mm=REFERENCE_SQUARE_SIDE_MM):
        """
        Dibuja un cuadrado fijo de referencia centrado en (center_x_mm,
        center_y_mm). No rota con el robot — representa las paredes reales
        de un "corral de prueba" de lado `side_mm`.

        Uso de diagnóstico:
          - La nube LiDAR debería calzar sobre los bordes de este cuadrado
            si la pose está bien calibrada.
          - Si la nube queda rotada respecto al cuadrado → ajusta con
            tecla G (debug) o con Q/E (saved_yaw_offset).
          - Si la nube queda trasladada → problema de pose XY
            (la corrección por matching de Fase 3 lo arreglará).
        """
        half = side_mm / 2.0
        x_min = center_x_mm - half
        x_max = center_x_mm + half
        y_min = center_y_mm - half
        y_max = center_y_mm + half

        # 4 esquinas en orden TL, TR, BR, BL
        corners_world = [
            (x_min, y_max),
            (x_max, y_max),
            (x_max, y_min),
            (x_min, y_min),
        ]
        pts = []
        for xw, yw in corners_world:
            sx, sy = self.world_to_screen(area_rect, xw, yw)
            pts.append((int(sx), int(sy)))

        if len(pts) < 4:
            return

        # Relleno semitransparente
        try:
            fill_surf = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
            pygame.draw.polygon(fill_surf, COLOR_REF_SQUARE_FILL, pts)
            screen.blit(fill_surf, (0, 0))
        except Exception:
            pass

        # Borde grueso
        pygame.draw.polygon(screen, COLOR_REF_SQUARE, pts, 3)

        # Cruz en el centro (posición inicial del robot)
        ccx, ccy = self.world_to_screen(area_rect, center_x_mm, center_y_mm)
        cs_px = 10
        pygame.draw.line(screen, COLOR_REF_SQUARE,
                          (int(ccx-cs_px), int(ccy)),
                          (int(ccx+cs_px), int(ccy)), 2)
        pygame.draw.line(screen, COLOR_REF_SQUARE,
                          (int(ccx), int(ccy-cs_px)),
                          (int(ccx), int(ccy+cs_px)), 2)

        # Etiqueta en esquina superior izquierda
        try:
            lf = pygame.font.SysFont("Consolas", 11, bold=True)
            side_m = side_mm / 1000.0
            ls = lf.render(f"REF {side_m:.1f}×{side_m:.1f}m",
                            True, COLOR_REF_SQUARE)
            screen.blit(ls, (pts[0][0] + 4, pts[0][1] + 4))
        except Exception:
            pass

    def draw_camera_roi(self, screen, area_rect, corners_mm, tile_label="",
                        border_color=None, fill_rgba=(0, 200, 80, 45)):
        """
        Dibuja el rectángulo ROI de la cámara rotado con el robot.
        corners_mm : lista de 4 tuplas (x_mm, y_mm) — salida de compute_roi_corners()
        tile_label : etiqueta de suciedad para mostrar dentro del ROI
        """
        pts = []
        for x_mm, y_mm in corners_mm:
            sx, sy = self.world_to_screen(area_rect, x_mm, y_mm)
            pts.append((int(sx), int(sy)))

        if len(pts) < 3:
            return

        b_color = border_color if border_color else COLOR_ROI_BORDER

        try:
            roi_surf = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
            pygame.draw.polygon(roi_surf, fill_rgba, pts)
            screen.blit(roi_surf, (0, 0))
        except Exception:
            pass

        pygame.draw.polygon(screen, b_color, pts, 2)

        if tile_label:
            cx = sum(p[0] for p in pts) // 4
            cy = sum(p[1] for p in pts) // 4
            lbl_col = {
                CAM_LABEL_CLEAN:  (100, 255, 140),
                CAM_LABEL_MEDIUM: (255, 210,  80),
                CAM_LABEL_DIRTY:  (255,  80,  80),
            }.get(tile_label, (200, 200, 200))
            try:
                lf = pygame.font.SysFont("Consolas", 11)
                ls = lf.render(tile_label, True, lbl_col)
                screen.blit(ls, (cx - ls.get_rect().width // 2, cy - 7))
            except Exception:
                pass

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

        FIX: ahora usa la misma convención de brújula que heading_forward_lateral
        y compute_roi_corners:
            fwd_mundo = (sin(yaw), cos(yaw))  — 0° = +Y (norte / arriba)
        En pantalla el eje Y está invertido (world_to_screen hace y0+h-y_mm),
        por eso el componente Y del fwd se niega para dibujar.
        Antes usaba (cos, sin) pero además 'sin' sin negar, lo cual hacía que
        el cono rotara en sentido opuesto al robot.
        """
        sx, sy = self.world_to_screen(area_rect, robot_x, robot_y)

        # Radio del cono en píxeles
        rx, _ = self.world_to_screen(area_rect,
                                     robot_x + FRONT_MAX_DIST_MM, robot_y)
        radius_px = max(4, int(abs(rx - sx)))

        color = COLOR_ALERT if blocked else COLOR_FRONT_CONE

        yaw_rad  = math.radians(yaw_deg)
        half_rad = math.radians(FRONT_HALF_ANGLE_DEG)

        # En convención brújula para pantalla:
        #   fwd_pantalla(a) = (sin(a), -cos(a))
        # donde `a` es el ángulo medido desde +Y en sentido horario (como yaw).
        def fwd_screen(a_rad):
            return (math.sin(a_rad), -math.cos(a_rad))

        steps = 18
        prev_pt = None
        for i in range(steps + 1):
            a = yaw_rad - half_rad + (2 * half_rad * i / steps)
            fx, fy = fwd_screen(a)
            px = int(sx + radius_px * fx)
            py = int(sy + radius_px * fy)
            if prev_pt:
                pygame.draw.line(screen, color, prev_pt, (px, py), 1)
            prev_pt = (px, py)

        # Líneas laterales del cono
        for sign in (+1, -1):
            a  = yaw_rad + sign * half_rad
            fx, fy = fwd_screen(a)
            ex = int(sx + radius_px * fx)
            ey = int(sy + radius_px * fy)
            pygame.draw.line(screen, color, (int(sx), int(sy)), (ex, ey), 1)

    def screen_to_world(self, area_rect, sx, sy):
        """Convierte coordenadas de pantalla a coordenadas globales en mm.
        Deshace la rotación de vista si está activa."""
        x0, y0, w, h = area_rect
        x_mm = (sx - x0) / w * self.width_mm
        y_mm = (1.0 - (sy - y0) / h) * self.height_mm
        if getattr(self, 'view_rotation_deg', 0.0) != 0.0:
            cx = self.width_mm * 0.5
            cy = self.height_mm * 0.5
            dx = x_mm - cx
            dy = y_mm - cy
            ang = math.radians(-self.view_rotation_deg)  # inversa
            cs, sn = math.cos(ang), math.sin(ang)
            x_mm = cx + dx * cs - dy * sn
            y_mm = cy + dx * sn + dy * cs
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
        if fsm_state == FSM_BLOCKED_ROTATE:
            active_color = COLOR_WP_BLOCKED
        elif fsm_state in (FSM_ROTATE, FSM_ROTATE_SETTLE):
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
def do_replan(grid_map, paso_mm=None, nogo_zones=None):
    """
    Ejecuta el replanning: genera nueva ruta sobre áreas sin limpiar.
    paso_mm: espaciado entre filas (None = usar PASO_LIMPIEZA_MM por defecto).
    Retorna (waypoints, patron_elegido, replan_succeeded).
    """
    wps, pat = replan_cleaning(grid_map, paso_mm=paso_mm, nogo_zones=nogo_zones)
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


_PANEL_SECTION_RECTS = {}   # section_name -> pygame.Rect (para click)


def draw_panel(screen, font, small, state, expanded_sections=None):
    """
    Panel lateral colapsable.
    expanded_sections : set de nombres de secciones abiertas
                        {"POSE", "RUTA", "CAMARA", "SISTEMA"}
    Actualiza _PANEL_SECTION_RECTS con los rects de cada header (para clicks).
    """
    global _PANEL_SECTION_RECTS
    _PANEL_SECTION_RECTS = {}

    if expanded_sections is None:
        expanded_sections = set()

    x0 = MAP_W
    pygame.draw.rect(screen, COLOR_PANEL, (x0, 0, PANEL_W, ALTO_VENTANA))
    pygame.draw.line(screen, (50, 60, 80), (x0, 0), (x0, ALTO_VENTANA), 2)

    cx = x0 + 10
    cw = PANEL_W - 20
    y  = 8

    # ── Título ──────────────────────────────────────────
    tf = pygame.font.SysFont("Arial", 17, bold=True)
    screen.blit(tf.render("ROBOT CLEANER PRO", True, (255, 215, 125)), (x0+14, y))
    y += 26

    # ── Status pills row (siempre visible) ──────────────
    modo      = state.get("modo", MODE_MANUAL)
    fsm_raw   = state.get("fsm_state", FSM_IDLE)
    spd       = state.get("speed_scale", 1.0)
    coverage  = state.get("clean_coverage", 0.0)
    threshold = state.get("clean_threshold", CLEAN_THRESHOLD_DEFAULT)
    ret_home  = state.get("is_returning_home", False)

    # Determinar display_fsm
    if fsm_raw == FSM_ADVANCE and spd < 0.99:
        display_fsm = FSM_SLOWDOWN_OBS
    elif ret_home:
        display_fsm = FSM_RETURN_HOME
    else:
        display_fsm = fsm_raw

    FSM_COLORS = {
        FSM_IDLE:           COLOR_SUBTEXT,
        FSM_ROTATE:         (255, 230,  80),   # amarillo — girando
        FSM_ROTATE_SETTLE:  (200, 180, 100),   # amarillo apagado — pausa match
        FSM_ADVANCE:        ( 80, 220, 255),   # cian — avanzando
        FSM_WP_REACHED:     (120, 255, 140),
        FSM_BLOCKED_ROTATE: (255, 120,  40),   # naranja — escape 90°
        FSM_ROUTE_DONE:     (120, 255, 140),
        FSM_SLOWDOWN_OBS:   (255, 160,  40),
        FSM_RETURN_HOME:    (255, 100, 220),
    }

    if   modo == MODE_SCAN: mc, mb = COLOR_MODE_SCAN,   (30,45,70)
    elif modo == MODE_RUTA: mc, mb = COLOR_MODE_RUTA,   (20,35,65)
    else:                   mc, mb = COLOR_MODE_MANUAL, (22,40,28)

    # Mode pill
    ms = small.render(f" {modo} ", True, mc)
    mr = pygame.Rect(cx, y, ms.get_rect().width+10, 20)
    pygame.draw.rect(screen, mb, mr, border_radius=4)
    screen.blit(ms, (cx+5, y+2))
    px = cx + mr.width + 5

    # FSM pill
    fc = FSM_COLORS.get(display_fsm, COLOR_SUBTEXT)
    fs = small.render(display_fsm, True, fc)
    fr = pygame.Rect(px, y, fs.get_rect().width+10, 20)
    pygame.draw.rect(screen, (22,24,38), fr, border_radius=4)
    pygame.draw.rect(screen, fc, fr, 1, border_radius=4)
    screen.blit(fs, (px+5, y+2))
    px += fr.width + 5

    # Speed pill (only when not full speed)
    if spd < 0.99:
        ss = small.render(f"{int(spd*100)}%", True, COLOR_THRESH_WARN)
        sr = pygame.Rect(px, y, ss.get_rect().width+8, 20)
        pygame.draw.rect(screen, (50,35,15), sr, border_radius=4)
        screen.blit(ss, (px+4, y+2))
    y += 26

    # Coverage mini-bar
    bar_w = cw
    pygame.draw.rect(screen, (35,38,52), (cx, y, bar_w, 8), border_radius=3)
    fw = int(bar_w * min(coverage/100.0, 1.0))
    bc = COLOR_THRESH_OK if coverage >= threshold else COLOR_THRESH_WARN
    if fw > 0:
        pygame.draw.rect(screen, bc, (cx, y, fw, 8), border_radius=3)
    tx = cx + int(bar_w * threshold/100.0)
    pygame.draw.line(screen, (255,255,255), (tx, y-1), (tx, y+9), 1)
    pygame.draw.rect(screen, COLOR_BORDER, (cx, y, bar_w, 8), 1, border_radius=3)
    cov_s = small.render(f"{coverage:.0f}% / {threshold:.0f}%", True, bc)
    screen.blit(cov_s, (cx + bar_w - cov_s.get_rect().width - 2, y + 10))
    y += 22

    # ── Collapsible sections ─────────────────────────────
    SECTION_DATA = {
        "POSE":    _panel_section_pose,
        "RUTA":    _panel_section_ruta,
        "CAMARA":  _panel_section_camara,
        "SISTEMA": _panel_section_sistema,
    }
    SECTION_KEYS = {"POSE":"7","RUTA":"8","CAMARA":"9","SISTEMA":"0"}

    for sec_name in PANEL_SECTIONS:
        expanded = sec_name in expanded_sections
        arrow    = "▴" if expanded else "▾"
        key_hint = SECTION_KEYS.get(sec_name, "")
        hdr_txt  = f"{key_hint}  {sec_name}  {arrow}"
        hs  = small.render(hdr_txt, True, COLOR_TEXT if expanded else COLOR_SUBTEXT)
        hh  = 22
        hrect = pygame.Rect(cx, y, cw, hh)
        hbg   = (36, 44, 62) if expanded else (26, 30, 44)
        pygame.draw.rect(screen, hbg, hrect, border_radius=5)
        pygame.draw.rect(screen, COLOR_BORDER if not expanded else (80,120,200),
                         hrect, 1, border_radius=5)
        screen.blit(hs, (cx+8, y+3))
        _PANEL_SECTION_RECTS[sec_name] = hrect
        y += hh + 2

        if expanded and sec_name in SECTION_DATA:
            y = SECTION_DATA[sec_name](screen, small, state, cx, cw, y)
            y += 4

    # ── Camera preview (siempre visible al fondo) ────────
    cam_surf_f = state.get("cam_front_surface", None)
    cam_surf_r = state.get("cam_rear_surface",  None)
    preview_y  = ALTO_VENTANA - CAM_PREVIEW_H - CAM_REAR_PREVIEW_H - 20

    # Front camera
    if cam_surf_f is not None:
        try:
            scaled = pygame.transform.scale(cam_surf_f,
                                            (CAM_PREVIEW_W, CAM_PREVIEW_H))
            screen.blit(scaled, (cx, preview_y))
        except Exception:
            pass
    else:
        pygame.draw.rect(screen, (20,22,34),
                         (cx, preview_y, CAM_PREVIEW_W, CAM_PREVIEW_H))
        ns = small.render("CAM FRONTAL SIN SEÑAL", True, COLOR_WARN)
        screen.blit(ns, (cx + 8, preview_y + CAM_PREVIEW_H//2 - 7))
    screen.blit(small.render("▲ Frontal", True, COLOR_SUBTEXT),
                (cx, preview_y - 16))

    # Rear camera
    rear_y = preview_y + CAM_PREVIEW_H + 2
    if cam_surf_r is not None:
        try:
            scaled_r = pygame.transform.scale(cam_surf_r,
                                              (CAM_PREVIEW_W, CAM_REAR_PREVIEW_H))
            screen.blit(scaled_r, (cx, rear_y))
        except Exception:
            pass
    else:
        pygame.draw.rect(screen, (20,22,34),
                         (cx, rear_y, CAM_PREVIEW_W, CAM_REAR_PREVIEW_H))
        rs = small.render("CAM TRASERA SIN SEÑAL", True, (140,80,160))
        screen.blit(rs, (cx + 8, rear_y + CAM_REAR_PREVIEW_H//2 - 7))
    screen.blit(small.render("▼ Trasera", True, COLOR_SUBTEXT),
                (cx, rear_y + CAM_REAR_PREVIEW_H + 1))


# ── Panel section renderers ─────────────────────────────
def _panel_section_pose(screen, small, state, cx, cw, y):
    for label, key in [("X:", "x_mm"), ("Y:", "y_mm"),
                       ("Yaw:", "yaw_deg"), ("DANG:", "last_dang")]:
        val = state.get(key, 0.0)
        fmt = f"{val:.1f}" if key != "last_dang" else f"{val:.4f}"
        suffix = "mm" if "mm" in key else ("°" if "yaw" in key else "")
        s = small.render(f"{label} {fmt}{suffix}", True, COLOR_SUBTEXT)
        screen.blit(s, (cx+10, y)); y += 17
    src = state.get("pose_source_label", "—")
    corr= state.get("pose_correction_mm", 0.0)
    k_un= state.get("pose_uncertainty_mm", 0.0)
    bc  = state.get("breadcrumb_count", 0)
    hp_len = state.get("home_path_len", 0)
    hp_idx = state.get("home_path_idx", 0)

    screen.blit(small.render(f"Fuente: {src}", True, (100,180,255)), (cx+10, y)); y+=17
    screen.blit(small.render(f"Corr ICP: {corr:.1f}mm", True, COLOR_SUBTEXT),(cx+10,y));y+=17

    # Incertidumbre de pose con color-code (amarillo alto, verde bajo)
    kc = ((255,220,80) if k_un > 20 else
          (255,180,60) if k_un > 10 else COLOR_OK)
    screen.blit(small.render(f"Pose σ: ±{k_un:.1f}mm",
                              True, kc), (cx+10,y)); y+=17

    # Breadcrumbs info
    screen.blit(small.render(f"Trail: {bc} migas",
                              True, (140, 180, 255)), (cx+10,y)); y+=17
    if hp_len > 0:
        screen.blit(small.render(f"Home: wp {hp_idx+1}/{hp_len}",
                                  True, (255, 100, 220)), (cx+10,y)); y+=17
    return y


_SAVE_HOME_BTN_RECT = None   # rect del botón "Guardar Home" (para click)
_CLEAR_NOGO_BTN_RECT = None  # rect del botón "Borrar zonas no-go"


def _panel_section_ruta(screen, small, state, cx, cw, y):
    global _SAVE_HOME_BTN_RECT
    n_wps     = state.get("n_waypoints", 0)
    wp_idx    = state.get("waypoint_idx", 0)
    skipped_n = state.get("skipped_count", 0)
    patron    = state.get("patron_actual", PATRON_NINGUNO)
    replan_n  = state.get("replan_count", 0)
    cleanable = state.get("cleanable_count", 0)
    uncleaned = state.get("uncleaned_count", 0)
    home_set  = state.get("mission_home_set", False)
    cur_x     = state.get("x_mm", 0.0)
    cur_y     = state.get("y_mm", 0.0)

    # ── Botón "Guardar Home" ───────────────────────────
    btn_h = 26
    btn_r = pygame.Rect(cx + 10, y, cw - 20, btn_h)
    pygame.draw.rect(screen, (80, 30, 110), btn_r, border_radius=5)
    pygame.draw.rect(screen, (255, 100, 220), btn_r, 1, border_radius=5)
    btn_txt = small.render("🏠 GUARDAR HOME AQUÍ", True, (255, 180, 240))
    screen.blit(btn_txt,
        (cx + 10 + (cw - 20 - btn_txt.get_rect().width)//2,
         y + (btn_h - btn_txt.get_rect().height)//2))
    _SAVE_HOME_BTN_RECT = btn_r
    y += btn_h + 4

    # Coordenadas del home actual (si set)
    if home_set:
        hx = state.get("mission_home_x", 0.0)
        hy = state.get("mission_home_y", 0.0)
        screen.blit(small.render(f"Home: ({hx:.0f}, {hy:.0f})mm",
                                  True, (255, 100, 220)), (cx+10, y)); y += 17
    else:
        screen.blit(small.render("Home: no guardado — click el botón",
                                  True, COLOR_SUBTEXT), (cx+10, y)); y += 17

    pc = COLOR_PATRON.get(patron, COLOR_SUBTEXT)
    for s, c in [
        (f"Patrón: {patron}", pc),
        (f"WP: {wp_idx+1 if n_wps else '-'}/{n_wps}  skip:{skipped_n}", COLOR_SUBTEXT),
        (f"Limpiable:{cleanable}  Sucias:{uncleaned}", COLOR_SUBTEXT),
        (f"Replans: {replan_n}", COLOR_WARN if replan_n>0 else COLOR_SUBTEXT),
    ]:
        screen.blit(small.render(s, True, c), (cx+10, y)); y += 17

    # ── Zonas no-go ─────────────────────────────────
    nogo_count  = state.get("nogo_count", 0)
    nogo_mode   = state.get("nogo_drawing", False)
    nogo_color  = (255, 120, 120) if nogo_count > 0 else COLOR_SUBTEXT
    screen.blit(small.render(
        f"No-go: {nogo_count}  {'✏ DIBUJANDO' if nogo_mode else '(N para dibujar)'}",
        True, nogo_color), (cx+10, y)); y += 17

    # Botón borrar todas las zonas (solo si hay alguna)
    if nogo_count > 0:
        global _CLEAR_NOGO_BTN_RECT
        btn_r = pygame.Rect(cx + 10, y, cw - 20, 22)
        pygame.draw.rect(screen, (80, 20, 20), btn_r, border_radius=5)
        pygame.draw.rect(screen, (255, 100, 100), btn_r, 1, border_radius=5)
        btn_txt = small.render("🗑 Borrar todas las zonas no-go",
                                 True, (255, 180, 180))
        screen.blit(btn_txt,
            (cx + 10 + (cw - 20 - btn_txt.get_rect().width)//2,
             y + (22 - btn_txt.get_rect().height)//2))
        _CLEAR_NOGO_BTN_RECT = btn_r
        y += 26
    else:
        _CLEAR_NOGO_BTN_RECT = None

    # ── Cattle count ────────────────────────────────
    cattle_n = state.get("cattle_count", 0)
    if cattle_n > 0:
        screen.blit(small.render(f"🐄 Vacas tracked: {cattle_n}",
                                  True, (255, 180, 100)), (cx+10, y)); y += 17

    return y


def _panel_section_camara(screen, small, state, cx, cw, y):
    cam_ok      = state.get("cam_ok",        False)
    cam_rear_ok = state.get("cam_rear_ok",   False)
    tile_f      = state.get("tile_label",    "—")
    tile_r      = state.get("rear_tile_label","—")
    wr          = state.get("clean_ratio",   0.0)
    dr          = state.get("dirt_ratio",    0.0)
    rdr         = state.get("rear_dirt_ratio",0.0)
    cam_pwm     = state.get("cam_aux_pwm",   0)
    cells_reset = state.get("rear_cells_reset", 0)

    tile_col = {CAM_LABEL_CLEAN:(100,255,140),
                CAM_LABEL_MEDIUM:(255,210,80),
                CAM_LABEL_DIRTY:(255,80,80)}
    for s, c in [
        (f"Frontal: {'OK' if cam_ok else 'SIN SEÑAL'}", COLOR_OK if cam_ok else COLOR_WARN),
        (f"  Tile: {tile_f}  L:{wr*100:.0f}% S:{dr*100:.0f}%",
         tile_col.get(tile_f, COLOR_SUBTEXT)),
        (f"  PWM visual: {cam_pwm}", COLOR_SUBTEXT),
        (f"Trasera: {'OK' if cam_rear_ok else 'SIN SEÑAL'}", COLOR_OK if cam_rear_ok else (140,80,160)),
        (f"  Tile: {tile_r}  S:{rdr*100:.0f}%", tile_col.get(tile_r, COLOR_SUBTEXT)),
        (f"  Celdas re-suciadas: {cells_reset}", COLOR_WARN if cells_reset>0 else COLOR_SUBTEXT),
    ]:
        screen.blit(small.render(s, True, c), (cx+10, y)); y += 17
    return y


def _panel_section_sistema(screen, small, state, cx, cw, y):
    arduino_ok = state.get("arduino_ok", False)
    lidar_ok   = state.get("lidar_ok",   False)
    rx_age     = state.get("rx_age_txt", "--")
    lidar_rx   = state.get("lidar_rx_txt","--")
    disc_pct   = state.get("disc_pct",   0.0)
    clean_pct  = state.get("clean_pct",  0.0)
    dyn_t      = state.get("dyn_count_total", 0)
    frontal_c  = state.get("frontal_count", 0)
    blocked    = state.get("frontal_blocked", False)
    match_q    = state.get("match_quality", 0.0)
    paso_mm    = state.get("adaptive_paso_mm", PASO_LIMPIEZA_MM)

    mode_label = "MODO: LiDAR + giroscopio"
    mode_color = (255, 160, 80)
    screen.blit(small.render(mode_label, True, mode_color), (cx+10, y)); y += 19

    rows = [
        (f"Arduino: {'OK' if arduino_ok else 'DESCONECTADO'}",
         COLOR_OK if arduino_ok else COLOR_WARN),
        (f"LiDAR:   {'OK' if lidar_ok else 'DESCONECTADO'}",
         COLOR_OK if lidar_ok else COLOR_WARN),
    ]


    rows.extend([
        (f"RX Ard: {rx_age}  LiDAR: {lidar_rx}", COLOR_SUBTEXT),
        (f"Scan match: {match_q:.0f}%",
         COLOR_OK if match_q>50 else COLOR_WARN),
        (f"Descubierto: {disc_pct:.1f}%  Limpio: {clean_pct:.1f}%", COLOR_SUBTEXT),
        (f"Hits dyn: {dyn_t}  Cono: {frontal_c}",
         COLOR_ALERT if blocked else COLOR_SUBTEXT),
    ])

    rows.extend([
        (f"Frontal: {'BLOQUEADO' if blocked else 'libre'}",
         COLOR_ALERT if blocked else COLOR_OK),
        (f"Paso limpieza: {paso_mm:.0f}mm",
         COLOR_WARN if paso_mm < PASO_LIMPIEZA_MM else COLOR_SUBTEXT),
    ])

    for s, c in rows:
        screen.blit(small.render(s, True, c), (cx+10, y)); y += 17
    return y


_CTRL_BTN_RECT = None


def draw_toolbar(screen, font, small, state, dropdown_open=False):
    """Barra delgada de estado + botón Controls ▾ que abre el dropdown."""
    global _CTRL_BTN_RECT
    tw = MAP_W
    th = TOOLBAR_H
    pygame.draw.rect(screen, COLOR_TOOLBAR_BG, (0, 0, tw, th))
    pygame.draw.line(screen, COLOR_TOOLBAR_BORDER, (0, th-1), (tw, th-1), 1)

    modo       = state.get("modo", MODE_MANUAL)
    fsm = state.get("display_fsm", state.get("fsm_state", FSM_IDLE))
    coverage   = state.get("clean_coverage", 0.0)
    threshold  = state.get("clean_threshold", CLEAN_THRESHOLD_DEFAULT)
    patron_now = state.get("patron_actual", PATRON_NINGUNO)
    has_walls  = state.get("has_wall_map", False)
    blocked    = state.get("frontal_blocked", False)
    clean_done = state.get("cleaning_complete", False)
    pump_override = state.get("pump_override_on", False)
    brush_override = state.get("brush_override_on", False)
    relay_open = state.get("relay_emergency_open", False)
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
        fc = {FSM_ROTATE:         (255, 220,  60),
              FSM_ROTATE_SETTLE:  (200, 180, 100),
              FSM_ADVANCE:        ( 80, 220, 255),
              FSM_BLOCKED_ROTATE: (255, 120,  40),
              FSM_IDLE:           COLOR_SUBTEXT,
              FSM_WP_REACHED:     (120, 255, 140),
              FSM_SLOWDOWN_OBS:   (255, 160,  40),
              FSM_RETURN_HOME:    (255, 100, 220)}.get(fsm, COLOR_SUBTEXT)
        cx = pill(fsm, fc, (22,24,38), cx)

    pygame.draw.line(screen, COLOR_TB_SEP, (cx,6), (cx,th-6), 1); cx += 8

    if pump_override:
        cx = pill("BOMBA:MAN", (255,220,80), (58,48,18), cx)
    if brush_override:
        cx = pill("CEP:MAN", (255,220,80), (58,48,18), cx)
    if relay_open:
        cx = pill("⚠ RELAY", (255,180,180), (90,28,28), cx)

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
    if has_walls: src, sc = "LIDAR MATCH", (80,160,255)
    else:         src, sc = "LIDAR RAW",   (140,140,150)
    cx = pill(src, sc, (22,24,38), cx)

    if blocked: cx = pill("⬛ BLOCKED", COLOR_ALERT, (60,18,18), cx)

    # Turn/Settle pill — muestra cuando matching está pausado
    if state.get("is_turning", False):
        cx = pill("↻ GIRO — ICP pausado", (255, 220, 120), (60, 45, 15), cx)
    elif state.get("turn_settle_left", 0) > 0:
        cx = pill(f"⏳ SETTLE {state['turn_settle_left']}",
                    (200, 200, 255), (30, 35, 55), cx)

    # View rotation pill (only when != 0)
    view_rot = state.get("view_rotation_deg", 0.0)
    if abs(view_rot) > 0.05:
        cx = pill(f"↻ {view_rot:+.1f}°", (255, 200, 120), (50, 35, 18), cx)

    # Pill: calidad del último scan match (visible cuando > 0)
    match_q = state.get("match_quality", 0.0)
    if match_q > 0.5:
        mq_col = ((120, 255, 140) if match_q >= 60 else
                  (255, 210,  80) if match_q >= 30 else
                  (255, 150,  80))
        cx = pill(f"⌖ match {match_q:.0f}%", mq_col, (15, 45, 30), cx)

    # Pill: pose incierta (modo RUTA se bloquea cuando se activa)
    if state.get("pose_uncertain", False):
        cx = pill("⚠ POSE INCIERTA", (255, 90, 90), (65, 20, 20), cx)

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
    """Dropdown flotante con atajos y leyenda de colores — lista completa."""
    items = [
        # ── Modos y navegación ─────────────
        ("F1",       "Scan paredes (quieto 4s)"),
        ("F2",       "Ciclar patrón (MATR→ESP→BOW)"),
        ("F3",       "Replan forzado"),
        ("F4",       "Toggle cuadrado de referencia 2.2m"),
        ("F5",       "Reset rotación vista"),
        ("Z",        "Recalibrar yaw (orientación actual → 0°)"),
        ("Tab",      "Manual / Ruta"),
        # ── Movimiento manual ──────────────
        ("W / S",    "Avanzar / retroceder"),
        ("A / D",    "Girar izq / der"),
        ("X",        "STOP total"),
        # ── Sistemas de limpieza ───────────
        ("R",        "Toggle limpieza"),
        ("O",        "Toggle compresor"),
        ("V",        "Cambiar potencia limpieza"),
        ("1 / 2",    "PWM avance ±"),
        ("3 / 4",    "PWM giro ±"),
        ("[ / ]",    "Umbral cobertura ± 5%"),
        # ── Localización ───────────────────
        ("F6",       "Scan match XY contra cuadrado (AHORA)"),
        # ── Zonas no-go ────────────────────
        ("N",        "Toggle modo dibujar no-go"),
        ("Drag L",   "Arrastre = rect no-go (modo N)"),
        ("Delete",   "Borrar última zona no-go"),
        # ── Exportar / log ─────────────────
        ("M",        "Exportar mapa PNG"),
        ("L",        "Exportar session log CSV"),
        # ── Panel ───────────────────────────
        ("7/8/9/0",  "Toggle POSE/RUTA/CAM/SIS"),
        # ── Reset global ────────────────────
        ("BkSp",     "Limpiar waypoints"),
        ("P",        "Reset runtime completo"),
        # ── Mouse ───────────────────────────
        ("L-Click",  "Añadir waypoint (modo RUTA)"),
        ("R-Click",  "Borrar último waypoint"),
        ("Panel hdr","Expand/colapsar sección"),
        ("Home btn", "Guardar pose como home"),
    ]
    col_w = 240; row_h = 19; cols = 2
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

def scan_match_full_against_square(hits_global_xy, robot_x, robot_y,
                                     square_center_x, square_center_y,
                                     side_mm):
    """
    Scan matching de POSE COMPLETA (XY + yaw) contra un cuadrado conocido.

    Algoritmo en 2 pasadas:
      1. Asocia cada hit a la pared más cercana de las 4 del cuadrado.
         Calcula dx, dy como promedio de errores perpendiculares por
         dirección (paredes verticales → dx; horizontales → dy).
      2. Para cada pared con suficientes hits asociados, ajusta una línea
         por mínimos cuadrados y mide el ángulo entre esa línea y la
         pared ideal (horizontal o vertical). Promedia los ángulos de
         las 4 paredes → delta_yaw.

    El giroscopio sigue siendo la referencia absoluta del yaw. Lo que
    devuelve esta función es la corrección que se aplica al offset
    acumulado (saved_yaw_offset), no al yaw instantáneo.

    Retorna
    -------
    dict con keys:
        'dx', 'dy'        : corrección sugerida de pose (mm)
        'dyaw'            : corrección sugerida de yaw (grados)
        'quality'         : fracción de hits emparejados (%)
        'n_pairs'         : nº total de pares válidos
        'yaw_confidence'  : fracción de paredes con yaw estimable (0..1)
    o None si no hay match confiable.
    """
    if len(hits_global_xy) == 0:
        return None

    half = side_mm / 2.0
    x_min = square_center_x - half
    x_max = square_center_x + half
    y_min = square_center_y - half
    y_max = square_center_y + half

    # Buckets: índice 0=left, 1=right, 2=bottom, 3=top
    # Guardamos (hit_x, hit_y) asociados a cada pared.
    wall_hits = ([], [], [], [])
    err_x_sum = 0.0
    err_y_sum = 0.0
    n_err_x   = 0
    n_err_y   = 0

    for hx, hy in hits_global_xy:
        d_left   = abs(hx - x_min)
        d_right  = abs(hx - x_max)
        d_bottom = abs(hy - y_min)
        d_top    = abs(hy - y_max)

        d_min = min(d_left, d_right, d_bottom, d_top)
        if d_min > REF_MATCH_MAX_DIST_MM:
            continue

        if d_min == d_left:
            err_x_sum += (x_min - hx)
            n_err_x   += 1
            wall_hits[0].append((hx, hy))
        elif d_min == d_right:
            err_x_sum += (x_max - hx)
            n_err_x   += 1
            wall_hits[1].append((hx, hy))
        elif d_min == d_bottom:
            err_y_sum += (y_min - hy)
            n_err_y   += 1
            wall_hits[2].append((hx, hy))
        else:
            err_y_sum += (y_max - hy)
            n_err_y   += 1
            wall_hits[3].append((hx, hy))

    n_total = n_err_x + n_err_y
    if n_total < REF_MATCH_MIN_PAIRS:
        return None

    # ── Corrección de XY ────────────────────────────────────────
    dx = (err_x_sum / n_err_x) if n_err_x > 0 else 0.0
    dy = (err_y_sum / n_err_y) if n_err_y > 0 else 0.0

    # Clamp XY (anti-outlier)
    MAX_CORR_MM = side_mm * 0.25
    if abs(dx) > MAX_CORR_MM:
        dx = MAX_CORR_MM if dx > 0 else -MAX_CORR_MM
    if abs(dy) > MAX_CORR_MM:
        dy = MAX_CORR_MM if dy > 0 else -MAX_CORR_MM

    # ── Corrección de YAW ───────────────────────────────────────
    # Para cada pared con ≥ REF_YAW_MIN_HITS_PER_WALL hits, ajustamos
    # una línea y medimos su desviación angular respecto a la pared
    # ideal (paredes L/R son verticales; B/T son horizontales).
    #
    # Paredes verticales ideales: x = constante. Ajustar y vs x:
    #   Si la pared está bien alineada, la pendiente dx/dy ≈ 0.
    #   Ángulo = atan2(dx, dy) ≈ 0 para la pared ideal.
    # Paredes horizontales ideales: y = constante. Ajustar x vs y:
    #   Ángulo = atan2(dy, dx) ≈ 0 para la pared ideal.
    #
    # Si la nube está rotada +θ respecto al mapa real, los hits de la
    # pared derecha (por ejemplo) aparecen rotados +θ. Midiendo ese
    # ángulo y promediando sobre las 4 paredes obtenemos la corrección.
    dyaw_samples = []
    walls_used = 0

    for wall_idx, hits in enumerate(wall_hits):
        if len(hits) < REF_YAW_MIN_HITS_PER_WALL:
            continue
        pts = np.asarray(hits, dtype=np.float64)
        xs = pts[:, 0]
        ys = pts[:, 1]

        # Centrar para el ajuste por SVD (más robusto que mínimos cuadrados
        # vertical cuando la pared es casi vertical o casi horizontal)
        cx = np.mean(xs)
        cy = np.mean(ys)
        xs_c = xs - cx
        ys_c = ys - cy

        # Vector dirección principal = autovector del mayor autovalor
        # de la matriz de covarianza 2x2
        Sxx = float(np.sum(xs_c * xs_c))
        Syy = float(np.sum(ys_c * ys_c))
        Sxy = float(np.sum(xs_c * ys_c))
        if Sxx + Syy < 1e-6:
            continue

        # Ángulo de la dirección principal (respecto al eje X del mapa)
        line_angle = 0.5 * math.atan2(2 * Sxy, Sxx - Syy)   # (-π/2, π/2]

        if wall_idx in (0, 1):
            # Pared vertical ideal: dirección principal apunta a ±Y (90° o -90°)
            # Ángulo esperado = ±π/2. Medimos desviación respecto al más cercano.
            target = math.pi / 2 if line_angle >= 0 else -math.pi / 2
        else:
            # Pared horizontal ideal: dirección principal apunta a ±X (0° o ±π)
            # Aquí line_angle está en (-π/2, π/2], así que target = 0.
            target = 0.0

        delta = line_angle - target
        # Normalizar a (-π/2, π/2]
        while delta > math.pi / 2:  delta -= math.pi
        while delta <= -math.pi / 2: delta += math.pi

        # Ponderar por número de hits en esta pared (más hits = más confianza)
        w = len(hits)
        dyaw_samples.append((delta, w))
        walls_used += 1

    # Promedio ponderado de las muestras de yaw
    if dyaw_samples:
        sum_w  = sum(w for _, w in dyaw_samples)
        sum_dw = sum(d * w for d, w in dyaw_samples)
        dyaw_rad = sum_dw / sum_w
        dyaw_deg = math.degrees(dyaw_rad)

        # Clamp (anti-outlier)
        if abs(dyaw_deg) > REF_YAW_MAX_CORRECTION_DEG:
            dyaw_deg = (REF_YAW_MAX_CORRECTION_DEG
                        if dyaw_deg > 0 else -REF_YAW_MAX_CORRECTION_DEG)

        yaw_confidence = walls_used / 4.0
    else:
        dyaw_deg = 0.0
        yaw_confidence = 0.0

    quality = 100.0 * n_total / max(1, len(hits_global_xy))

    return {
        'dx': dx,
        'dy': dy,
        'dyaw': dyaw_deg,
        'quality': quality,
        'n_pairs': n_total,
        'yaw_confidence': yaw_confidence,
    }


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

        ang_to_hit  = math.atan2(dx, dy)
        ang_diff    = abs(math.atan2(
            math.sin(ang_to_hit - yaw_rad),
            math.cos(ang_to_hit - yaw_rad)
        ))
        if ang_diff <= half_ang:
            frontal.append((xh, yh))

    return len(frontal), frontal


# ==========================================
# FASE 2 — DESACELERACIÓN Y RETROCESO
# ==========================================

def obstacle_speed_scale(dist_mm, slow_zone_mm, stop_zone_mm):
    """
    Calcula el factor de velocidad (0.0–1.0) según distancia al obstáculo.

    dist_mm     : distancia al obstáculo más cercano en el cono relevante
    slow_zone_mm: distancia a la que empieza a frenar
    stop_zone_mm: distancia crítica — velocidad 0

    Retorna 1.0 (velocidad normal) si no hay obstáculo relevante,
    valor proporcional entre 0 y 1 en zona de desaceleración,
    y 0.0 en zona crítica.
    """
    if dist_mm >= slow_zone_mm:
        return 1.0
    if dist_mm <= stop_zone_mm:
        return 0.0
    # Interpolación lineal dentro de la zona de desaceleración
    ratio = (dist_mm - stop_zone_mm) / (slow_zone_mm - stop_zone_mm)
    return float(clamp(ratio, 0.0, 1.0))


def get_nearest_hit_dist(hits, robot_x, robot_y):
    """
    Retorna la distancia en mm al hit más cercano de una lista.
    Retorna float('inf') si la lista está vacía.
    """
    if not hits:
        return float('inf')
    min_d = float('inf')
    for xh, yh in hits:
        d = math.hypot(xh - robot_x, yh - robot_y)
        if d < min_d:
            min_d = d
    return min_d


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

def get_cleaning_bbox(grid_map, margin_mm=MARGEN_PARED_MM,
                       margin_cells=None):
    """
    Calcula el bounding box interior del área limpiable.

    Si grid_map.cleanable tiene contenido, usa ese mask (es más preciso
    porque respeta la forma real del piso limpiable, no el bbox crudo
    de las paredes).

    margin_cells : si se provee, se usa como ROUTE_MARGIN_CELLS adicionales
                   de contracción desde el borde del cleanable mask.
    """
    if margin_cells is None:
        margin_cells = ROUTE_MARGIN_CELLS

    # Preferir cleanable mask si existe
    if grid_map.cleanable.any():
        idxs = np.argwhere(grid_map.cleanable)
        row_min = int(idxs[:, 0].min())
        row_max = int(idxs[:, 0].max())
        col_min = int(idxs[:, 1].min())
        col_max = int(idxs[:, 1].max())

        # Contraer por margin_cells desde el borde del cleanable
        row_min += margin_cells
        row_max -= margin_cells
        col_min += margin_cells
        col_max -= margin_cells

        if row_min > row_max or col_min > col_max:
            return None

        x_min = (col_min + 0.5) * grid_map.cell_mm
        y_min = (row_min + 0.5) * grid_map.cell_mm
        x_max = (col_max + 0.5) * grid_map.cell_mm
        y_max = (row_max + 0.5) * grid_map.cell_mm

        return (x_min, y_min, x_max, y_max)

    # Fallback: bbox por paredes
    if not grid_map.has_wall_map():
        return None

    idxs = np.argwhere(grid_map.walls)
    if len(idxs) == 0:
        return None

    row_min = int(idxs[:, 0].min())
    row_max = int(idxs[:, 0].max())
    col_min = int(idxs[:, 1].min())
    col_max = int(idxs[:, 1].max())

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

def filter_route_waypoints(wps, grid_map,
                            normal_mm=None,
                            turn_mm=None,
                            nogo_zones=None):
    """
    Filtra waypoints demasiado cercanos a paredes o fuera del área limpiable.

    normal_mm : clearance mínimo para waypoints de avance normal (default: MARGEN_PARED_MM)
    turn_mm   : clearance para waypoints de giro (default: WP_TURN_RADIUS_MM)
    nogo_zones: NoGoZones opcional. Los WPs dentro de zonas no-go se descartan.

    Un waypoint es "de giro" si:
      - Es el primero o último de la lista, O
      - El ángulo entre el segmento anterior y el siguiente supera 60°
        (el robot tendrá que girar significativamente)

    Waypoints descartados por clearance insuficiente se eliminan de la lista;
    no se reemplazan para no alterar el patrón de cobertura.
    """
    if normal_mm is None:
        normal_mm = float(MARGEN_PARED_MM)
    if turn_mm is None:
        turn_mm = float(WP_TURN_RADIUS_MM)

    if not wps or not grid_map.has_wall_map():
        return wps   # sin mapa de paredes no podemos filtrar

    n = len(wps)
    result = []

    for i, (wx, wy) in enumerate(wps):
        # ── Determinar si es waypoint de giro ──────────────────────
        is_turn = (i == 0 or i == n - 1)

        if not is_turn and 0 < i < n - 1:
            px, py = wps[i - 1]
            nx, ny = wps[i + 1]
            v1x, v1y = wx - px, wy - py
            v2x, v2y = nx - wx, ny - wy
            m1 = math.hypot(v1x, v1y)
            m2 = math.hypot(v2x, v2y)
            if m1 > 10.0 and m2 > 10.0:
                cos_a = (v1x*v2x + v1y*v2y) / (m1 * m2)
                if cos_a < 0.5:   # ángulo > ~60° → el robot tiene que girar
                    is_turn = True

        clearance = turn_mm if is_turn else normal_mm

        # ── Descartar si hay pared dentro del clearance ────────────
        if grid_map.has_wall_within(wx, wy, clearance):
            continue   # demasiado cerca de pared

        # ── Descartar si la celda no es limpiable (fuera del mapa) ─
        # (tolera pequeñas discrepancias de bbox vs grid real)
        if not grid_map.is_cleanable_cell(wx, wy):
            continue

        # ── Descartar si está en zona no-go ──
        if nogo_zones is not None and nogo_zones.is_blocked(wx, wy):
            continue

        result.append((wx, wy))

    return result


def enforce_waypoint_spacing(wps, grid_map,
                               min_x_cells=WP_MIN_SPACING_X_CELLS,
                               min_y_cells=WP_MIN_SPACING_Y_CELLS):
    """
    Elimina waypoints consecutivos demasiado cercanos.

    Un waypoint se descarta si su separación con el anterior aceptado
    es menor que min_x_cells en X y min_y_cells en Y (ambas condiciones).
    El primero y el último siempre se conservan.
    """
    if not wps:
        return []
    cell = grid_map.cell_mm
    min_dx = min_x_cells * cell
    min_dy = min_y_cells * cell

    result = [wps[0]]
    for i in range(1, len(wps) - 1):
        wx, wy   = wps[i]
        lx, ly   = result[-1]
        dx = abs(wx - lx)
        dy = abs(wy - ly)
        # Descarta si ambos son pequeños (muy cerca en ambos ejes)
        if dx < min_dx and dy < min_dy:
            continue
        result.append((wx, wy))
    # Mantener el último punto del patrón
    if len(wps) > 1 and wps[-1] != result[-1]:
        result.append(wps[-1])
    return result


def replan_cleaning_dirt_focused(grid_map,
                                   paso_mm=None,
                                   min_tiles_per_row=DIRT_REPLAN_MIN_TILES_PER_ROW):
    """
    Replan que ignora las zonas limpias y genera pasadas matriciales
    SOLO en las filas que contienen tiles sucios/medios.

    Retorna (waypoints, "DIRT-FOCUS") o ([], PATRON_NINGUNO) si no hay
    filas sucias suficientes.
    """
    paso = paso_mm if paso_mm is not None else PASO_LIMPIEZA_MM

    dirty_rows = grid_map.dirty_tiles_by_row(threshold=DIRT_THRESHOLD_CLEAN)
    # Filtrar filas con pocos tiles
    dirty_rows = {r: cols for r, cols in dirty_rows.items()
                  if len(cols) >= min_tiles_per_row}

    if not dirty_rows:
        return [], PATRON_NINGUNO

    cell = grid_map.cell_mm
    # Agrupar filas cercanas en bandas según paso_mm (para no hacer
    # pasadas consecutivas muy juntas)
    row_step = max(1, int(paso / cell))
    selected_rows = sorted(dirty_rows.keys())

    # Filtrar para que no haya dos filas consecutivas muy cercanas
    filtered = [selected_rows[0]]
    for r in selected_rows[1:]:
        if r - filtered[-1] >= row_step:
            filtered.append(r)

    wps = []
    forward = True
    for r in filtered:
        cols_dirty = sorted(dirty_rows[r])
        c_start = cols_dirty[0]
        c_end   = cols_dirty[-1]
        # Extender un poco para entrar/salir limpio
        c_start = max(0, c_start - 1)
        c_end   = min(grid_map.cols - 1, c_end + 1)
        y_mm = (r + 0.5) * cell
        x_a  = (c_start + 0.5) * cell
        x_b  = (c_end + 0.5) * cell
        if forward:
            wps.append((x_a, y_mm))
            wps.append((x_b, y_mm))
        else:
            wps.append((x_b, y_mm))
            wps.append((x_a, y_mm))
        forward = not forward

    return wps, "DIRT-FOCUS"


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


def replan_cleaning(grid_map, paso_mm=None, nogo_zones=None):
    """
    Genera una nueva ruta de limpieza enfocada en las áreas sin limpiar.
    paso_mm: espaciado adaptativo entre filas (None = PASO_LIMPIEZA_MM).
    Los waypoints generados se filtran para respetar el mapa de paredes
    real y dejar margen de giro suficiente.
    """
    paso = paso_mm if paso_mm is not None else PASO_LIMPIEZA_MM

    if not grid_map.has_cleanable_mask():
        return [], PATRON_NINGUNO

    uncleaned_count = grid_map.get_uncleaned_cell_count()
    if uncleaned_count == 0:
        return [], PATRON_NINGUNO

    # ── PRIORIDAD 1: replan enfocado en tiles sucios/medios ───────
    # Si ya pasamos por el corral y hay zonas identificadas como sucias,
    # concentrarse solo en esas filas.
    if DIRT_GRID_ENABLED:
        dirt_wps, dirt_pat = replan_cleaning_dirt_focused(
            grid_map, paso_mm=paso)
        if dirt_wps:
            # Aplicar filtros de pared y spacing
            dirt_wps = filter_route_waypoints(dirt_wps, grid_map, nogo_zones=nogo_zones)
            dirt_wps = enforce_waypoint_spacing(dirt_wps, grid_map)
            if dirt_wps and grid_map.has_wall_map():
                dirt_wps = build_route_with_astar(dirt_wps, grid_map, nogo_zones=nogo_zones)
            if dirt_wps:
                print(f"[REPLAN] DIRT-FOCUS: {len(dirt_wps)} wps en filas sucias")
                return dirt_wps, dirt_pat

    patron     = select_best_pattern(grid_map)
    dirty_bbox = grid_map.get_uncleaned_bbox()
    if dirty_bbox is None:
        return [], PATRON_NINGUNO

    if patron == PATRON_MATRICIAL:
        wps = generate_matricial(dirty_bbox, paso_mm=paso)
    elif patron == PATRON_ESPIRAL:
        wps = generate_espiral(dirty_bbox, paso_mm=paso)
    elif patron == PATRON_BOWTIE:
        wps = generate_bowtie(dirty_bbox, paso_mm=paso)
    else:
        wps = generate_matricial(dirty_bbox)

    # ── Filtrar waypoints contra el mapa real de paredes ──────────
    wps = filter_route_waypoints(wps, grid_map, nogo_zones=nogo_zones)

    if not wps:
        wps = filter_route_waypoints(
            generate_matricial(dirty_bbox, paso_mm=paso),
            grid_map,
            normal_mm=float(MARGEN_PARED_MM) * 0.75,
            turn_mm=float(WP_TURN_RADIUS_MM) * 0.75,
            nogo_zones=nogo_zones,
        )

    # ── Aplicar spacing mínimo ────────────────────────────────────
    wps = enforce_waypoint_spacing(wps, grid_map)

    # ── A*: construir el camino completo entre waypoints filtrados ──
    if wps and grid_map.has_wall_map():
        wps = build_route_with_astar(wps, grid_map, nogo_zones=nogo_zones)

    return wps, patron


def follow_route_step(robot_x, robot_y, yaw_deg,
                      waypoints, wp_idx, fsm_state,
                      obstacle_hold=False, heading_pid=None, dt=0.033,
                      pwm_align=None, pwm_advance=None,
                      rotate_state=None):
    """
    Follower: rotar en sitio con pausas para match, después avanzar recto.

    Filosofía:
      - NUNCA retrocede
      - Gira 100% hacia el WP antes de avanzar (ALIGN_FINE_DEG de tolerancia)
      - Durante giros grandes, hace "bursts" de rotación con pausas de
        settle para que el scan match pueda converger con la nueva pose

    Parámetros nuevos
    -----------------
    rotate_state : dict mutable con estado del rotate/settle:
        'burst_start_t'   : timestamp cuando empezó el burst actual
        'settle_start_t'  : timestamp cuando empezó la pausa actual
        'last_match_ok_t' : timestamp del último match exitoso (lo llena el tick)
    Si rotate_state es None, el follower actúa como el viejo follower
    (gira sin pausas) — útil para no romper legacy.

    Retorna (pl, pr, new_fsm, new_wp_idx).
    """
    if not waypoints or wp_idx >= len(waypoints):
        return 0.0, 0.0, FSM_IDLE, wp_idx

    target_x, target_y = waypoints[wp_idx]
    dx = target_x - robot_x
    dy = target_y - robot_y
    dist_mm = math.hypot(dx, dy)
    target_heading = heading_from_vector_deg(dx, dy)
    heading_error = _normalize_angle(target_heading - yaw_deg)

    align_pwm   = pwm_align   if pwm_align   is not None else PWM_FOLLOWER_ALIGN
    advance_pwm = pwm_advance if pwm_advance is not None else PWM_FOLLOWER_ADVANCE

    now_t = time.time()

    # IDLE → arrancar siempre rotando
    if fsm_state == FSM_IDLE:
        if rotate_state is not None:
            rotate_state['burst_start_t']  = now_t
            rotate_state['settle_start_t'] = None
        return 0.0, 0.0, FSM_ROTATE, wp_idx

    # ROTATE: girando en sitio hacia el WP
    if fsm_state == FSM_ROTATE:
        if obstacle_hold:
            return 0.0, 0.0, FSM_BLOCKED_ROTATE, wp_idx
        if dist_mm < WP_REACH_MM:
            return 0.0, 0.0, FSM_WP_REACHED, wp_idx

        # ¿Ya estamos bien alineados? → avanzar
        if abs(heading_error) <= ALIGN_FINE_DEG:
            return 0.0, 0.0, FSM_ADVANCE, wp_idx

        # Si tenemos rotate_state, chequear si el burst se agotó
        if rotate_state is not None:
            burst_start = rotate_state.get('burst_start_t')
            # Si nunca se inicializó (o viene de un settle previo),
            # arrancar burst ahora y girar este tick.
            if burst_start is None:
                rotate_state['burst_start_t']  = now_t
                rotate_state['settle_start_t'] = None
            elif (now_t - burst_start) * 1000.0 >= ROTATE_BURST_MS:
                # Burst cumplido → entrar en SETTLE
                rotate_state['settle_start_t'] = now_t
                rotate_state['burst_start_t']  = None
                return 0.0, 0.0, FSM_ROTATE_SETTLE, wp_idx

        # Rotar en sitio. heading_error > 0 ⇒ target a la izquierda del heading
        # actual en convención brújula ⇒ girar CCW (izquierda) ⇒ pl negativo,
        # pr positivo con signos diferenciales.
        if heading_error > 0:
            return (-align_pwm, +align_pwm, FSM_ROTATE, wp_idx)
        else:
            return (+align_pwm, -align_pwm, FSM_ROTATE, wp_idx)

    # ROTATE_SETTLE: motores a 0, esperando que el match converja
    if fsm_state == FSM_ROTATE_SETTLE:
        if obstacle_hold:
            return 0.0, 0.0, FSM_BLOCKED_ROTATE, wp_idx
        if dist_mm < WP_REACH_MM:
            return 0.0, 0.0, FSM_WP_REACHED, wp_idx

        # ¿Ya alineados tras el settle? → avanzar
        if abs(heading_error) <= ALIGN_FINE_DEG:
            return 0.0, 0.0, FSM_ADVANCE, wp_idx

        # ¿El settle se cumplió? → volver a ROTATE con burst fresco
        if rotate_state is not None:
            settle_start = rotate_state.get('settle_start_t')
            # Si el settle no se inicializó (estado raro), inicializar ahora
            if settle_start is None:
                rotate_state['settle_start_t'] = now_t
                rotate_state['burst_start_t']  = None
            elif (now_t - settle_start) * 1000.0 >= ROTATE_SETTLE_MS:
                rotate_state['burst_start_t']  = now_t
                rotate_state['settle_start_t'] = None
                return 0.0, 0.0, FSM_ROTATE, wp_idx
        else:
            # Sin rotate_state (legacy) → volver a ROTATE inmediatamente
            return 0.0, 0.0, FSM_ROTATE, wp_idx

        # Durante settle: motores quietos
        return 0.0, 0.0, FSM_ROTATE_SETTLE, wp_idx

    # ADVANCE: avanzando recto hacia el WP
    if fsm_state == FSM_ADVANCE:
        if obstacle_hold:
            return 0.0, 0.0, FSM_BLOCKED_ROTATE, wp_idx
        if dist_mm < WP_REACH_MM:
            return 0.0, 0.0, FSM_WP_REACHED, wp_idx
        # Si el error se abre mucho, volver a girar
        if abs(heading_error) > ALIGN_FINE_DEG * 2.0:
            if rotate_state is not None:
                rotate_state['burst_start_t']  = now_t
                rotate_state['settle_start_t'] = None
            return 0.0, 0.0, FSM_ROTATE, wp_idx
        return advance_pwm, advance_pwm, FSM_ADVANCE, wp_idx

    # WP alcanzado → siguiente WP
    if fsm_state == FSM_WP_REACHED:
        next_idx = wp_idx + 1
        if next_idx >= len(waypoints):
            return 0.0, 0.0, FSM_ROUTE_DONE, wp_idx
        if rotate_state is not None:
            rotate_state['burst_start_t']  = now_t
            rotate_state['settle_start_t'] = None
        return 0.0, 0.0, FSM_ROTATE, next_idx

    # BLOCKED_ROTATE: no hacemos nada aquí, lo maneja el tick externamente
    # (necesita lógica de acumulación de rotación 90° y evaluación del cono)
    if fsm_state == FSM_BLOCKED_ROTATE:
        if obstacle_hold:
            # Mantener bloqueo hasta que el tick resuelva el escape
            return 0.0, 0.0, FSM_BLOCKED_ROTATE, wp_idx
        # Cono despejado → retomar plan (ir a ROTATE hacia el WP)
        if rotate_state is not None:
            rotate_state['burst_start_t']  = now_t
            rotate_state['settle_start_t'] = None
        return 0.0, 0.0, FSM_ROTATE, wp_idx

    if fsm_state == FSM_ROUTE_DONE:
        return 0.0, 0.0, FSM_ROUTE_DONE, wp_idx

    return 0.0, 0.0, FSM_IDLE, wp_idx


# ==========================================
# MAIN
# ==========================================
def restore_session_modal(screen, clock, font, small, doc):
    """
    Modal de restauración de sesión.
    Muestra los datos del último estado y espera que el usuario
    presione S (restaurar) o N (nueva sesión).
    Retorna True si el usuario quiere restaurar, False si no.
    """
    W, H  = screen.get_size()
    mw, mh = 560, 340
    mx    = (W - mw) // 2
    my    = (H - mh) // 2

    ts         = doc.get("timestamp", "desconocido")
    clean_exit = doc.get("clean_exit", False)
    x_mm       = doc.get("x_mm", 0.0)
    y_mm       = doc.get("y_mm", 0.0)
    yaw_deg    = doc.get("yaw_deg", 0.0)
    patron     = doc.get("patron_actual", PATRON_NINGUNO)
    threshold  = doc.get("clean_threshold", CLEAN_THRESHOLD_DEFAULT)
    replan_n   = doc.get("replan_count", 0)
    rows       = doc.get("grid_rows", 0)
    cols       = doc.get("grid_cols", 0)

    # Cobertura estimada desde cleaned/cleanable arrays si están
    coverage = 0.0
    try:
        if "cleaned" in doc and "cleanable" in doc and rows > 0 and cols > 0:
            shape = (rows, cols)
            cl  = StateManager._decode_array(doc["cleaned"],   shape)
            ca  = StateManager._decode_array(doc["cleanable"], shape)
            tot = int(np.sum(ca))
            if tot > 0:
                coverage = 100.0 * int(np.sum(cl & ca)) / tot
    except Exception:
        pass

    exit_str = "cierre limpio ✓" if clean_exit else "crash / desconexión !"
    exit_col = COLOR_OK if clean_exit else COLOR_WARN

    title_font = pygame.font.SysFont("Arial", 22, bold=True)

    while True:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                return False
            if e.type == pygame.KEYDOWN:
                if e.key in (pygame.K_s, pygame.K_RETURN):
                    return True
                if e.key in (pygame.K_n, pygame.K_ESCAPE):
                    return False

        # Dim background
        screen.fill(COLOR_BG)
        dim = pygame.Surface((W, H), pygame.SRCALPHA)
        dim.fill((0, 0, 0, 160))
        screen.blit(dim, (0, 0))

        # Modal box
        pygame.draw.rect(screen, (20, 22, 34),
                         (mx, my, mw, mh), border_radius=14)
        pygame.draw.rect(screen, COLOR_BORDER,
                         (mx, my, mw, mh), 2, border_radius=14)

        # Title
        title = title_font.render("Sesión anterior detectada", True, COLOR_TEXT)
        screen.blit(title, (mx + (mw - title.get_rect().width) // 2, my + 18))

        pygame.draw.line(screen, COLOR_BORDER,
                         (mx + 20, my + 52), (mx + mw - 20, my + 52), 1)

        # Info rows
        rows_info = [
            ("Guardado:",     ts,                      COLOR_SUBTEXT),
            ("Origen:",       exit_str,                exit_col),
            ("Pose X / Y:",   f"{x_mm:.0f} mm  /  {y_mm:.0f} mm", COLOR_SUBTEXT),
            ("Orientación:",  f"{yaw_deg:.1f}°",       COLOR_SUBTEXT),
            ("Cobertura:",    f"{coverage:.1f}%",
                COLOR_THRESH_OK if coverage >= threshold else COLOR_THRESH_WARN),
            ("Umbral:",       f"{threshold:.0f}%",     COLOR_SUBTEXT),
            ("Patrón:",       patron,
                COLOR_PATRON.get(patron, COLOR_SUBTEXT)),
            ("Replans:",      str(replan_n),           COLOR_SUBTEXT),
            ("Mapa:",         f"{rows}×{cols} celdas", COLOR_SUBTEXT),
        ]

        ry = my + 64
        for label, value, vcol in rows_info:
            ls = small.render(label, True, (130, 140, 160))
            vs = small.render(value, True, vcol)
            screen.blit(ls, (mx + 28,  ry))
            screen.blit(vs, (mx + 175, ry))
            ry += 22

        # Buttons
        by = my + mh - 54
        pygame.draw.line(screen, COLOR_BORDER,
                         (mx + 20, by - 8), (mx + mw - 20, by - 8), 1)

        btn_w = 200; gap = 20
        # S — Restore
        sr = pygame.Rect(mx + (mw//2) - btn_w - gap//2, by, btn_w, 36)
        pygame.draw.rect(screen, (20, 70, 40), sr, border_radius=8)
        pygame.draw.rect(screen, COLOR_OK, sr, 1, border_radius=8)
        ss = font.render("S  /  Enter  —  Restaurar", True, COLOR_OK)
        screen.blit(ss, (sr.x + (btn_w - ss.get_rect().width)//2, by + 9))

        # N — New
        nr = pygame.Rect(mx + mw//2 + gap//2, by, btn_w, 36)
        pygame.draw.rect(screen, (60, 20, 20), nr, border_radius=8)
        pygame.draw.rect(screen, COLOR_WARN, nr, 1, border_radius=8)
        ns = font.render("N  /  Esc   —  Nueva sesión", True, COLOR_WARN)
        screen.blit(ns, (nr.x + (btn_w - ns.get_rect().width)//2, by + 9))

        pygame.display.flip()
        clock.tick(30)


def main():
    pygame.init()
    screen = pygame.display.set_mode((ANCHO_VENTANA, ALTO_VENTANA))
    pygame.display.set_caption("Robot Cleaner Pro - UI + Full Grid Visible")
    clock = pygame.time.Clock()

    font = pygame.font.SysFont("Arial", 18, bold=True)
    small = pygame.font.SysFont("Consolas", 14)

    # ── Constantes activas fijas para modo LiDAR+gyro ──
    active_pwm_base       = OP_PWM_BASE
    active_match_every    = OP_MATCH_EVERY
    active_icp_iters      = OP_ICP_ITERS
    active_slow_dyn_mm    = SLOW_ZONE_DYN_MM * OP_SLOW_MULT
    active_slow_sta_mm    = SLOW_ZONE_STA_MM * OP_SLOW_MULT
    active_follow_align   = OP_FOLLOW_ALIGN
    active_follow_advance = OP_FOLLOW_ADVANCE
    print("═" * 62)
    print("  MODO LiDAR+GYRO simplificado activado")
    print(f"  PWM_BASE={active_pwm_base}")
    print(f"  ICP cada {active_match_every} ticks × {active_icp_iters} iters")
    print(f"  Slowdown zones ×{OP_SLOW_MULT}")
    print("═" * 62)

    lidar   = LiDAR_LD20(PUERTO_LIDAR, BAUD_LIDAR)
    arduino = ArduinoSerialController(PUERTO_ARDUINO, BAUD_ARDUINO)
    camera      = CameraCapture(index=CAM_INDEX)
    cam_ok      = camera.connect()
    camera_rear = CameraCapture(index=CAM_REAR_INDEX)
    cam_rear_ok = camera_rear.connect()

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

    # ── StateManager y restauración de sesión ────────────
    state_mgr = StateManager()
    saved_doc = state_mgr.load()

    clean_threshold_init   = CLEAN_THRESHOLD_DEFAULT
    patron_actual_init     = PATRON_NINGUNO
    replan_count_init      = 0

    if saved_doc is not None:
        restore = restore_session_modal(screen, clock, font, small, saved_doc)
        if restore:
            # Restaurar pose
            lidar_x_mm = saved_doc.get("x_mm", LIDAR_X_INICIAL_MM)
            lidar_y_mm = saved_doc.get("y_mm", LIDAR_Y_INICIAL_MM)
            # Restaurar grillas
            ok = state_mgr.restore_grids(saved_doc, grid_map)
            if ok:
                # Recomputar wall_centers para scan matching si hay paredes
                pass   # se hace más abajo cuando wall_centers_xy se asigna
            # Restaurar meta
            clean_threshold_init = saved_doc.get("clean_threshold", CLEAN_THRESHOLD_DEFAULT)
            patron_actual_init   = saved_doc.get("patron_actual",   PATRON_NINGUNO)
            replan_count_init    = saved_doc.get("replan_count",    0)
            print(f"[StateManager] Sesión restaurada desde {saved_doc.get('timestamp','?')}")
        else:
            state_mgr.delete()
            print("[StateManager] Nueva sesión iniciada.")

    clean_sys = False
    comp_on = False
    pump_override_on = False
    brush_override_on = False
    relay_emergency_open = False

    # ── Offsets de yaw (Fase 2-3: único offset) ─────────────
    # Todos los offsets viejos se eliminaron. saved_yaw_offset es el único
    # valor que ajusta el yaw y ya NO se ajusta manualmente: se corrige
    # automáticamente cuando el scan match contra el cuadrado converge.
    # Para retrocompatibilidad con sesiones viejas, el body_pose_yaw_offset
    # guardado se absorbe al saved_yaw_offset en la inicialización.
    _legacy_body_offset = saved_doc.get("body_pose_yaw_offset_deg", 0.0) if saved_doc else 0.0

    # ── Cuadrado de referencia (diagnóstico visual) ────────
    # Se centra en el CENTRO DEL CUERPO del robot en pose inicial
    # (no en el LiDAR). Una vez calculado, queda fijo en el mapa y no
    # se mueve aunque el robot se desplace.
    show_reference_square     = REFERENCE_SQUARE_ENABLED
    # Los valores reales se calculan más abajo, tras determinar pose inicial.
    reference_square_center_x = None
    reference_square_center_y = None

    # ── Scan matching automático + confianza de pose ───────
    # Fase 3: pose XY se corrige SOLO por matching contra el cuadrado.
    # - auto_match_counter: cuenta ticks; cada REF_MATCH_AUTO_EVERY_TICKS
    #   se dispara un match si el robot está quieto.
    # - pose_confidence_ticks: ticks desde último match exitoso. Si supera
    #   POSE_CONFIDENCE_MAX_TICKS, en modo RUTA el robot se detiene.
    auto_match_counter    = 0
    pose_confidence_ticks = 0

    pwr_idx = 0

    pwm_base = active_pwm_base
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
    # Si se restauraron paredes, precomputar wall_centers inmediatamente
    if grid_map.has_wall_map():
        wall_centers_xy = grid_map.get_wall_centers_xy()
    else:
        wall_centers_xy = None
    match_tick_counter   = 0

    # ── Turn pause (evitar matching durante giros) ──────
    turn_settle_counter  = 0       # ticks de espera tras detectar giro
    was_turning_prev     = False   # estado anterior para edge detection
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
    patron_actual       = patron_actual_init
    patron_idx          = (PATRONES_CICLO.index(patron_actual_init)
                           if patron_actual_init in PATRONES_CICLO else 0)

    # ── Replanning de limpieza ───────────────────────────
    clean_threshold     = clean_threshold_init
    replan_count        = replan_count_init
    cleaning_complete   = False

    # ── UI estado ────────────────────────────────────────
    dropdown_open    = False
    last_export_file = None

    # Offset de yaw: absorbe el yaw guardado en sesión anterior.
    # El Arduino siempre arranca en 0, así que sumamos el ángulo previo.
    # En arranque limpio, se aplica FRAME_ROTATION_DEG — rota TODO el frame
    # del mapa según la orientación física que el usuario especifica.
    if saved_doc is not None:
        saved_yaw_offset = saved_doc.get("yaw_deg", LIDAR_YAW_INICIAL_DEG)
        # Migración Fase 2: si hay body_pose_yaw_offset_deg guardado,
        # lo absorbemos aquí para que el offset nuevo sea uno solo.
        if abs(_legacy_body_offset) > 0.05:
            saved_yaw_offset += _legacy_body_offset
            print(f"[MIGRACIÓN] Absorbido body_pose_yaw_offset legacy: "
                  f"{_legacy_body_offset:+.1f}° → saved_yaw_offset={saved_yaw_offset:+.1f}°")
    else:
        saved_yaw_offset = LIDAR_YAW_INICIAL_DEG + STARTUP_YAW_OFFSET_DEG
        if abs(STARTUP_YAW_OFFSET_DEG) > 0.05:
            print(f"[FRAME] Rotación inicial del frame: {STARTUP_YAW_OFFSET_DEG:+.1f}°")

    # ── Centro del cuadrado de referencia ──────────────────
    # Se calcula UNA SOLA VEZ con la pose inicial del robot. A partir de
    # aquí es una constante: el cuadrado no se mueve aunque el robot sí.
    # El centro del cuerpo del robot está desplazado del centro del LiDAR
    # por body_offset en dirección del heading inicial.
    _body_offset_mm  = (DIST_LIDAR_FRENTE_MM - DIST_LIDAR_ATRAS_MM) / 2.0
    _init_yaw_rad    = math.radians(saved_yaw_offset)
    _init_fwd_x      = math.sin(_init_yaw_rad)
    _init_fwd_y      = math.cos(_init_yaw_rad)
    reference_square_center_x = lidar_x_mm + _body_offset_mm * _init_fwd_x
    reference_square_center_y = lidar_y_mm + _body_offset_mm * _init_fwd_y
    print(f"[REF SQUARE] Centro fijado en "
          f"({reference_square_center_x:.0f}, {reference_square_center_y:.0f}) mm "
          f"— lado {REFERENCE_SQUARE_SIDE_MM/1000:.1f}m")

    # ── Estado de cámara frontal / ROI ──────────────────
    cam_dirt_ema    = saved_doc.get("cam_dirt_ema", 0.0) if saved_doc else 0.0
    cam_tile_label  = CAM_LABEL_CLEAN
    cam_clean_ratio = 1.0       # fracción NO-blanca del frame (lo limpio)
    cam_dirt_ratio  = 0.0       # fracción blanca (lo sucio)
    cam_aux_pwm     = 0
    cam_roi_corners = []
    cam_front_surf  = None   # pygame.Surface para preview en panel

    # ── Estado de cámara trasera / ROI ──────────────────
    cam_rear_tile_label  = CAM_LABEL_CLEAN
    cam_rear_clean_ratio = 1.0
    cam_rear_dirt_ratio  = 0.0
    cam_rear_dirt_ema    = 0.0
    cam_rear_roi_corners = []
    cam_rear_surf        = None   # pygame.Surface para preview en panel
    cam_rear_cells_reset = 0

    # ── PID heading ──────────────────────────────────────
    heading_pid         = HeadingPID()
    gyro_prev_yaw       = saved_yaw_offset   # FIX: init con yaw restaurado, no 0

    # ── Incertidumbre de pose (para halo visual del robot) ─
    # Ya no viene de un Kalman; se aproxima con la calidad de match.
    pose_uncertainty_mm = 0.0

    # ── Breadcrumbs para retorno inteligente ─────────────
    breadcrumbs         = BreadcrumbTrail()

    # ── Retorno a casa con A* ─────────────────────────────
    home_path           = []   # waypoints del camino al home (calculado con astar_path)
    home_path_idx       = 0

    # ── ICP y pose adaptativa ─────────────────────────────
    # ICP: scan_matching_step se llama hasta ICP_ITERATIONS veces por tick
    # (ya integrado en el bloque de estimación de pose)

    # ── Espaciado adaptativo entre filas ─────────────────
    adaptive_paso_mm    = float(PASO_LIMPIEZA_MM)

    # ── Session log ───────────────────────────────────────
    session_log         = []
    SESSION_LOG_MAX     = 3600   # ~1 hora a LOG_EVERY_N_TICKS=30 y 30FPS
    log_tick_counter    = 0     # acumulado de celdas desmarcadas por ROI trasero

    # ── Panel colapsable ──────────────────────────────────
    expanded_sections = {"SISTEMA"}  # SISTEMA abierto por defecto

    # ── Obstáculos y velocidades ─────────────────────────
    wait_dyn_ticks    = 0
    speed_scale       = 1.0
    nearest_dyn_dist  = float('inf')
    nearest_sta_dist  = float('inf')

    # Posición de inicio de misión
    mission_home_x   = saved_doc.get("mission_home_x", LIDAR_X_INICIAL_MM) if saved_doc else LIDAR_X_INICIAL_MM
    mission_home_y   = saved_doc.get("mission_home_y", LIDAR_Y_INICIAL_MM) if saved_doc else LIDAR_Y_INICIAL_MM
    mission_home_set = (saved_doc is not None and
                        (abs(mission_home_x - LIDAR_X_INICIAL_MM) > 10 or
                         abs(mission_home_y - LIDAR_Y_INICIAL_MM) > 10))
    is_returning_home = False

    # ── Estado del rotate/settle del follower ───────────────
    # El follower usa este dict mutable para recordar los timestamps de
    # los bursts de giro y las pausas de settle entre bursts.
    fsm_rotate_state = {
        'burst_start_t':  None,
        'settle_start_t': None,
        'last_fsm':       None,
    }

    # ── Estado del escape por rotación 90° ─────────────────
    # Al detectar obstáculo en el cono frontal, el robot rota 90° en
    # ESCAPE_ROTATION_SIGN hasta encontrar camino libre. escape_state
    # lleva el target de yaw a alcanzar y el número de intentos acumulados.
    escape_state = {
        'target_yaw':  None,  # yaw absoluto al que debe rotar
        'attempts':    0,     # intentos hechos (cada uno es 90°)
        'saved_wp':    None,  # índice de wp que se estaba persiguiendo
    }

    # ── Anti-stuck ───────────────────────────────────────
    wp_attempt_counts     = {}    # {wp_idx: num_attempts}
    stuck_pose_history    = []    # [(x, y, tick), ...] para ventana
    stuck_align_counter   = 0     # ticks consecutivos en ALIGN/ROTATE
    in_stuck_recovery     = False

    # ── Zonas no-go (rectángulos prohibidos) ─────────────
    nogo_zones            = NoGoZones()
    if saved_doc and "nogo_zones" in saved_doc:
        nogo_zones.from_list(saved_doc["nogo_zones"])
    nogo_drawing          = False
    nogo_draw_start       = None

    # ── Recovery watchdogs ──────────────────────────────
    map_corrupt_stuck_ticks = 0
    backup_mgr              = StateBackupManager(state_mgr.filepath)

    # ── Mapa corrupto (auto-F1) ─────────────────────────
    map_corrupt_prev_x      = None
    map_corrupt_prev_y      = None
    map_corrupt_no_progress = 0       # ticks consecutivos sin progreso

    last_render_time = 0.0
    robot_moving     = False

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
                    _mods = pygame.key.get_mods()
                    _shift_held = bool(_mods & pygame.KMOD_SHIFT)

                    # Shift+E mantiene su función: toggle relé de emergencia
                    if e.key == pygame.K_e and _shift_held:
                        relay_emergency_open = not relay_emergency_open
                        print(f"[RELAY] Emergencia: {'ABIERTO / compresor cortado' if relay_emergency_open else 'AUTO / relé cerrado'}")

                    elif e.key == pygame.K_F6:
                        # F6: scan match completo AHORA (XY + yaw).
                        # Corrige la pose inmediatamente contra el cuadrado.
                        if (show_reference_square and
                                reference_square_center_x is not None and
                                len(lidar_hits_global) >= REF_MATCH_MIN_PAIRS):
                            res = scan_match_full_against_square(
                                lidar_hits_global,
                                lidar_x_mm, lidar_y_mm,
                                reference_square_center_x,
                                reference_square_center_y,
                                REFERENCE_SQUARE_SIDE_MM,
                            )
                            if res is not None:
                                # F6 aplica corrección COMPLETA (sin damping)
                                lidar_x_mm += res['dx']
                                lidar_y_mm += res['dy']
                                lidar_x_mm = clamp(lidar_x_mm, 0.0, float(MAPA_ANCHO_MM))
                                lidar_y_mm = clamp(lidar_y_mm, 0.0, float(MAPA_ALTO_MM))
                                if res['yaw_confidence'] > 0.0:
                                    saved_yaw_offset += res['dyaw']
                                    # Normalizar a (-180, 180]
                                    saved_yaw_offset = (
                                        (saved_yaw_offset + 540.0) % 360.0 - 180.0
                                    )
                                last_match_quality   = res['quality']
                                last_match_corr_mm   = math.hypot(res['dx'], res['dy'])
                                last_pose_correction = last_match_corr_mm
                                pose_confidence_ticks = 0
                                print(f"[MATCH F6] dx={res['dx']:+.1f}mm "
                                      f"dy={res['dy']:+.1f}mm "
                                      f"dyaw={res['dyaw']:+.2f}° "
                                      f"pairs={res['n_pairs']} "
                                      f"q={res['quality']:.0f}% "
                                      f"walls={int(res['yaw_confidence']*4)}/4")
                            else:
                                print("[MATCH F6] Falló: pocos pares válidos")
                        else:
                            print("[MATCH F6] Requiere cuadrado de referencia activo (F4) "
                                  "y hits LiDAR suficientes")

                    elif e.key == pygame.K_m:
                        # M = Exportar mapa PNG (antes estaba en E)
                        coverage_now = grid_map.get_clean_coverage_pct()
                        last_export_file = export_cleaning_map(grid_map, coverage_now)
                        print(f"Mapa exportado: {last_export_file}")
                    elif e.key == pygame.K_n:
                        # N = toggle modo "dibujar no-go zone"
                        nogo_drawing = not nogo_drawing
                        nogo_draw_start = None
                        print(f"[NO-GO] Modo dibujo: "
                              f"{'ACTIVO (arrastra mouse)' if nogo_drawing else 'OFF'}")

                    elif e.key == pygame.K_DELETE:
                        # Delete = borrar última zona no-go
                        if nogo_zones.zones:
                            nogo_zones.remove_last()
                            print(f"[NO-GO] Zona removida. Restantes: "
                                  f"{len(nogo_zones.zones)}")

                    elif e.key == pygame.K_l:
                        # Exportar log de sesión CSV
                        if session_log:
                            log_file = export_session_log(session_log)
                            if log_file:
                                print(f"Log exportado: {log_file} ({len(session_log)} entradas)")
                        else:
                            print("[Log] Sin datos de sesión aún.")

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
                            arduino.send_command(0, 0, 0, False, False, relay_emergency_open)
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
                                        arduino.send_command(0, 0, 0, False, False, relay_emergency_open)
                            else:
                                ruta_waypoints     = []
                                waypoint_idx       = 0
                                skipped_wp_indices = set()

                    elif e.key == pygame.K_F3:
                        # Forzar replanning inmediato
                        if grid_map.has_cleanable_mask() and modo != MODE_SCAN:
                            wps, pat, ok = do_replan(grid_map, paso_mm=adaptive_paso_mm, nogo_zones=nogo_zones)
                            if ok:
                                ruta_waypoints     = wps
                                patron_actual      = pat
                                waypoint_idx       = 0
                                skipped_wp_indices = set()
                                path_check_counter = 0
                                fsm_state          = FSM_IDLE
                                replan_count      += 1
                                cleaning_complete  = False
                                cam_rear_cells_reset = 0  # FIX: reset counter on replan
                                # FIX: restore adaptive paso partially on replan success
                                adaptive_paso_mm = min(
                                    float(PASO_LIMPIEZA_MM),
                                    adaptive_paso_mm + ADAPTIVE_PASO_STEP_MM * 2
                                )
                                if modo == MODE_RUTA:
                                    modo = MODE_MANUAL
                                    pl, pr = 0.0, 0.0
                                    arduino.send_command(0, 0, 0, False, False, relay_emergency_open)

                    elif e.key == pygame.K_F4:
                        # F4: toggle cuadrado de referencia (diagnóstico)
                        show_reference_square = not show_reference_square
                        print(f"[REF SQUARE] {'ON' if show_reference_square else 'OFF'}")

                    # ── Panel section toggles ─────────────────────
                    elif e.key == pygame.K_7:
                        expanded_sections ^= {"POSE"}
                    elif e.key == pygame.K_8:
                        expanded_sections ^= {"RUTA"}
                    elif e.key == pygame.K_9:
                        expanded_sections ^= {"CAMARA"}
                    elif e.key == pygame.K_0:
                        expanded_sections ^= {"SISTEMA"}

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
                            arduino.send_command(0, 0, 0, False, False, relay_emergency_open)
                            waypoint_idx       = 0
                            fsm_state          = FSM_IDLE
                            skipped_wp_indices = set()
                            path_check_counter = 0
                            # El home ya NO se guarda automáticamente aquí.
                            # El usuario debe usar el botón "Guardar Home" del
                            # panel RUTA para registrar la pose de inicio.
                            if not mission_home_set:
                                print("[RUTA] Advertencia: no se ha guardado "
                                      "home. El robot no podrá volver al "
                                      "punto de inicio al terminar.")
                            # Reset estado de retroceso
                            wait_dyn_ticks    = 0
                            is_returning_home = False
                        else:
                            pl, pr = 0.0, 0.0
                            arduino.send_command(0, 0, 0, False, False, relay_emergency_open)
                            fsm_state         = FSM_IDLE
                            wait_dyn_ticks    = 0
                    elif e.key == pygame.K_BACKSPACE:
                        ruta_waypoints.clear()
                        waypoint_idx       = 0
                        skipped_wp_indices = set()
                        path_check_counter = 0
                    elif e.key == pygame.K_r:
                        clean_sys = not clean_sys
                    elif e.key == pygame.K_o:
                        comp_on = not comp_on
                    elif e.key == pygame.K_b:
                        pump_override_on = not pump_override_on
                        print(f"[ACT] Bomba override: {'MANUAL ON' if pump_override_on else 'AUTO'}")
                    elif e.key == pygame.K_c:
                        brush_override_on = not brush_override_on
                        print(f"[ACT] Cepillos override: {'MANUAL ON' if brush_override_on else 'AUTO'}")
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
                        arduino.send_command(0, 0, 0, False, False, relay_emergency_open)
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
                        # FIX: reset completo del estado de Fase 2 y sensores
                        mission_home_x        = LIDAR_X_INICIAL_MM
                        mission_home_y        = LIDAR_Y_INICIAL_MM
                        mission_home_set      = False
                        is_returning_home     = False
                        wait_dyn_ticks        = 0
                        adaptive_paso_mm      = float(PASO_LIMPIEZA_MM)
                        cam_rear_cells_reset  = 0
                        cam_dirt_ratio        = 0.0
                        cam_rear_dirt_ratio   = 0.0
                        session_log.clear()
                        log_tick_counter      = 0
                        heading_pid.reset()
                        # Reset breadcrumbs + pose uncertainty
                        pose_uncertainty_mm   = 0.0
                        breadcrumbs.clear()
                        home_path             = []
                        home_path_idx         = 0
                        # Reset anti-stuck
                        wp_attempt_counts     = {}
                        stuck_pose_history    = []
                        stuck_align_counter   = 0
                        in_stuck_recovery     = False
                        # Reset rotate/settle/escape states
                        fsm_rotate_state['burst_start_t']  = None
                        fsm_rotate_state['settle_start_t'] = None
                        fsm_rotate_state['last_fsm']       = None
                        escape_state['target_yaw'] = None
                        escape_state['attempts']   = 0
                        escape_state['saved_wp']   = None
                        # Reset rotación de vista
                        grid_map.view_rotation_deg = 0.0
                        # Reset detección de obstáculos + corrupt-map detection
                        map_corrupt_prev_x      = None
                        map_corrupt_prev_y      = None
                        map_corrupt_no_progress = 0
                        # NOTA: nogo_zones NO se borran con P (usar botón)
                        # re-inicializar referencia de yaw tras reset
                        gyro_prev_yaw         = saved_yaw_offset + arduino.get_yaw()
                        # Nota: las paredes NO se borran con P
                        # Para re-escanear paredes usar F1

                # ── Click del mouse ───────────────────────────────────
                if e.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = e.pos
                    _click_consumed = False

                    # ── Modo dibujo no-go: empezar rectángulo ──
                    if (nogo_drawing and e.button == 1
                            and grid_map.is_inside_map_rect(map_rect, mx, my)):
                        wx, wy = grid_map.screen_to_world(map_rect, mx, my)
                        nogo_draw_start = (wx, wy)
                        _click_consumed = True

                    # Toggle Controls dropdown — prioridad más alta
                    if not _click_consumed and _CTRL_BTN_RECT and _CTRL_BTN_RECT.collidepoint(mx, my):
                        dropdown_open   = not dropdown_open
                        _click_consumed = True

                    # Cerrar dropdown si se hace click fuera de él
                    elif not _click_consumed and dropdown_open:
                        dropdown_open   = False
                        _click_consumed = True   # no procesar más este click

                    # Click en botón "Guardar Home" (panel RUTA)
                    if (not _click_consumed
                            and _SAVE_HOME_BTN_RECT is not None
                            and _SAVE_HOME_BTN_RECT.collidepoint(mx, my)):
                        mission_home_x   = lidar_x_mm
                        mission_home_y   = lidar_y_mm
                        mission_home_set = True
                        _click_consumed  = True
                        print(f"[HOME] Guardado en ({mission_home_x:.0f}, "
                              f"{mission_home_y:.0f})mm yaw={yaw_deg:.1f}°")

                    # Click en botón "Borrar zonas no-go" (panel RUTA)
                    if (not _click_consumed
                            and _CLEAR_NOGO_BTN_RECT is not None
                            and _CLEAR_NOGO_BTN_RECT.collidepoint(mx, my)):
                        n = len(nogo_zones.zones)
                        nogo_zones.clear()
                        _click_consumed = True
                        print(f"[NO-GO] {n} zonas eliminadas")

                    # Toggle secciones del panel (solo si dropdown no consumió)
                    if not _click_consumed and mx >= MAP_W:
                        for sec_name, sec_rect in _PANEL_SECTION_RECTS.items():
                            if sec_rect.collidepoint(mx, my):
                                expanded_sections ^= {sec_name}
                                _click_consumed = True
                                break

                    # Waypoint clicks en el mapa
                    if (not _click_consumed and my > TOOLBAR_H
                            and grid_map.is_inside_map_rect(map_rect, mx, my)):
                        if e.button == 1:
                            wx, wy = grid_map.screen_to_world(map_rect, mx, my)
                            wx = clamp(wx, 0.0, float(MAPA_ANCHO_MM))
                            wy = clamp(wy, 0.0, float(MAPA_ALTO_MM))
                            wx, wy = grid_map.nearest_free_cell_mm(wx, wy)
                            ruta_waypoints.append((wx, wy))
                        elif e.button == 3:
                            if ruta_waypoints:
                                ruta_waypoints.pop()
                                waypoint_idx = clamp(waypoint_idx, 0,
                                    max(0, len(ruta_waypoints) - 1))

                # ── Fin de arrastre no-go ────────────────────────
                if e.type == pygame.MOUSEBUTTONUP and e.button == 1:
                    if nogo_drawing and nogo_draw_start is not None:
                        mx_up, my_up = e.pos
                        if grid_map.is_inside_map_rect(map_rect, mx_up, my_up):
                            wx2, wy2 = grid_map.screen_to_world(map_rect, mx_up, my_up)
                            added = nogo_zones.add_rect(
                                nogo_draw_start[0], nogo_draw_start[1],
                                wx2, wy2)
                            if added:
                                print(f"[NO-GO] Zona añadida. Total: "
                                      f"{len(nogo_zones.zones)}")
                            else:
                                print("[NO-GO] Rectángulo muy pequeño, descartado.")
                        nogo_draw_start = None

            # ── Pipeline de sensores ───────────────────────────────
            # Un único yaw_deg para TODO (chasis, ROIs, conos, nube).
            # Si antes había desajustes entre la nube y el resto, era porque
            # se usaban offsets distintos. Ahora todo comparte este valor.
            yaw_deg = saved_yaw_offset + arduino.get_yaw()
            # Aliases legacy para no romper referencias aún no refactorizadas
            yaw_deg_base   = yaw_deg
            yaw_deg_real   = yaw_deg
            yaw_deg_motion = yaw_deg
            yaw_deg_draw   = yaw_deg
            last_dang = arduino.get_last_dang()

            lidar_hits_global = []
            dyn_alert = False
            dyn_count = 0

            ang_new, dist_new = lidar.obtener_datos_nuevos()
            if len(dist_new) > 0:
                lidar_mem.update(ang_new, dist_new, now=now)

            ang, dist = lidar_mem.get_scan(now=now)
            lidar_hits_local  = transform_scan_to_local(ang, dist)
            # La nube se deposita en el mapa usando yaw puro del giroscopio.
            # No hay ajuste manual: la pose XY se corrige por scan matching
            # contra el cuadrado de referencia (F6 o automático).
            lidar_hits_global = transform_scan_to_global(
                ang, dist, lidar_x_mm, lidar_y_mm, yaw_deg
            )

            # ── CÁMARA FRONTAL: percepción visual ──────────────────
            cam_ok = camera.is_ok()
            cam_roi_corners = compute_roi_corners(lidar_x_mm, lidar_y_mm, yaw_deg_motion)

            if cam_ok:
                frame = camera.get_frame()
                if frame is not None:
                    cam_clean_ratio, _dirt_raw = analyze_frame_dirt(frame)
                    cam_dirt_ratio = ((1.0 - CAM_EMA_ALPHA) * cam_dirt_ratio
                                      + CAM_EMA_ALPHA * _dirt_raw)
                    cam_tile_label = classify_tile(cam_clean_ratio)

                    # ── Proyectar dirt medido al grid (celdas bajo el ROI) ──
                    if DIRT_GRID_ENABLED and cam_roi_corners:
                        grid_map.update_dirt_roi(cam_roi_corners, _dirt_raw)

                    # Preparar surface pygame para preview (BGR→RGB→Surface)
                    if _CV2_AVAILABLE:
                        try:
                            rgb = _cv2.cvtColor(frame, _cv2.COLOR_BGR2RGB)
                            cam_front_surf = pygame.surfarray.make_surface(
                                rgb.swapaxes(0, 1))
                        except Exception:
                            cam_front_surf = None
            else:
                cam_clean_ratio = 1.0
                cam_dirt_ratio  = 0.0
                cam_tile_label  = CAM_LABEL_CLEAN
                cam_front_surf  = None

            # PWM auxiliar adaptativo por visión
            if clean_sys:
                comp_base = int(255 * NIVELES_LIMPIEZA[pwr_idx])
                cam_aux_pwm = dirt_to_aux_pwm(cam_dirt_ratio,
                                               comp_on=comp_on,
                                               comp_pwm=comp_base)
            else:
                cam_aux_pwm = 0

            # ── CÁMARA TRASERA: validación de limpieza ──────────────
            cam_rear_ok = camera_rear.is_ok()
            cam_rear_roi_corners = compute_rear_roi_corners(
                lidar_x_mm, lidar_y_mm, yaw_deg_motion)

            if cam_rear_ok:
                rear_frame = camera_rear.get_frame()
                if rear_frame is not None:
                    cam_rear_clean_ratio, _rdirt = analyze_frame_dirt(rear_frame)
                    cam_rear_dirt_ratio = ((1.0 - CAM_EMA_ALPHA) * cam_rear_dirt_ratio
                                           + CAM_EMA_ALPHA * _rdirt)
                    cam_rear_tile_label = classify_tile(cam_rear_clean_ratio)

                    # ── Actualizar dirt grid con medición trasera ──
                    # La cámara trasera valida si el robot realmente limpió.
                    # Si todavía está sucio, actualizar grid para próximo replan.
                    if DIRT_GRID_ENABLED and len(cam_rear_roi_corners) == 4:
                        grid_map.update_dirt_roi(cam_rear_roi_corners, _rdirt)

                    # Retroalimentar mapa si la zona trasera sigue sucia
                    if (cam_rear_dirt_ratio > CAM_REAR_DIRT_FEEDBACK_THR
                            and len(cam_rear_roi_corners) == 4):
                        n_reset = grid_map.unmark_cleaned_in_roi(cam_rear_roi_corners)
                        if n_reset > 0:
                            cam_rear_cells_reset += n_reset
                            if n_reset >= ADAPTIVE_DIRTY_CELLS:
                                adaptive_paso_mm = max(
                                    ADAPTIVE_PASO_MIN_MM,
                                    adaptive_paso_mm - ADAPTIVE_PASO_STEP_MM
                                )

                    # Surface para preview
                    if _CV2_AVAILABLE:
                        try:
                            rgb_r = _cv2.cvtColor(rear_frame, _cv2.COLOR_BGR2RGB)
                            cam_rear_surf = pygame.surfarray.make_surface(
                                rgb_r.swapaxes(0, 1))
                        except Exception:
                            cam_rear_surf = None
            else:
                cam_rear_clean_ratio = 1.0
                cam_rear_dirt_ratio  = 0.0
                cam_rear_tile_label  = CAM_LABEL_CLEAN
                cam_rear_surf        = None

            # ── Fase D: clasificar hits en estáticos / dinámicos ──
            dynamic_hits_render, static_hits_render, _ = classify_hits(
                lidar_hits_global, lidar_x_mm, lidar_y_mm, yaw_deg_motion
            )

            # Conteo dinámico global simplificado
            dyn_count = len(dynamic_hits_render)
            dyn_alert = dyn_count >= UMBRAL_PUNTOS_DINAMICOS

            # Obstáculo frontal persistente con clasificación geométrica
            n_frontal, _ = check_frontal_obstacle(
                dynamic_hits_render, lidar_x_mm, lidar_y_mm, yaw_deg_motion
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
                # ════════════════════════════════════════════════
                # MODO RUTA — Fase 2: desaceleración + backup
                # ════════════════════════════════════════════════

                # ── Calcular distancias al obstáculo más cercano ─
                _, frontal_dyn_hits = check_frontal_obstacle(
                    dynamic_hits_render, lidar_x_mm, lidar_y_mm, yaw_deg_motion)
                _, frontal_sta_hits = check_frontal_obstacle(
                    static_hits_render,  lidar_x_mm, lidar_y_mm, yaw_deg_motion)

                nearest_dyn_dist = get_nearest_hit_dist(
                    frontal_dyn_hits, lidar_x_mm, lidar_y_mm)
                nearest_sta_dist = get_nearest_hit_dist(
                    frontal_sta_hits, lidar_x_mm, lidar_y_mm)

                # Factor de velocidad: el más restrictivo gana
                scale_dyn  = obstacle_speed_scale(
                    nearest_dyn_dist, active_slow_dyn_mm, STOP_ZONE_DYN_MM)
                scale_sta  = obstacle_speed_scale(
                    nearest_sta_dist, active_slow_sta_mm, STOP_ZONE_STA_MM)
                speed_scale = min(scale_dyn, scale_sta)

                # ── Multiplicador por suciedad del tile adelante ──
                # Mirar LOOKAHEAD_CELLS_DIRT celdas hacia adelante y
                # usar el dirt_ratio de esa celda para ajustar velocidad:
                # tile sucio → ir más lento para limpiar mejor.
                if DIRT_GRID_ENABLED and modo == MODE_RUTA:
                    look_dist = LOOKAHEAD_CELLS_DIRT * grid_map.cell_mm
                    fwd_look, _ = heading_forward_lateral(yaw_deg_motion)
                    look_x = lidar_x_mm + look_dist * fwd_look[0]
                    look_y = lidar_y_mm + look_dist * fwd_look[1]
                    dirt_val = grid_map.get_dirt_cell(look_x, look_y)
                    if dirt_val >= DIRT_THRESHOLD_MEDIUM:
                        dirt_mult = SPEED_MULT_DIRTY
                    elif dirt_val >= DIRT_THRESHOLD_CLEAN:
                        dirt_mult = SPEED_MULT_MEDIUM
                    else:
                        dirt_mult = SPEED_MULT_CLEAN
                    speed_scale *= dirt_mult

                # ── Estado RETURN_HOME: tratar el home como un WP normal ─
                # El home es solo un waypoint. El follower estándar (ROTATE→
                # ROTATE_SETTLE→ADVANCE) lo maneja igual que cualquier otro.
                # Cuando llegue, salimos de RETURN_HOME.
                if fsm_state == FSM_RETURN_HOME:
                    target = (mission_home_x, mission_home_y)
                    dx_h      = target[0] - lidar_x_mm
                    dy_h      = target[1] - lidar_y_mm
                    dist_home = math.hypot(dx_h, dy_h)

                    if dist_home < WP_REACH_MM:
                        # Llegamos al home
                        fsm_state         = FSM_IDLE
                        is_returning_home = False
                        home_path         = []
                        home_path_idx     = 0
                        modo              = MODE_MANUAL
                        pl, pr            = 0.0, 0.0
                        arduino.send_command(0, 0, 0, False, False, relay_emergency_open)
                        print("[RETURN_HOME] Llegada al home.")
                    else:
                        # Construir ruta con A* si no existe o si se agotó
                        if not home_path or home_path_idx >= len(home_path):
                            _astar = astar_path(
                                grid_map,
                                (lidar_x_mm, lidar_y_mm),
                                (mission_home_x, mission_home_y),
                                nogo_zones=nogo_zones,
                            )
                            if _astar and len(_astar) > 0:
                                home_path     = _astar
                                home_path_idx = 0
                                print(f"[RETURN_HOME] Ruta A*: {len(home_path)} wps")
                            else:
                                # A* no encuentra camino → ir directo
                                home_path     = [(mission_home_x, mission_home_y)]
                                home_path_idx = 0
                                print("[RETURN_HOME] A* falló, ir directo al home")

                        # Usar el follower normal sobre home_path
                        pl, pr, _inner, new_hp_idx = follow_route_step(
                            lidar_x_mm, lidar_y_mm, yaw_deg_motion,
                            home_path, home_path_idx,
                            FSM_ROTATE if fsm_rotate_state.get('last_fsm') != FSM_ADVANCE
                                       else FSM_ADVANCE,
                            False,
                            heading_pid=heading_pid,
                            dt=dt,
                            pwm_align=active_follow_align,
                            pwm_advance=active_follow_advance,
                            rotate_state=fsm_rotate_state,
                        )
                        if _inner == FSM_WP_REACHED:
                            home_path_idx = new_hp_idx + 1
                            heading_pid.reset()
                        pl *= speed_scale
                        pr *= speed_scale

                # ── Estado BLOCKED_ROTATE: escape por rotación 90° ─────
                # Cuando el cono frontal detectó bloqueo persistente, el
                # robot rota ESCAPE_ROTATION_DEG (90°) en ESCAPE_ROTATION_SIGN.
                # Al terminar cada rotación reevalúa: si despejó → retomar el
                # WP original; si no → acumular otros 90° hasta ESCAPE_MAX_ATTEMPTS.
                elif fsm_state == FSM_BLOCKED_ROTATE:
                    # Si no hay target, calcular uno basado en yaw actual
                    if escape_state['target_yaw'] is None:
                        escape_state['target_yaw'] = _normalize_angle(
                            yaw_deg_motion + ESCAPE_ROTATION_DEG * ESCAPE_ROTATION_SIGN
                        )
                        escape_state['attempts']  = 0
                        escape_state['saved_wp']  = waypoint_idx
                        fsm_rotate_state['burst_start_t']  = now
                        fsm_rotate_state['settle_start_t'] = None
                        print(f"[ESCAPE] Iniciando rotación de escape — "
                              f"target yaw = {escape_state['target_yaw']:+.1f}°")

                    yaw_err = _normalize_angle(
                        escape_state['target_yaw'] - yaw_deg_motion
                    )

                    # ¿Llegamos al target de rotación?
                    if abs(yaw_err) <= ESCAPE_ROTATION_TOL_DEG:
                        # Pausa obligatoria para que el match converja antes
                        # de reevaluar el cono. Reutilizamos el settle state.
                        if fsm_rotate_state.get('settle_start_t') is None:
                            fsm_rotate_state['settle_start_t'] = now
                            pl, pr = 0.0, 0.0
                        elif (now - fsm_rotate_state['settle_start_t']) * 1000.0 >= ROTATE_SETTLE_MS:
                            # Settle cumplido → evaluar si seguimos bloqueados
                            if not frontal_blocked:
                                # Despejó → volver al WP original
                                print(f"[ESCAPE] Despejado tras "
                                      f"{escape_state['attempts']+1} rotación(es). "
                                      f"Retomando WP {escape_state['saved_wp']}")
                                escape_state['target_yaw'] = None
                                escape_state['attempts']   = 0
                                escape_state['saved_wp']   = None
                                fsm_rotate_state['burst_start_t']  = now
                                fsm_rotate_state['settle_start_t'] = None
                                fsm_state = FSM_ROTATE
                                pl, pr = 0.0, 0.0
                            else:
                                # Sigue bloqueado → otro escalón de 90°
                                escape_state['attempts'] += 1
                                if escape_state['attempts'] >= ESCAPE_MAX_ATTEMPTS:
                                    # 360° sin encontrar salida → skip WP
                                    print(f"[ESCAPE] Giro completo sin salida. "
                                          f"Marcando WP {waypoint_idx} como inalcanzable.")
                                    skipped_wp_indices.add(waypoint_idx)
                                    waypoint_idx = min(waypoint_idx + 1,
                                                        len(ruta_waypoints) - 1)
                                    escape_state['target_yaw'] = None
                                    escape_state['attempts']   = 0
                                    escape_state['saved_wp']   = None
                                    fsm_rotate_state['burst_start_t']  = now
                                    fsm_rotate_state['settle_start_t'] = None
                                    fsm_state = FSM_ROTATE
                                    pl, pr = 0.0, 0.0
                                else:
                                    # Nuevo target +90°
                                    escape_state['target_yaw'] = _normalize_angle(
                                        escape_state['target_yaw']
                                        + ESCAPE_ROTATION_DEG * ESCAPE_ROTATION_SIGN
                                    )
                                    fsm_rotate_state['burst_start_t']  = now
                                    fsm_rotate_state['settle_start_t'] = None
                                    print(f"[ESCAPE] Intento {escape_state['attempts']+1}"
                                          f"/{ESCAPE_MAX_ATTEMPTS} — "
                                          f"nuevo target {escape_state['target_yaw']:+.1f}°")
                                    pl, pr = 0.0, 0.0
                        else:
                            pl, pr = 0.0, 0.0
                    else:
                        # Rotar hacia el target con bursts (misma mecánica que ROTATE)
                        burst_start = fsm_rotate_state.get('burst_start_t')
                        if burst_start is None:
                            fsm_rotate_state['burst_start_t']  = now
                            fsm_rotate_state['settle_start_t'] = None
                            burst_start = now

                        if (now - burst_start) * 1000.0 >= ROTATE_BURST_MS:
                            # Pausa para match a mitad del giro
                            if fsm_rotate_state.get('settle_start_t') is None:
                                fsm_rotate_state['settle_start_t'] = now
                                pl, pr = 0.0, 0.0
                            elif (now - fsm_rotate_state['settle_start_t']) * 1000.0 >= ROTATE_SETTLE_MS:
                                fsm_rotate_state['burst_start_t']  = now
                                fsm_rotate_state['settle_start_t'] = None
                                pl, pr = 0.0, 0.0
                            else:
                                pl, pr = 0.0, 0.0
                        else:
                            # Girar en la dirección que reduce el error
                            if yaw_err > 0:
                                pl, pr = -float(active_follow_align), +float(active_follow_align)
                            else:
                                pl, pr = +float(active_follow_align), -float(active_follow_align)

                # ── Estados normales (IDLE / ROTATE / ROTATE_SETTLE / ADVANCE) ─
                else:
                    # ── Anti-stuck: trackear pose history ────────
                    if modo == MODE_RUTA:
                        stuck_pose_history.append((lidar_x_mm, lidar_y_mm, now))
                        # Mantener solo ventana de observación
                        cutoff = now - (STUCK_POSE_WINDOW_TICKS * INTERVALO_DIBUJO)
                        stuck_pose_history = [
                            h for h in stuck_pose_history if h[2] >= cutoff
                        ]

                    # ── Anti-stuck: contar ticks en ROTATE/ROTATE_SETTLE ──
                    if fsm_state in (FSM_ROTATE, FSM_ROTATE_SETTLE):
                        stuck_align_counter += 1
                    else:
                        stuck_align_counter = 0
                        # Resetear contador de intentos al cambiar de WP
                        # exitosamente (ADVANCE o WP_REACHED limpian el historial
                        # del WP anterior).
                        if fsm_state == FSM_WP_REACHED:
                            # Limpiar el contador del WP que se acaba de alcanzar
                            wp_attempt_counts.pop(waypoint_idx, None)

                    # ── Detectar stuck (dos criterios) ────────────
                    stuck_detected = False
                    if not in_stuck_recovery and modo == MODE_RUTA and ruta_waypoints:
                        # Criterio A: demasiados ticks en ROTATE sin alinearse.
                        # Solo dispara si el error angular actual es grande —
                        # si ya estamos cerca de ALIGN_FINE_DEG el robot está
                        # a punto de entrar a ADVANCE, no hay que interrumpir.
                        if stuck_align_counter >= STUCK_ALIGN_TICKS:
                            # Medir el error angular al WP actual
                            if waypoint_idx < len(ruta_waypoints):
                                _twx, _twy = ruta_waypoints[waypoint_idx]
                                _tgt_head = heading_from_vector_deg(
                                    _twx - lidar_x_mm, _twy - lidar_y_mm)
                                _h_err = abs(_normalize_angle(
                                    _tgt_head - yaw_deg_motion))
                                # Solo declara stuck si el error es grande
                                # (si está a 5° no es stuck, solo está siendo preciso)
                                if _h_err > ALIGN_FINE_DEG * 3.0:
                                    stuck_detected = True
                                    print(f"[STUCK] ROTATE por >{STUCK_ALIGN_TICKS} "
                                          f"ticks en WP {waypoint_idx} "
                                          f"(error angular {_h_err:.1f}°)")
                                else:
                                    # Error pequeño: resetear contador y seguir
                                    stuck_align_counter = 0

                        # Criterio B: pose casi fija por ventana larga + PWM activo.
                        # IMPORTANTE: NO aplica durante ROTATE/ROTATE_SETTLE porque
                        # allí el robot NO se mueve en XY, solo rota en sitio.
                        # Solo mide "pose estancada" cuando debería estar avanzando.
                        elif (fsm_state == FSM_ADVANCE
                                and len(stuck_pose_history) >= 30
                                and (abs(pl) > 40 or abs(pr) > 40)):
                            xs = [h[0] for h in stuck_pose_history]
                            ys = [h[1] for h in stuck_pose_history]
                            span = math.hypot(max(xs)-min(xs), max(ys)-min(ys))
                            if span < STUCK_POSE_CHANGE_MM:
                                stuck_detected = True
                                print(f"[STUCK] ADVANCE sin avanzar "
                                      f"({span:.0f}mm en {len(stuck_pose_history)} muestras)")

                    # ── Disparar recovery si corresponde ──────────
                    if stuck_detected:
                        wp_attempt_counts[waypoint_idx] = wp_attempt_counts.get(
                            waypoint_idx, 0) + 1
                        attempts = wp_attempt_counts[waypoint_idx]
                        print(f"[STUCK] Intento {attempts}/{STUCK_MAX_ATTEMPTS} "
                              f"para WP {waypoint_idx}")

                        if attempts >= STUCK_MAX_ATTEMPTS:
                            # Skip permanente del WP actual — pasar al siguiente
                            skipped_wp_indices.add(waypoint_idx)
                            print(f"[STUCK] Skip permanente WP {waypoint_idx}")
                            waypoint_idx = min(waypoint_idx + 1,
                                                len(ruta_waypoints) - 1)
                            fsm_state = FSM_ROTATE
                            heading_pid.reset()
                            stuck_align_counter = 0
                            stuck_pose_history.clear()
                            pl, pr = 0.0, 0.0
                        else:
                            # Recovery sin retroceso: intentar saltar al siguiente WP
                            # si está cerca. Si no, marcar el actual como skip.
                            in_stuck_recovery = True
                            next_idx = waypoint_idx + 1
                            if (next_idx < len(ruta_waypoints)):
                                nxt = ruta_waypoints[next_idx]
                                d_next = math.hypot(nxt[0]-lidar_x_mm,
                                                     nxt[1]-lidar_y_mm)
                                if d_next < STUCK_BYPASS_DIST_MM:
                                    print(f"[STUCK] Saltando al WP {next_idx} "
                                          f"({d_next:.0f}mm) sin retroceso")
                                    skipped_wp_indices.add(waypoint_idx)
                                    waypoint_idx = next_idx
                            fsm_state = FSM_ROTATE
                            heading_pid.reset()
                            stuck_pose_history.clear()
                            in_stuck_recovery = False
                            pl, pr = 0.0, 0.0

                    # ── Follower normal (si no está en recovery) ──
                    # obstacle_hold combina:
                    #   - speed_scale=0 (obstáculo dinámico cerca, frenado total)
                    #   - frontal_blocked (obstáculo persistente confirmado)
                    # Cuando obstacle_hold=True, el follower devuelve
                    # FSM_BLOCKED_ROTATE y la lógica de escape (arriba) toma
                    # el control para rotar 90° hasta encontrar camino libre.
                    if not in_stuck_recovery and not stuck_detected:
                        obstacle_hold = (speed_scale <= 0.0) or frontal_blocked
                        pl, pr, fsm_state, waypoint_idx = follow_route_step(
                            lidar_x_mm, lidar_y_mm, yaw_deg_motion,
                            ruta_waypoints, waypoint_idx,
                            fsm_state, obstacle_hold,
                            heading_pid=heading_pid,
                            dt=dt,
                            pwm_align=active_follow_align,
                            pwm_advance=active_follow_advance,
                            rotate_state=fsm_rotate_state,
                        )
                        # Si el follower acaba de transicionar a BLOCKED_ROTATE,
                        # limpiar escape_state para que el manejador de arriba
                        # configure un target nuevo en el próximo tick.
                        if fsm_state == FSM_BLOCKED_ROTATE and escape_state['target_yaw'] is None:
                            # No seteamos nada aquí; el manejador lo hace en el
                            # siguiente tick. Solo nos aseguramos de no arrastrar
                            # estado basura de un escape previo.
                            escape_state['attempts'] = 0
                            escape_state['saved_wp'] = waypoint_idx
                    # Trackear el último fsm_state para el return_home
                    fsm_rotate_state['last_fsm'] = fsm_state
                    # Aplicar desaceleración proporcional solo mientras avanza.
                    if fsm_state == FSM_ADVANCE and 0.0 < speed_scale < 1.0:
                        pl *= speed_scale
                        pr *= speed_scale

                    # ── ROUTE_DONE: replan o retorno a casa ────────
                    if fsm_state == FSM_ROUTE_DONE:
                        coverage = grid_map.get_clean_coverage_pct()
                        if coverage >= clean_threshold:
                            # Cobertura alcanzada → retorno a inicio de misión
                            cleaning_complete = True
                            if mission_home_set and math.hypot(
                                    lidar_x_mm - mission_home_x,
                                    lidar_y_mm - mission_home_y) > WP_REACH_MM:
                                # Hay que volver a casa
                                fsm_state             = FSM_RETURN_HOME
                                is_returning_home     = True
                                home_path             = []   # forzar recálculo
                                home_path_idx         = 0
                                pl, pr = 0.0, 0.0
                            else:
                                modo      = MODE_MANUAL
                                fsm_state = FSM_IDLE
                                pl, pr    = 0.0, 0.0
                                arduino.send_command(0, 0, 0, False, False, relay_emergency_open)
                        else:
                            # Cobertura insuficiente → replan
                            wps, pat, ok = do_replan(grid_map, paso_mm=adaptive_paso_mm, nogo_zones=nogo_zones)
                            if ok:
                                ruta_waypoints     = wps
                                patron_actual      = pat
                                waypoint_idx       = 0
                                skipped_wp_indices = set()
                                path_check_counter = 0
                                fsm_state          = FSM_IDLE
                                replan_count      += 1
                                cam_rear_cells_reset = 0
                                adaptive_paso_mm = min(
                                    float(PASO_LIMPIEZA_MM),
                                    adaptive_paso_mm + ADAPTIVE_PASO_STEP_MM * 2
                                )
                            else:
                                cleaning_complete = True
                                modo      = MODE_MANUAL
                                fsm_state = FSM_IDLE
                                pl, pr    = 0.0, 0.0
                                arduino.send_command(0, 0, 0, False, False, relay_emergency_open)                # ── Verificación de camino simplificada ─────────────
                # Solo evaluar el waypoint activo. Saltar waypoints futuros
                # de forma masiva causaba conflictos con la limpieza automática.
                if grid_map.has_wall_map():
                    path_check_counter += 1
                    if (path_check_counter >= PATH_CHECK_EVERY_N
                            and 0 <= waypoint_idx < len(ruta_waypoints)):
                        path_check_counter = 0
                        wx, wy = ruta_waypoints[waypoint_idx]
                        if grid_map.path_crosses_wall(lidar_x_mm, lidar_y_mm, wx, wy):
                            skipped_wp_indices.add(waypoint_idx)
                            waypoint_idx += 1
                            while (waypoint_idx < len(ruta_waypoints)
                                   and waypoint_idx in skipped_wp_indices):
                                waypoint_idx += 1
                            if waypoint_idx >= len(ruta_waypoints):
                                modo = MODE_MANUAL
                                fsm_state = FSM_IDLE
                                pl, pr = 0.0, 0.0
                                arduino.send_command(0, 0, 0, False, False, relay_emergency_open)
                            else:
                                fsm_state = FSM_ROTATE
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
                                        arduino.send_command(0, 0, 0, False, False, relay_emergency_open)
                                    else:
                                        fsm_state = FSM_ROTATE
                            i += 1

            for xp, yp in lidar_hits_global:
                grid_map.mark_discovered(xp, yp)

            pl_cmd = pl * SIGNO_MOTOR_IZQ
            pr_cmd = pr * SIGNO_MOTOR_DER

            # FASE 3 — FREEZE POR POSE INCIERTA
            # En modo RUTA, si no hay match exitoso en POSE_CONFIDENCE_MAX_TICKS,
            # detenemos los motores: es mejor quedarse quieto que navegar con
            # pose XY desactualizada. El yaw sigue vivo (giroscopio) así que el
            # robot no pierde orientación; solo espera un match.
            # Modo MANUAL NO se bloquea — el usuario opera bajo su criterio.
            if (modo == MODE_RUTA
                    and pose_confidence_ticks > POSE_CONFIDENCE_MAX_TICKS):
                pl_cmd = 0.0
                pr_cmd = 0.0

            route_clean_active = (modo == MODE_RUTA and fsm_state in (FSM_ROTATE, FSM_ROTATE_SETTLE, FSM_ADVANCE, FSM_BLOCKED_ROTATE, FSM_WP_REACHED))
            auto_clean_logic_active = ((clean_sys and comp_on) or route_clean_active)
            auto_aux_enable = bool(auto_clean_logic_active and cam_aux_pwm > 0)
            pump_enable = auto_aux_enable or pump_override_on
            brush_enable = auto_aux_enable or brush_override_on
            aux_pwm = cam_aux_pwm if (pump_enable or brush_enable) else 0
            arduino.send_command(pl_cmd, pr_cmd, aux_pwm,
                                 pump_enable, brush_enable,
                                 relay_emergency_open)

            # ── Estimación de pose ─────────────────────────────────
            robot_moving = (abs(pl_cmd) >= PWM_VEL_THRESHOLD or
                            abs(pr_cmd) >= PWM_VEL_THRESHOLD)
            # Giro fuerte = PWM diferencial grande. Durante giros rápidos el
            # LiDAR barre con la plataforma en movimiento rotacional y la
            # nube se distorsiona; pausamos el match hasta que el giro acabe.
            turning_hard = abs(pl_cmd - pr_cmd) >= REF_MATCH_TURN_PAUSE_PWM

            # FASE 3: Scan matching XY + yaw contra el cuadrado de referencia.
            # Corre CADA TICK (cuando corresponde) y aplica corrección con
            # damping. El yaw ajusta saved_yaw_offset, no el yaw instantáneo;
            # el giroscopio sigue mandando el yaw en tiempo real.
            auto_match_counter += 1
            do_auto_match = (
                show_reference_square
                and reference_square_center_x is not None
                and auto_match_counter >= REF_MATCH_AUTO_EVERY_TICKS
                and not turning_hard
                and len(lidar_hits_global) >= REF_MATCH_MIN_PAIRS
            )
            if do_auto_match:
                auto_match_counter = 0
                _res = scan_match_full_against_square(
                    lidar_hits_global,
                    lidar_x_mm, lidar_y_mm,
                    reference_square_center_x,
                    reference_square_center_y,
                    REFERENCE_SQUARE_SIDE_MM,
                )
                if _res is not None:
                    # Aplicar corrección con damping (alpha < 1) para suavizar
                    # saltos y no oscilar al correr tick-a-tick
                    dx_apply = _res['dx'] * REF_XY_ALPHA
                    dy_apply = _res['dy'] * REF_XY_ALPHA
                    lidar_x_mm = clamp(lidar_x_mm + dx_apply,
                                        0.0, float(MAPA_ANCHO_MM))
                    lidar_y_mm = clamp(lidar_y_mm + dy_apply,
                                        0.0, float(MAPA_ALTO_MM))
                    if _res['yaw_confidence'] > 0.0:
                        dyaw_apply = _res['dyaw'] * REF_YAW_ALPHA
                        saved_yaw_offset = (
                            (saved_yaw_offset + dyaw_apply + 540.0) % 360.0 - 180.0
                        )
                    last_match_quality   = _res['quality']
                    last_match_corr_mm   = math.hypot(dx_apply, dy_apply)
                    last_pose_correction = last_match_corr_mm
                    pose_confidence_ticks = 0
                else:
                    last_match_quality = 0.0

            # Incrementar contador de incertidumbre de pose cada tick.
            # Se resetea a 0 cuando hay match exitoso (manual o auto).
            pose_confidence_ticks += 1
            pose_uncertainty_mm  = min(
                300.0,
                pose_confidence_ticks * 2.0   # crece 2mm por tick sin match
            )

            last_pl_cmd = pl_cmd
            last_pr_cmd = pr_cmd

            # ── Breadcrumbs: registrar trayectoria para retorno ─────
            # Solo acumular cuando el robot se mueve en modo ruta, para
            # que el trail represente el camino limpiado real.
            if modo == MODE_RUTA and (abs(pl) > 5 or abs(pr) > 5):
                breadcrumbs.update(lidar_x_mm, lidar_y_mm)

            # ── Mapa ───────────────────────────────────────────────
            if (((clean_sys and comp_on) or modo == MODE_RUTA) and (abs(pl) > 5 or abs(pr) > 5)):
                grid_map.mark_cleaned_disk(lidar_x_mm, lidar_y_mm, radius_mm=RADIO_LIMPIEZA_MM)

            # ── Detección de mapa corrupto (auto-F1) ──────────────
            # Si coverage > 95% pero la pose no cambia y el robot intenta
            # moverse → el mapa probablemente está mal georreferenciado.
            # Disparamos F1 automático como recuperación.
            if modo == MODE_RUTA:
                cov_now = grid_map.get_clean_coverage_pct()
                cmd_active = (abs(pl) > 30 or abs(pr) > 30)
                if (cov_now > MAP_CORRUPT_COV_MIN_PCT and cmd_active):
                    if map_corrupt_prev_x is not None:
                        pose_delta = math.hypot(
                            lidar_x_mm - map_corrupt_prev_x,
                            lidar_y_mm - map_corrupt_prev_y)
                        if pose_delta < MAP_CORRUPT_POSE_THR_MM:
                            map_corrupt_no_progress += 1
                        else:
                            map_corrupt_no_progress = 0
                    map_corrupt_prev_x = lidar_x_mm
                    map_corrupt_prev_y = lidar_y_mm

                    if map_corrupt_no_progress >= MAP_CORRUPT_NO_PROGRESS_TICKS:
                        print(f"[CORRUPT] Coverage {cov_now:.1f}% sin progreso "
                              f">{MAP_CORRUPT_NO_PROGRESS_TICKS} ticks → "
                              f"F1 automático")
                        # Disparar re-escaneo
                        grid_map.clear_walls()
                        scan_hit_counts.fill(0)
                        scan_start_time = now
                        wall_centers_xy = None
                        modo = MODE_SCAN
                        fsm_state = FSM_IDLE
                        pl, pr = 0.0, 0.0
                        arduino.send_command(0, 0, 0, False, False, relay_emergency_open)
                        map_corrupt_no_progress = 0
                else:
                    map_corrupt_no_progress = 0
                    map_corrupt_prev_x = None
                    map_corrupt_prev_y = None
            else:
                map_corrupt_no_progress = 0

            arduino_ok = arduino.is_connected()
            lidar_ok   = bool(lidar and lidar.running)

            # ── Autosave cuando hay cambios relevantes ─────────────
            if state_mgr.is_dirty(lidar_x_mm, lidar_y_mm, yaw_deg, grid_map):
                backup_mgr.rotate()   # rotar .bak antes de sobrescribir
                state_mgr.save({
                    "x_mm": lidar_x_mm, "y_mm": lidar_y_mm,
                    "yaw_deg": yaw_deg, "patron_actual": patron_actual,
                    "clean_threshold": clean_threshold,
                    "replan_count": replan_count,
                    "mission_home_x": mission_home_x,
                    "mission_home_y": mission_home_y,
                    "cam_dirt_ema": cam_dirt_ratio,
                    "grid_map": grid_map,
                    "nogo_zones": nogo_zones.to_list(),
                })

            # ── Session log (cada LOG_EVERY_N_TICKS) ──────────────
            log_tick_counter += 1
            if log_tick_counter >= LOG_EVERY_N_TICKS:
                log_tick_counter = 0
                session_log.append({
                    "timestamp":    datetime.datetime.now().isoformat(timespec="seconds"),
                    "x_mm":         round(lidar_x_mm, 1),
                    "y_mm":         round(lidar_y_mm, 1),
                    "yaw_deg":      round(yaw_deg, 2),
                    "modo":         modo,
                    "fsm":          fsm_state,
                    "coverage_pct": round(grid_map.get_clean_coverage_pct(), 2),
                    "dirt_ratio":   round(cam_dirt_ratio, 3),
                    "rear_dirt":    round(cam_rear_dirt_ratio, 3),
                    "cam_aux_pwm":  cam_aux_pwm,
                    "speed_scale":  round(speed_scale, 3),
                    "adaptive_paso":round(adaptive_paso_mm, 0),
                })
                # Rolling buffer: eliminar la entrada más antigua si se supera el límite
                if len(session_log) > SESSION_LOG_MAX:
                    session_log.pop(0)

            if (now - last_render_time) >= INTERVALO_DIBUJO:
                last_render_time = now

                screen.fill(COLOR_BG)

                grid_map.draw(screen, map_rect)
                # Cuadrado de referencia (diagnóstico). Se dibuja ANTES de
                # los hits para que la nube quede visible por encima y
                # puedas ver fácilmente si calza sobre los bordes del cuadrado.
                if show_reference_square and reference_square_center_x is not None:
                    grid_map.draw_reference_square(
                        screen, map_rect,
                        reference_square_center_x,
                        reference_square_center_y,
                        side_mm=REFERENCE_SQUARE_SIDE_MM,
                    )
                grid_map.draw_classified_hits(screen, map_rect,
                    dynamic_hits_render, static_hits_render)
                grid_map.draw_frontal_cone(screen, map_rect,
                    lidar_x_mm, lidar_y_mm, yaw_deg_draw, blocked=frontal_blocked)
                # ROI de cámara (rectángulo verde rotado delante del robot)
                if cam_roi_corners:
                    grid_map.draw_camera_roi(screen, map_rect,
                                             cam_roi_corners, cam_tile_label)
                    # ROI trasero (morado)
                    if cam_rear_roi_corners:
                        grid_map.draw_camera_roi(
                            screen, map_rect,
                            cam_rear_roi_corners, cam_rear_tile_label,
                            border_color=COLOR_REAR_ROI_BORDER,
                            fill_rgba=(180, 80, 255, 35))
                grid_map.draw_route(screen, map_rect, ruta_waypoints,
                                    waypoint_idx, fsm_state,
                                    skipped_indices=skipped_wp_indices)

                # Pose source para indicador visual en el robot
                _pose_src = (
                    "LIDAR+MATCH" if wall_centers_xy is not None
                    else "LIDAR+PWM"
                )
                # ── Dibujar zonas no-go ──────────────────────
                for (xmin, ymin, xmax, ymax) in nogo_zones.zones:
                    c1 = grid_map.world_to_screen(map_rect, xmin, ymin)
                    c2 = grid_map.world_to_screen(map_rect, xmax, ymin)
                    c3 = grid_map.world_to_screen(map_rect, xmax, ymax)
                    c4 = grid_map.world_to_screen(map_rect, xmin, ymax)
                    poly = [(int(c1[0]), int(c1[1])),
                             (int(c2[0]), int(c2[1])),
                             (int(c3[0]), int(c3[1])),
                             (int(c4[0]), int(c4[1]))]
                    try:
                        overlay = pygame.Surface(
                            (map_rect[2], map_rect[3]), pygame.SRCALPHA)
                        # Coords ajustadas al overlay
                        poly_adj = [(p[0] - map_rect[0], p[1] - map_rect[1])
                                    for p in poly]
                        pygame.draw.polygon(overlay, (255, 60, 60, 70), poly_adj)
                        pygame.draw.polygon(overlay, (255, 80, 80, 200),
                                             poly_adj, 2)
                        screen.blit(overlay, (map_rect[0], map_rect[1]))
                    except Exception:
                        pygame.draw.polygon(screen, (180, 40, 40), poly, 2)

                # ── Dibujar rectángulo de dibujo en curso ────
                if nogo_drawing and nogo_draw_start is not None:
                    try:
                        mx_cur, my_cur = pygame.mouse.get_pos()
                        if grid_map.is_inside_map_rect(map_rect, mx_cur, my_cur):
                            wx_cur, wy_cur = grid_map.screen_to_world(
                                map_rect, mx_cur, my_cur)
                            s1 = grid_map.world_to_screen(
                                map_rect, nogo_draw_start[0], nogo_draw_start[1])
                            s2 = grid_map.world_to_screen(
                                map_rect, wx_cur, nogo_draw_start[1])
                            s3 = grid_map.world_to_screen(
                                map_rect, wx_cur, wy_cur)
                            s4 = grid_map.world_to_screen(
                                map_rect, nogo_draw_start[0], wy_cur)
                            preview = [(int(s1[0]), int(s1[1])),
                                        (int(s2[0]), int(s2[1])),
                                        (int(s3[0]), int(s3[1])),
                                        (int(s4[0]), int(s4[1]))]
                            pygame.draw.polygon(screen, (255, 200, 80),
                                                 preview, 2)
                    except Exception:
                        pass

                # ── Obstáculos dinámicos simplificados ─────────────

                grid_map.draw_robot(screen, map_rect, lidar_x_mm, lidar_y_mm, yaw_deg_draw,
                                    alert=frontal_blocked,
                                    uncertainty_mm=pose_uncertainty_mm,
                                    speed_scale=speed_scale,
                                    pose_source=_pose_src,
                                    breadcrumbs=breadcrumbs)

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
                    "frontal_blocked": frontal_blocked,
                    "frontal_count": last_frontal_count,
                    "skipped_count": len(skipped_wp_indices),
                    "patron_actual":    patron_actual,
                    "clean_coverage":   grid_map.get_clean_coverage_pct(),
                    "clean_threshold":  clean_threshold,
                    "replan_count":     replan_count,
                    "cleaning_complete": cleaning_complete,
                    "pose_confidence_ticks": pose_confidence_ticks,
                    "pose_uncertain":       pose_confidence_ticks > POSE_CONFIDENCE_MAX_TICKS,
                    "in_stuck_recovery":  in_stuck_recovery,
                    "stuck_attempts":     wp_attempt_counts.get(waypoint_idx, 0),
                    "cleanable_count":  grid_map.get_cleanable_count(),
                    "uncleaned_count":  grid_map.get_uncleaned_cell_count(),
                    "dyn_count_total": dyn_count,
                    "clean_on": clean_sys,
                    "comp_on": comp_on,
                    "pump_override_on": pump_override_on,
                    "brush_override_on": brush_override_on,
                    "relay_emergency_open": relay_emergency_open,
                    "power": NIVELES_LIMPIEZA[pwr_idx],
                    "aux_pwm": aux_pwm,
                    "disc_pct": grid_map.get_discovered_pct(),
                    "clean_pct": grid_map.get_cleaned_pct(),
                    "last_dang": last_dang,
                    "arduino_ok": arduino_ok,
                    "lidar_ok": lidar_ok,
                    "rx_age_txt": rx_age_txt,
                    "lidar_rx_txt": lidar_rx_txt,
                    # ── Cámara ──
                    "cam_ok":       cam_ok,
                    "tile_label":   cam_tile_label,
                    "clean_ratio":  cam_clean_ratio,
                    "dirt_ratio":   cam_dirt_ratio,
                    "cam_aux_pwm":   cam_aux_pwm,
                    # Rear camera
                    "cam_rear_ok":        cam_rear_ok,
                    "rear_tile_label":    cam_rear_tile_label,
                    "rear_dirt_ratio":    cam_rear_dirt_ratio,
                    "rear_cells_reset":   cam_rear_cells_reset,
                    # Camera surfaces for live preview
                    "cam_front_surface":  cam_front_surf,
                    "cam_rear_surface":   cam_rear_surf,
                    # Pose source label for panel
                    "pose_source_label":  _pose_src,
                    # display_fsm (SLOWDOWN visible when slowing)
                    "display_fsm": (
                        FSM_SLOWDOWN_OBS if (fsm_state == FSM_ADVANCE and speed_scale < 0.99)
                        else FSM_RETURN_HOME if is_returning_home
                        else fsm_state),
                    # ── Fase 2 ──
                    "speed_scale":        speed_scale,
                    "nearest_dyn_dist":   nearest_dyn_dist if nearest_dyn_dist != float('inf') else -1,
                    "nearest_sta_dist":   nearest_sta_dist if nearest_sta_dist != float('inf') else -1,
                    "is_returning_home":  is_returning_home,
                    "mission_home_set":   mission_home_set,
                    "mission_home_x":     mission_home_x,
                    "mission_home_y":     mission_home_y,
                    "nogo_count":         len(nogo_zones.zones),
                    "nogo_drawing":       nogo_drawing,
                    "is_turning":         (abs(last_pl_cmd - last_pr_cmd)
                                            >= MATCH_TURN_PWM_DIFF_THR),
                    "turn_settle_left":   turn_settle_counter,
                    "yaw_offset":         saved_yaw_offset,
                    "adaptive_paso_mm":   adaptive_paso_mm,
                    "pose_uncertainty_mm": pose_uncertainty_mm,
                    "breadcrumb_count":   len(breadcrumbs.trail),
                    "home_path_len":      len(home_path),
                    "home_path_idx":      home_path_idx,
                }

                draw_panel(screen, font, small, state, expanded_sections)
                draw_toolbar(screen, font, small, state, dropdown_open=dropdown_open)
                if dropdown_open:
                    draw_controls_dropdown(screen, small)
                pygame.display.flip()

    except KeyboardInterrupt:
        pass

    finally:
        # Guardado final y marca de cierre limpio
        try:
            state_mgr.save({
                "x_mm":           lidar_x_mm,
                "y_mm":           lidar_y_mm,
                "yaw_deg":        yaw_deg,
                "patron_actual":  patron_actual,
                "clean_threshold":clean_threshold,
                "replan_count":   replan_count,
                "mission_home_x": mission_home_x,
                "mission_home_y": mission_home_y,
                "cam_dirt_ema":   cam_dirt_ratio,
                "grid_map":       grid_map,
            })
            state_mgr.mark_clean_exit()
            print("[StateManager] Estado guardado al cerrar.")
        except Exception as e:
            print(f"[StateManager] Error al guardar al cerrar: {e}")

        try:
            arduino.send_command(0, 0, 0, False, False, relay_emergency_open)
        except Exception:
            pass

        try:
            lidar.close()
        except Exception:
            pass

        try:
            camera.close()
        except Exception:
            pass

        try:
            camera_rear.close()
        except Exception:
            pass

        try:
            arduino.close()
        except Exception:
            pass

        pygame.quit()


if __name__ == "__main__":
    main()