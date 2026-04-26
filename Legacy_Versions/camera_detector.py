import cv2          # Librería OpenCV para visión por computadora
import numpy as np  # Librería para operaciones numéricas eficientes

class DetectorColor:
    def __init__(self):

        # Índices de cámara a probar (0 = principal, 1 = secundaria)
        self.CAMERA_INDEX_PRIMARY = 0
        self.CAMERA_INDEX_SECONDARY = 1

        # Backend de captura (DSHOW funciona mejor en Windows generalmente)
        self.CAMERA_BACKEND = cv2.CAP_DSHOW
        # Alternativa:
        # self.CAMERA_BACKEND = cv2.CAP_MSMF

        # Configuración de la cámara (control manual)
        self.AUTOFOCUS = 0          # 0 = desactivado
        self.AUTO_EXPOSURE = 0      # 0 = manual
        self.EXPOSURE = -4          # Valor de exposición (depende de la cámara)

        # Resolución de captura (lo que entrega la cámara)
        self.FRAME_WIDTH = 640
        self.FRAME_HEIGHT = 480

        # Resolución de procesamiento (para acelerar análisis)
        self.PROCESS_WIDTH = 160
        self.PROCESS_HEIGHT = 120

        # Umbrales para detectar colores en HSV
        self.WHITE_S_MAX = 50       # Blanco = baja saturación
        self.WHITE_V_MIN = 90       # y alto brillo

        self.BLACK_V_MAX = 50       # Negro = bajo brillo

        # Activar prints de debug
        self.DEBUG = False

        # Inicialización de la cámara
        self.cap = None
        self.abrir_camara()

    def abrir_camara(self):
        # Intenta abrir la cámara principal
        self.cap = cv2.VideoCapture(self.CAMERA_INDEX_PRIMARY, self.CAMERA_BACKEND)

        if not self.cap.isOpened():
            # Si falla, intenta con la secundaria
            self.cap = cv2.VideoCapture(self.CAMERA_INDEX_SECONDARY, self.CAMERA_BACKEND)

            if not self.cap.isOpened():
                # Si ninguna funciona, lanza error
                raise Exception("ERROR: No se encuentra ninguna cámara.")

        # Configura parámetros de la cámara
        self.configurar_camara()

        # Mensajes de debug
        if self.DEBUG:
            print("Cámara abierta correctamente.")
            print("Resolución configurada:",
                  self.cap.get(cv2.CAP_PROP_FRAME_WIDTH), "x",
                  self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    def configurar_camara(self):
        # Configura resolución
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.FRAME_HEIGHT)

        # Configura enfoque y exposición
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, self.AUTOFOCUS)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, self.AUTO_EXPOSURE)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, self.EXPOSURE)

    def obtener_frame(self):
        # Captura un frame
        ret, frame = self.cap.read()

        if not ret or frame is None:
            if self.DEBUG:
                print("WARNING: No se pudo leer frame.")
            return None

        return frame

    def analizar_frame(self, frame):
        # Reduce resolución para acelerar procesamiento
        small = cv2.resize(
            frame,
            (self.PROCESS_WIDTH, self.PROCESS_HEIGHT),
            interpolation=cv2.INTER_AREA
        )
        
        # Suavizado para reducir ruido (opcional pero recomendado)
        small = cv2.GaussianBlur(small, (5, 5), 0)
        
        # Convierte de BGR (OpenCV) a HSV
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)

        # Extrae canales Saturación (S) y Valor (V)
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]

        # Clasificación de píxeles
        mask_blanco = (s < self.WHITE_S_MAX) & (v > self.WHITE_V_MIN)
        mask_negro = v < self.BLACK_V_MAX
        mask_otros = ~(mask_blanco | mask_negro)

        # Total de píxeles analizados
        total_pixeles = small.shape[0] * small.shape[1]

        # Cálculo de porcentajes
        porcentaje_blanco = np.count_nonzero(mask_blanco) / total_pixeles * 100
        porcentaje_negro = np.count_nonzero(mask_negro) / total_pixeles * 100
        porcentaje_otros = np.count_nonzero(mask_otros) / total_pixeles * 100

        return porcentaje_blanco, porcentaje_negro, porcentaje_otros

    def release(self):
        # Libera la cámara
        if self.cap is not None:
            self.cap.release()