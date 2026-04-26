import cv2
import numpy as np
import time
from collections import deque


class CameraConfig:
    def __init__(self):
        # Configuración manual
        self.AUTOFOCUS = 0
        self.AUTO_EXPOSURE = 0
        self.EXPOSURE = -5

        # Resolución de captura
        self.FRAME_WIDTH = 640
        self.FRAME_HEIGHT = 480

        # Resolución de procesamiento
        self.PROCESS_WIDTH = 160
        self.PROCESS_HEIGHT = 120

        # Umbrales HSV
        self.WHITE_S_MAX = 50
        self.WHITE_V_MIN = 90
        self.BLACK_V_MAX = 50

        # Blur
        self.USE_BLUR = True
        self.BLUR_KERNEL = (5, 5)

        # Historial
        self.MAX_SAMPLES = 100

        # Desfase temporal para cámara 2 al imprimir
        self.DELAY_CAM2_SECONDS = 3


def configurar_camara(cap, config: CameraConfig):
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, config.AUTOFOCUS)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, config.AUTO_EXPOSURE)
    cap.set(cv2.CAP_PROP_EXPOSURE, config.EXPOSURE)


def analizar_frame(frame, config: CameraConfig):
    small = cv2.resize(
        frame,
        (config.PROCESS_WIDTH, config.PROCESS_HEIGHT),
        interpolation=cv2.INTER_AREA
    )

    if config.USE_BLUR:
        small = cv2.GaussianBlur(small, config.BLUR_KERNEL, 0)

    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)

    s = hsv[:, :, 1]
    v = hsv[:, :, 2]

    mask_blanco = (s < config.WHITE_S_MAX) & (v > config.WHITE_V_MIN)
    mask_negro = v < config.BLACK_V_MAX
    mask_otros = ~(mask_blanco | mask_negro)

    total_pixeles = small.shape[0] * small.shape[1]

    porcentaje_blanco = np.count_nonzero(mask_blanco) / total_pixeles * 100
    porcentaje_negro = np.count_nonzero(mask_negro) / total_pixeles * 100
    porcentaje_otros = np.count_nonzero(mask_otros) / total_pixeles * 100

    return porcentaje_blanco, porcentaje_negro, porcentaje_otros


def dibujar_texto(frame, titulo, blanco, negro, otros):
    y0 = 30
    dy = 30

    cv2.putText(frame, titulo, (10, y0),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    cv2.putText(frame, f"Blanco: {blanco:.1f}%", (10, y0 + dy),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    cv2.putText(frame, f"Negro: {negro:.1f}%", (10, y0 + 2 * dy),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (50, 50, 50), 2)

    cv2.putText(frame, f"Otros: {otros:.1f}%", (10, y0 + 3 * dy),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)


class CameraAnalyzer:
    def __init__(self, cam_index_1=0, cam_index_2=1):
        self.config = CameraConfig()

        self.cap1 = cv2.VideoCapture(cam_index_1, cv2.CAP_DSHOW)
        self.cap2 = cv2.VideoCapture(cam_index_2, cv2.CAP_DSHOW)

        if not self.cap1.isOpened() or not self.cap2.isOpened():
            if self.cap1.isOpened():
                self.cap1.release()
            if self.cap2.isOpened():
                self.cap2.release()
            raise RuntimeError("Error abriendo cámaras")

        configurar_camara(self.cap1, self.config)
        configurar_camara(self.cap2, self.config)

        # Arrays cámara 1
        self.hist_blanco_1 = deque(maxlen=self.config.MAX_SAMPLES)
        self.hist_negro_1 = deque(maxlen=self.config.MAX_SAMPLES)
        self.hist_otros_1 = deque(maxlen=self.config.MAX_SAMPLES)

        # Arrays cámara 2
        self.hist_blanco_2 = deque(maxlen=self.config.MAX_SAMPLES)
        self.hist_negro_2 = deque(maxlen=self.config.MAX_SAMPLES)
        self.hist_otros_2 = deque(maxlen=self.config.MAX_SAMPLES)

    def leer_y_analizar(self):
        ret1, frame1 = self.cap1.read()
        ret2, frame2 = self.cap2.read()

        if not ret1 or frame1 is None:
            raise RuntimeError("Error leyendo frames de la cámara 1")

        if not ret2 or frame2 is None:
            raise RuntimeError("Error leyendo frames de la cámara 2")

        blanco1, negro1, otros1 = analizar_frame(frame1, self.config)
        blanco2, negro2, otros2 = analizar_frame(frame2, self.config)

        self.hist_blanco_1.append(blanco1)
        self.hist_negro_1.append(negro1)
        self.hist_otros_1.append(otros1)

        self.hist_blanco_2.append(blanco2)
        self.hist_negro_2.append(negro2)
        self.hist_otros_2.append(otros2)

        dibujar_texto(frame1, "Webcam 1", blanco1, negro1, otros1)
        dibujar_texto(frame2, "Webcam 2", blanco2, negro2, otros2)

        return frame1, frame2

    def obtener_porcentajes_actuales_con_desfase(self):
        if len(self.hist_blanco_1) == 0 or len(self.hist_blanco_2) == 0:
            return None

        resultados = {
            "camara_1": {
                "blanco": self.hist_blanco_1[-1],
                "negro": self.hist_negro_1[-1],
                "otros": self.hist_otros_1[-1]
            }
        }

        time.sleep(self.config.DELAY_CAM2_SECONDS)

        resultados["camara_2"] = {
            "blanco": self.hist_blanco_2[-1],
            "negro": self.hist_negro_2[-1],
            "otros": self.hist_otros_2[-1]
        }

        return resultados

    def release(self):
        self.cap1.release()
        self.cap2.release()