camera_detector.py

Clase: DetectorColor
Descripción:
    Esta clase abre una cámara, captura frames y permite analizar
    el porcentaje de píxeles blancos, negros y otros colores
    dentro de una imagen capturada.

--------------------------------------------------
INSTRUCCIONES DE USO EN UN ARCHIVO main CUALQUIERA
--------------------------------------------------

1) Importa la clase:
    from camera_test import DetectorColor

2) Crea una instancia:
    detector = DetectorColor()

3) Obtén frames con:
    frame = detector.obtener_frame()

4) Analiza un frame con:
    blanco, negro, otros = detector.analizar_frame(frame)

5) Libera la cámara al terminar:
    detector.release()

--------------------------------------------------
EJEMPLO DE USO
--------------------------------------------------

import cv2
from camera_test import DetectorColor

def main():
    detector = DetectorColor()

    try:
        while True:
            frame = detector.obtener_frame()

            if frame is None:
                print("No se recibió frame.")
                break

            cv2.imshow("Camara", frame)

            key = cv2.waitKey(1) & 0xFF

            if key == 27:  # ESC
                break

            elif key == ord('r'):
                blanco, negro, otros = detector.analizar_frame(frame)
                print(
                    f"Blanco: {blanco:.2f}% | "
                    f"Negro: {negro:.2f}% | "
                    f"Otros: {otros:.2f}%"
                )

    finally:
        detector.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

--------------------------------------------------
NOTAS
--------------------------------------------------

- Si no se encuentra la cámara principal, la clase intentará abrir
  la cámara secundaria.
- La función analizar_frame(frame) NO captura imágenes por sí sola.
  Recibe un frame ya capturado.
- Si quieres analizar el frame actual al presionar una tecla,
  primero debes obtener el frame y luego pasarlo a analizar_frame().
- Siempre libera la cámara con detector.release() al finalizar.