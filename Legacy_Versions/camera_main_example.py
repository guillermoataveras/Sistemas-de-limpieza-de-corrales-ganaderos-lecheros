import cv2
from camera_detector import DetectorColor  # Importa la clase que controla la cámara y analiza colores

# Crea una instancia del detector.
# En este punto se intenta abrir y configurar la cámara automáticamente.
detector = DetectorColor()

try:
    while True:
        # Captura un frame desde la cámara
        frame = detector.obtener_frame()

        # Si no se pudo obtener el frame, termina el programa
        if frame is None:
            print("No se recibió frame.")
            break

        # Muestra el frame en una ventana llamada "Camara"
        cv2.imshow("Camara", frame)

        # Verifica si la ventana fue cerrada manualmente por el usuario
        if cv2.getWindowProperty("Camara", cv2.WND_PROP_VISIBLE) < 1:
            print("Ventana cerrada.")
            break

        # Espera 1 ms por una tecla y obtiene su código
        key = cv2.waitKey(1) & 0xFF

        # Si se presiona ESC, salir del bucle
        if key == 27:  # ESC
            print("Saliendo.")
            break

        # Si se presiona la tecla 'r', analiza el frame actual
        elif key == ord('r'):
            # Calcula el porcentaje de píxeles blancos, negros y otros
            blanco, negro, otros = detector.analizar_frame(frame)

            # Imprime los resultados en consola con dos decimales
            print(
                f"Blanco: {blanco:.2f}% | "
                f"Negro: {negro:.2f}% | "
                f"Otros: {otros:.2f}%"
            )

# Permite cerrar el programa con Ctrl + C sin que explote feo
except KeyboardInterrupt:
    print("Programa interrumpido por teclado.")

finally:
    # Libera la cámara pase lo que pase
    detector.release()

    # Cierra todas las ventanas de OpenCV
    cv2.destroyAllWindows()

    print("Cámara liberada correctamente.")