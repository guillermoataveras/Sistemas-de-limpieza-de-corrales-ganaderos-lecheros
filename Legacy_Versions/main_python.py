import cv2
from camera_analysis import CameraAnalyzer


def imprimir_resultados(resultados):
    if resultados is None:
        print("No hay datos aún.")
        return

    cam1 = resultados["camara_1"]
    cam2 = resultados["camara_2"]

    print("\n=== Porcentajes actuales ===")

    print("\nWebcam 1:")
    print(f"Blanco: {cam1['blanco']:.2f}%")
    print(f"Negro:  {cam1['negro']:.2f}%")
    print(f"Otros:  {cam1['otros']:.2f}%")

    print("\nWebcam 2:")
    print(f"Blanco: {cam2['blanco']:.2f}%")
    print(f"Negro:  {cam2['negro']:.2f}%")
    print(f"Otros:  {cam2['otros']:.2f}%")


def main():
    analyzer = None

    try:
        analyzer = CameraAnalyzer(cam_index_1=0, cam_index_2=1)

        cv2.namedWindow("Webcam 1", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Webcam 2", cv2.WINDOW_NORMAL)

        while True:
            frame1, frame2 = analyzer.leer_y_analizar()

            cv2.imshow("Webcam 1", frame1)
            cv2.imshow("Webcam 2", frame2)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('r'):
                print("\nImprimiendo Webcam 1 ahora y Webcam 2 después del desfase...")
                resultados = analyzer.obtener_porcentajes_actuales_con_desfase()
                imprimir_resultados(resultados)

            if key == 27 or key == ord('q'):
                break

            if cv2.getWindowProperty("Webcam 1", cv2.WND_PROP_VISIBLE) < 1:
                break
            if cv2.getWindowProperty("Webcam 2", cv2.WND_PROP_VISIBLE) < 1:
                break

    except RuntimeError as e:
        print(e)

    finally:
        if analyzer is not None:
            analyzer.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()