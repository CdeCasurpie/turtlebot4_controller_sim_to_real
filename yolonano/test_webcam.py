import cv2
from ultralytics import YOLO

def main():
    # Load the YOLO model
    print("Cargando el modelo...")
    try:
        model = YOLO("best.pt")
    except Exception as e:
        print(f"Error al cargar el modelo: {e}")
        return

    # Open the webcam (usually 0 is the built-in webcam)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: No se pudo abrir la cámara.")
        return

    print("Cámara abierta. Presiona 'q' para salir.")

    # Crear la ventana antes del bucle y evitar acentos en el nombre
    cv2.namedWindow("YOLO_Detections", cv2.WINDOW_NORMAL)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("No se pudo leer el frame de la cámara. Saliendo...")
            break

        # Run inference on the frame (only confidence > 60%)
        results = model(frame, verbose=False, conf=0.85)

        # Draw the results on the frame
        annotated_frame = results[0].plot()

        # Display the frame
        cv2.imshow("YOLO_Detections", annotated_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
