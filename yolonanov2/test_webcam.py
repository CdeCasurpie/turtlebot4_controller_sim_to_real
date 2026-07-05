from ultralytics import YOLO
import cv2
import time

# 1. Carga del grafo cuantizado NCNN (apuntar al directorio descargado)
model = YOLO('/home/cesar/Descargas/turtlebot_signals_v2_oño/turtlebot_signals_v2_best_oño.pt', task='detect')

# 2. Inicialización del flujo de video (Ajustar ID según /dev/videoX del TurtleBot)
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret: break

    t0 = time.perf_counter()
    
    # 3. Paso forward (imprescindible forzar imgsz=416 para mantener O(1) en latencia)
    results = model(frame, imgsz=416, verbose=False)
    
    dt_ms = (time.perf_counter() - t0) * 1000

    # Procesamiento del tensor de salida
    for r in results:
        # Extraer cajas y clases
        boxes = r.boxes
        for box in boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            
            # Lógica de bifurcación de control
            if conf > 0.75: # Umbral de confianza
                if cls == 0:
                    print("Control: Rotación +w (Derecha)")
                elif cls == 1:
                    print("Control: Rotación -w (Izquierda)")
                elif cls == 2:
                    print("Control: Freno inercial (Stop)")
                elif cls == 3:
                    print("Control: Secuencia de Meta")

        # Visualización de telemetría (Remover en producción para ahorrar ciclos)
        annotated_frame = r.plot()
        cv2.putText(annotated_frame, f"Latencia Inferencia: {dt_ms:.1f}ms", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("TurtleBot Vision Pipeline", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
cv2.destroyAllWindows()