# Registro Cronológico de Experimentos de Visión - TurtleBot 4 🐢👁️

Este documento reconstruye de principio a fin todos los pasos, comandos, dolores de cabeza y victorias técnicas que tuvimos a lo largo del proyecto intentando dotar al TurtleBot 4 de la capacidad de leer señales. 

---

## 1. La Odisea de YOLOv8 y la VPU (Myriad X)
Todo comenzó con un modelo personalizado YOLOv8 Nano entrenado para 4 clases: `left`, `right`, `stop`, `finish`. Queríamos que el TurtleBot imprimiera qué flechas veía, hiciera tracking y girara a verlas.

1. **Problema con la "Ñ":** Intentamos subir el modelo original (`turtlebot_signals_v2_best_oño.pt`). El servidor de compilación colapsó por el carácter "ñ", así que lo renombramos a `custom_model.pt`.
2. **Conversión a Blob:** Usamos `blobconverter` para pasarlo a formato `.blob` e inyectarlo en el chip VPU de la cámara OAK-D Lite.
3. **El nodo DepthAI falló:** El nodo `YoloDetectionNetwork` nativo de la cámara no soportaba la arquitectura de YOLOv8. 
4. **Solución manual:** Tuvimos que crear `vpu_vision.py` con una red neuronal plana (`dai.node.NeuralNetwork`) y parsear los tensores FP16 manualmente usando Numpy y `cv2.dnn.NMSBoxes`.
5. **Resultado:** El modelo compilado arrojaba "falsos positivos" horribles. Constantemente veía la señal "left" incluso cuando no había nada enfrente de él. La conversión a FP16 arruinó la precisión del modelo original.

## 2. El Secuestro de la Cámara y el X11
Entre pruebas, la cámara OAK-D se negaba a inicializar arrojando el error: `RuntimeError: Failed to connect to device, error message: X_LINK_DEVICE_ALREADY_IN_USE`.

1. **La causa:** Al prender el TurtleBot, los nodos internos de ROS (`oakd_container` y `component_container`) inician de fondo y toman el control exclusivo del USB.
2. **Solución:** Creamos una regla en el `Makefile` (`make stop_cam`) para aniquilar los contenedores de ROS antes de hacer pruebas de visión. (También tuvimos un caso donde un script de python atascado, el PID 9927, secuestró el puerto y tuvimos que matarlo con `kill -9`).
3. **El problema visual (X11):** Para depurar qué estaba viendo la cámara, enviamos el video a la laptop usando SSH (X11 Forwarding). Esto causó un cuello de botella en la red tan masivo que el video iba a menos de 1 FPS y pensamos que el modelo estaba fallando. Cuando quitamos la UI (`cv2.imshow`), recuperamos la velocidad.

## 3. El Salto a CPU (PyTorch puro)
Debido a que el `.blob` daba falsos positivos, el usuario (César) dijo: *"y si lo hacemos defrente en cpu¿ y ya no como blob¿... no imorta ya, con los hilos que tenga o aprovechando cada cosa... pero quiero usarlo ya con el oak"*.

1. Implementamos `ultralytics` para correr el modelo original `.pt` usando la CPU de la Raspberry Pi 4.
2. **Problema:** En la laptop (Intel i7) el `.pt` volaba, pero en la Raspberry Pi iba terriblemente lento (más de 1 segundo por frame).
3. **Solución:** Modificamos la tubería para que capture la imagen a 640x640 pero la reduzca drásticamente a **320x320** justo antes de la inferencia. Esto multiplicó por 4 la velocidad en la CPU, haciéndolo finalmente viable.

## 4. El Pivot Definitivo: Códigos QR (WeChat Engine)
Tras luchar con la imprecisión y pesadez de YOLO, el usuario propuso una alternativa magistral: usar un lector de códigos QR basado en el motor de WeChat.

1. **Adaptación de la OAK-D:** El script original usaba `cv2.VideoCapture(0)`, pero eso casi nunca funciona con la OAK-D. Adaptamos el script (`qr_detector.py`) para usar la librería `depthai` y jalar el flujo RGB directamente.
2. **Falta de dependencias:** Faltaba el módulo `cv2.wechat_qrcode_WeChatQRCode`.
3. **Solución:** Entramos al entorno virtual del robot (`~/venv`) e instalamos el paquete pesado `opencv-contrib-python`.
4. **Resultado:** El escáner funcionó maravillosamente sin ventanas gráficas y reportando a la consola a la velocidad de la luz.

## 5. Otros Arreglos en el Camino
* Reparamos un `IndentationError` en la línea 200 de `turtlebot.py` que impedía correr el controlador autónomo cuando el flag `--no-yolo` estaba activado.
* Respaldamos todo el código (incluyendo modificaciones directas hechas por SSH) localmente en la laptop y lo subimos a la rama de Git `experimental-vision`.

---

## 6. ¿QUÉ FALTA POR HACER? (TO-DO)
Actualmente el proyecto está en un estado híbrido estable. Para completar el ciclo autónomo, quedan los siguientes pasos pendientes:

- [ ] **Integrar QR al Controlador Autónomo:** Mudar la lógica de `qr_detector.py` hacia dentro de `turtlebot.py` (posiblemente renombrando `VpuYoloDetector` a `QRVisionDetector`) para que la clase `TurtleBotReal` pueda consumir las direcciones de los QR.
- [ ] **Mapear las respuestas del QR:** Asegurar que cuando el lector WeChat decodifique el string del QR (ej. "LEFT"), el diccionario de salida del robot coincida con lo que espera `test_autonomous_controller.py` (`{'class': 'left', 'distance': ..., 'relative_angle': ...}`).
- [ ] **Calcular distancia al QR:** Dado que un QR no da profundidad per se con una sola cámara RGB, usar la distancia focal de la cámara (FOV) y el ancho en píxeles del QR detectado para estimar a qué distancia está el cartel, y calcular su ángulo relativo para que el robot sepa cuánto debe girar para centrarlo.
- [ ] **Pruebas de Campo Finales:** Encender el controlador, poner carteles QR físicos, y verificar que el algoritmo de navegación pase del estado `EXPLORANDO` a `ACERCANDOSE_A_SENAL`, evada obstáculos, y luego ejecute `GIRANDO_IZQ` o `DETENIDO` con éxito.
