# Registro de Experimentos de Visión - TurtleBot 4 🐢👁️

Este documento es un registro detallado de **TODO** lo que se intentó, los problemas que enfrentamos y las soluciones que implementamos para lograr que el TurtleBot 4 viera las señales direccionales de forma autónoma.

---

## 1. El Problema Inicial: Modelo Personalizado YOLOv8
Entrenaste un modelo YOLOv8 Nano (`turtlebot_signals_v2_best.pt`) para detectar 4 clases:
- `left`
- `right`
- `stop`
- `finish`

El reto era **ejecutar este modelo dentro de la Raspberry Pi 4** del TurtleBot, la cual tiene recursos computacionales muy limitados, usando una cámara avanzada **OAK-D Lite**.

---

## 2. Intento 1: Uso del Chip VPU (Myriad X) de la OAK-D
**El objetivo:** Convertir el modelo `.pt` a un formato `.blob` para que el chip de IA interno de la cámara OAK-D hiciera todo el trabajo matemático (VPU), liberando al pobre procesador de la Raspberry Pi.

**Lo que hicimos:**
1. Renombramos temporalmente el modelo (por problemas de codificación SSH con la letra "ñ") a `custom_model.pt`.
2. Lo exportamos a ONNX y luego lo compilamos a `.blob` usando `blobconverter`.
3. Configuramos la API de `depthai` para inyectar este `.blob` directamente a la cámara.

**El Resultado y Problemas:**
- ❌ **Falla de Parsing (YoloDetectionNetwork):** El nodo nativo de DepthAI para YOLO no era compatible con la arquitectura de salida de YOLOv8.
- 🛠️ **Solución:** Tuvimos que crear un decodificador manual de tensores FP16 usando Numpy en `vpu_vision.py` (`np.max`, NMS, etc).
- ❌ **Falsos Positivos:** El modelo en VPU empezó a detectar fantasmas (detectaba "left" constantemente). Esto ocurrió probablemente por pérdida de precisión al comprimir a FP16 o un error en la decodificación manual de las cajas delimitadoras.

---

## 3. Problema Crítico: `X_LINK_DEVICE_ALREADY_IN_USE`
Durante nuestras pruebas con la cámara, empezamos a recibir este error fatal de DepthAI.

**La causa:**
Los contenedores de ROS 2 (`oakd_container` y `component_container`) que controlan el robot se inician automáticamente de fondo y se adueñan del puerto USB de la cámara.

**La Solución:**
Creamos un comando en tu `Makefile` para matar a los secuestradores antes de hacer pruebas de visión:
```bash
make stop_cam
```
*(O manualmente: `pkill -f oakd_container ; pkill -f component_container`)*

---

## 4. Intento 2: Inferencia en CPU Pura (PyTorch)
Ante los problemas matemáticos del VPU, decidimos volver a lo seguro: usar tu modelo original `.pt` ejecutado por la CPU de la Raspberry Pi usando la librería oficial `ultralytics`.

**El Resultado y Problemas:**
- ✅ **Precisión perfecta:** Ya no había fantasmas, detectaba todo correctamente.
- ❌ **Lag Extremo:** La Raspberry Pi tardaba muchísimo en procesar un frame (iba casi a 1 FPS), haciendo imposible usarlo en tiempo real. Adicionalmente, tener una ventana (`cv2.imshow`) abierta enviando video por SSH (X11) asfixiaba aún más la red.

**La Solución Híbrida (Optimización):**
Para salvar la CPU, modificamos `vpu_vision.py` para:
1. Apagar todas las ventanas visuales.
2. Extraer el frame RGB usando la OAK-D y **reducir su tamaño a 320x320 píxeles** (`cv2.resize(frame, (320, 320))`) justo antes de dárselo a YOLO.
3. Esto aceleró la detección X4 veces, permitiendo un uso decente en el robot real.

---

## 5. El Pivot Final: El Detector de Códigos QR (WeChat Engine)
Ante las dificultades de YOLO, sugeriste usar una alternativa hiperrobusta: códigos QR usando el motor avanzado de WeChat de OpenCV.

**Lo que hicimos:**
1. Arreglamos un `IndentationError` en `turtlebot.py` (Línea 200) que impedía correr el robot con el flag `--no-yolo`.
2. Creamos `qr_detector.py`. En vez de usar `cv2.VideoCapture(0)` (lo cual falla con la OAK-D por no ser una simple webcam USB PnP), lo conectamos a `depthai` para extraer el video crudo y pasarlo por WeChat.
3. ❌ **Error de dependencias:** Faltaba el módulo contrib.
4. ✅ **Solución:** Entramos al TurtleBot e instalamos `opencv-contrib-python` dentro del entorno virtual, reviviendo el escáner exitosamente.

---

## 6. Estado Actual del Repositorio Local
Justo antes de que el TurtleBot se desconectara, todo el código (incluyendo scripts que creamos allá) fue salvado localmente en tu laptop y añadido a un nuevo branch en Git.

**Archivos añadidos/modificados en local:**
- `TurtleBotController/vpu_vision.py`: Contiene la lógica híbrida CPU súper rápida para YOLO a 320x320.
- `TurtleBotController/turtlebot.py`: Código reparado (IndentationError resuelto) listo para funcionar con y sin YOLO.
- `TurtleBotController/config.json`: Umbral de confianza adaptado.
- `qr_detector.py`: El súper script escáner de QR integrado con OAK-D.
- `test_vision.py`: Recreado localmente para que puedas probar la visión sin mover al robot.

Todo esto está empaquetado y comiteado en tu rama **`experimental-vision`**.
