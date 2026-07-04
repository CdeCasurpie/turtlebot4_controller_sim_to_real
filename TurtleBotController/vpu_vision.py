"""
Detector de señales YOLO corriendo en la VPU (Myriad X) de la OAK-D Lite vía DepthAI.

Reemplaza la inferencia por CPU (`ultralytics`, sobre frames recibidos por un tópico
ROS) que usaba antes `TurtleBotReal`: la `ColorCamera` y el `YoloDetectionNetwork`
corren enteramente dentro del chip OAK-D; este módulo solo lee resultados ya resueltos
(clase + confianza + bbox) desde la cola de salida `nn`, igual que el sandbox validado
en `vpu_deployment/test_depthai_yolo.py` (modo `run_headless`).
"""
import threading
import time
import numpy as np
import cv2
from pathlib import Path
from typing import List, Dict

try:
    import depthai as dai
except ImportError:
    dai = None

INPUT_SIZE = (640, 640)  # debe coincidir con el imgsz usado al exportar el ONNX -> blob
COORDINATE_SIZE = 4  # cx, cy, w, h (YOLOv8 es anchor-free: sin objectness ni anclas)


def _load_class_names(path: Path, num_classes: int) -> List[str]:
    if not path.exists():
        raise FileNotFoundError(f"No se encontró el archivo de clases en {path}.")
    names = [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if len(names) != num_classes:
        raise ValueError(f"{path} tiene {len(names)} clases pero se esperaban {num_classes}.")
    return names


class VpuYoloDetector:
    """
    Corre YOLO en la VPU de la OAK-D y expone la última tanda de detecciones ya
    convertidas al mismo formato que espera la máquina de estados de navegación:
    [{'class': str, 'distance': float, 'relative_angle': float}, ...]
    """

    def __init__(self, blob_path, classes_path, num_classes: int, confidence_threshold: float,
                 iou_threshold: float, fps: int, camera_fov_rad: float, real_sign_width_m: float = 0.20):
        if dai is None or not hasattr(dai.node, "NeuralNetwork"):
            instalado = "no instalado" if dai is None else f"instalado, versión {getattr(dai, '__version__', '?')} (rama incompatible)"
            raise RuntimeError(
                f"'depthai' {instalado}. Se requiere la rama 2.x: pip install \"depthai<3\""
            )

        self.class_names = _load_class_names(Path(classes_path), num_classes)
        self.camera_fov = camera_fov_rad
        self.real_sign_width_m = real_sign_width_m

        blob_path = Path(blob_path)
        if not blob_path.exists():
            raise FileNotFoundError(f"No se encontró el modelo .blob en {blob_path}.")

        pipeline = dai.Pipeline()

        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(*INPUT_SIZE)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(fps)

        self.conf_threshold = confidence_threshold
        self.iou_threshold = iou_threshold

        detection_nn = pipeline.create(dai.node.NeuralNetwork)
        detection_nn.setBlobPath(str(blob_path))
        detection_nn.setNumInferenceThreads(2)
        detection_nn.input.setBlocking(False)
        cam_rgb.preview.link(detection_nn.input)

        xout_nn = pipeline.create(dai.node.XLinkOut)
        xout_nn.setStreamName("nn")
        detection_nn.out.link(xout_nn.input)

        self.device = dai.Device(pipeline)
        self.q_nn = self.device.getOutputQueue(name="nn", maxSize=4, blocking=True)

        self._lock = threading.Lock()
        self._latest: List[Dict] = []
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def _poll_loop(self):
        # `.get()` bloqueante: el hilo queda dormido esperando a la VPU en vez de
        # consumir CPU de la Raspberry Pi en un loop activo.
        while self._running:
            try:
                in_nn = self.q_nn.get()
            except RuntimeError:
                break  # el dispositivo se cerró (ver close())
            if in_nn is None:
                continue
            
            # YOLOv8: Output tensor shape is typically [1, 4+num_classes, 8400]
            layer_names = in_nn.getAllLayerNames()
            if not layer_names:
                continue
            data = np.array(in_nn.getLayerFp16(layer_names[0]))
            
            # Reshape a (4+classes, 8400) y transponer a (8400, 4+classes)
            num_classes = len(self.class_names)
            try:
                data = data.reshape(4 + num_classes, -1).T
            except ValueError:
                continue
                
            scores = np.max(data[:, 4:], axis=1)
            classes = np.argmax(data[:, 4:], axis=1)
            
            mask = scores > self.conf_threshold
            filtered_data = data[mask]
            filtered_scores = scores[mask]
            filtered_classes = classes[mask]
            
            boxes = []
            confidences = []
            class_ids = []
            
            for i in range(len(filtered_data)):
                cx, cy, w, h = filtered_data[i, 0:4]
                # Convertir de pixeles absolutos (0-640) a normalizados (0-1) si es necesario.
                # Si YOLOv8 da salida ya normalizada, esto será menor a 1 y no afectará si max() recorta, 
                # pero usualmente YOLOv8 crudo da [0, imgsz]
                if cx > 2.0 or w > 2.0:
                    cx /= INPUT_SIZE[0]
                    cy /= INPUT_SIZE[1]
                    w /= INPUT_SIZE[0]
                    h /= INPUT_SIZE[1]
                
                xmin = cx - w/2
                ymin = cy - h/2
                boxes.append([float(xmin), float(ymin), float(w), float(h)])
                confidences.append(float(filtered_scores[i]))
                class_ids.append(int(filtered_classes[i]))
                
            parsed = []
            if len(boxes) > 0:
                indices = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_threshold, self.iou_threshold)
                if len(indices) > 0:
                    for i in indices.flatten():
                        xmin, ymin, w, h = boxes[i]
                        parsed.append(self._to_detection(xmin, xmin+w, class_ids[i]))
                        
            with self._lock:
                self._latest = parsed

    def _to_detection(self, xmin, xmax, label_idx) -> Dict:
        # Bbox normalizado [0,1]
        cx = (xmin + xmax) / 2.0
        normalized_x = (cx - 0.5) * 2.0  # -1 (izq) a 1 (der)
        relative_angle = -normalized_x * (self.camera_fov / 2.0)

        bbox_width_px = max((xmax - xmin) * INPUT_SIZE[0], 1.0)
        focal_length = INPUT_SIZE[0]
        distance = (self.real_sign_width_m * focal_length) / bbox_width_px

        label = self.class_names[label_idx] if label_idx < len(self.class_names) else str(label_idx)
        return {'class': label, 'distance': distance, 'relative_angle': relative_angle}

    def get_detections(self) -> List[Dict]:
        with self._lock:
            return list(self._latest)

    def close(self):
        self._running = False
        self.device.close()
        self._thread.join(timeout=1.0)
