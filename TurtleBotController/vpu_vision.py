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
        if dai is None or not hasattr(dai.node, "YoloDetectionNetwork"):
            # dai.node.YoloDetectionNetwork/XLinkIn/XLinkOut no existen en depthai 3.x (API
            # rediseñada); este pipeline necesita la rama 2.x.
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

        detection_nn = pipeline.create(dai.node.YoloDetectionNetwork)
        detection_nn.setBlobPath(str(blob_path))
        detection_nn.setConfidenceThreshold(confidence_threshold)
        detection_nn.setNumClasses(num_classes)
        detection_nn.setCoordinateSize(COORDINATE_SIZE)
        detection_nn.setAnchors([])
        detection_nn.setAnchorMasks({})
        detection_nn.setIouThreshold(iou_threshold)
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
            parsed = [self._to_detection(d) for d in in_nn.detections]
            with self._lock:
                self._latest = parsed

    def _to_detection(self, det) -> Dict:
        # Bbox normalizado [0,1] -> mismo cálculo de ángulo/distancia que usaba
        # la versión por CPU, pero con el ancho fijo del preview (640) como "foco".
        cx = (det.xmin + det.xmax) / 2.0
        normalized_x = (cx - 0.5) * 2.0  # -1 (izq) a 1 (der)
        relative_angle = -normalized_x * (self.camera_fov / 2.0)

        bbox_width_px = max((det.xmax - det.xmin) * INPUT_SIZE[0], 1.0)
        focal_length = INPUT_SIZE[0]
        distance = (self.real_sign_width_m * focal_length) / bbox_width_px

        label = self.class_names[det.label] if det.label < len(self.class_names) else str(det.label)
        return {'class': label, 'distance': distance, 'relative_angle': relative_angle}

    def get_detections(self) -> List[Dict]:
        with self._lock:
            return list(self._latest)

    def close(self):
        self._running = False
        self.device.close()
        self._thread.join(timeout=1.0)
