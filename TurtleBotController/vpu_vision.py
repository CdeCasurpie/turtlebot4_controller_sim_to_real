import threading
import time
import cv2
from pathlib import Path
from typing import List, Dict

try:
    import depthai as dai
except ImportError:
    dai = None

from ultralytics import YOLO

INPUT_SIZE = (640, 640)

def _load_class_names(path: Path, num_classes: int) -> List[str]:
    if not path.exists():
        return [str(i) for i in range(num_classes)]
    names = [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    return names

class VpuYoloDetector:
    def __init__(self, pt_path, classes_path, num_classes: int, confidence_threshold: float,
                 iou_threshold: float, fps: int, camera_fov_rad: float, real_sign_width_m: float = 0.20):
        if dai is None:
            raise RuntimeError("'depthai' no instalado.")

        self.class_names = _load_class_names(Path(classes_path), num_classes)
        self.camera_fov = camera_fov_rad
        self.real_sign_width_m = real_sign_width_m
        self.conf_threshold = confidence_threshold

        print(f"[VISION-CPU] Cargando modelo {pt_path} en CPU usando ultralytics...")
        self.model = YOLO(pt_path)

        pipeline = dai.Pipeline()

        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(*INPUT_SIZE)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(fps)

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        self.device = dai.Device(pipeline)
        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=1, blocking=False)

        self._lock = threading.Lock()
        self._latest: List[Dict] = []
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def _poll_loop(self):
        while self._running:
            try:
                in_rgb = self.q_rgb.get()
            except RuntimeError:
                break
            
            if in_rgb is None:
                continue

            frame = in_rgb.getCvFrame()
            
            # Reducir a 320x320 para que la CPU de la Raspberry Pi sobreviva
            frame_resized = cv2.resize(frame, (320, 320))
            
            # Inferencia 100% en CPU con la librería oficial
            results = self.model(frame_resized, verbose=False, conf=self.conf_threshold)
            
            parsed = []
            if len(results) > 0:
                boxes = results[0].boxes
                for box in boxes:
                    # Las coordenadas resultantes están en base a 320x320, hay que multiplicarlas por 2
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy() * 2.0
                    cls_id = int(box.cls[0].cpu().numpy())
                    
                    parsed.append(self._to_detection(x1, x2, cls_id))
                    
            with self._lock:
                self._latest = parsed

    def _to_detection(self, xmin, xmax, label_idx) -> Dict:
        cx = (xmin + xmax) / 2.0
        normalized_x = (cx - 0.5) * 2.0
        relative_angle = -normalized_x * (self.camera_fov / 2.0)

        bbox_width_px = max((xmax - xmin), 1.0)
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
