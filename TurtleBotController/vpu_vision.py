"""Backend de visión que corre YOLO dentro de la VPU (Myriad X) de la cámara OAK-D.

La inferencia ocurre en el chip de la cámara: el Pi solo recibe detecciones ya
decodificadas (~20 FPS, CPU casi libre). Requiere:

  - pip install depthai
  - el modelo convertido a blob (ver DEPLOY.md, sección VPU):
      yolonano/vpu/best.blob  +  yolonano/vpu/best.json
  - el nodo ROS de la cámara detenido (sudo systemctl stop oakd), porque la
    OAK-D solo admite un proceso dueño a la vez.

Entrega detecciones en el mismo formato que el backend ROS+YOLO:
  [{'class': 'left', 'distance': 1.5, 'relative_angle': 0.1}, ...]
"""
import json
import os
import threading
import time


def _load_nn_config(config_path):
    """Parsea el JSON que genera tools.luxonis.com junto al blob."""
    with open(config_path, 'r') as f:
        raw = json.load(f)

    nn_cfg = raw.get("nn_config", raw)
    meta = nn_cfg.get("NN_specific_metadata", {})

    input_size = nn_cfg.get("input_size", "416x416")
    width, height = (int(v) for v in str(input_size).lower().split("x"))

    labels = raw.get("mappings", {}).get("labels") or ["left", "right", "stop"]

    return {
        "width": width,
        "height": height,
        "classes": meta.get("classes", len(labels)),
        "coordinates": meta.get("coordinates", 4),
        "anchors": meta.get("anchors") or [],
        "anchor_masks": meta.get("anchor_masks") or {},
        "iou_threshold": meta.get("iou_threshold", 0.5),
        "labels": labels,
    }


class VpuVision:
    def __init__(self, blob_path, config_path, camera_fov,
                 conf_threshold=0.85, fps=15.0, distance_scale=1.0,
                 detection_max_age=1.0):
        import depthai as dai  # import tardío: solo hace falta con este backend

        if not os.path.exists(blob_path):
            raise FileNotFoundError(f"Blob no encontrado: {blob_path}")
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Config del blob no encontrado: {config_path}")

        self.camera_fov = camera_fov
        self.distance_scale = distance_scale
        self.detection_max_age = detection_max_age

        cfg = _load_nn_config(config_path)
        self._labels = cfg["labels"]

        pipeline = dai.Pipeline()

        cam = pipeline.create(dai.node.ColorCamera)
        cam.setPreviewSize(cfg["width"], cfg["height"])
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setFps(fps)

        nn = pipeline.create(dai.node.YoloDetectionNetwork)
        nn.setBlobPath(blob_path)
        nn.setConfidenceThreshold(float(conf_threshold))
        nn.setNumClasses(cfg["classes"])
        nn.setCoordinateSize(cfg["coordinates"])
        nn.setAnchors(cfg["anchors"])
        nn.setAnchorMasks(cfg["anchor_masks"])
        nn.setIouThreshold(cfg["iou_threshold"])
        nn.setNumInferenceThreads(2)
        nn.input.setBlocking(False)

        cam.preview.link(nn.input)

        xout = pipeline.create(dai.node.XLinkOut)
        xout.setStreamName("detections")
        nn.out.link(xout.input)

        # Abre la cámara y arranca el pipeline en la VPU (puede tardar unos segundos).
        # Si el nodo oakd de ROS sigue corriendo, esto lanza X_LINK_DEVICE_ALREADY_IN_USE.
        self._device = dai.Device(pipeline)
        self._queue = self._device.getOutputQueue("detections", maxSize=4, blocking=False)

        self._lock = threading.Lock()
        self._cached = []
        self._cached_time = 0.0
        self.frames = 0  # frames inferidos desde el arranque (para diagnóstico)
        self._stop = False
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

        print(f"[VISION-VPU] Pipeline corriendo en la Myriad X "
              f"({cfg['width']}x{cfg['height']} @ {fps} FPS, clases: {self._labels})")

    def _reader_loop(self):
        while not self._stop:
            msg = self._queue.tryGet()
            if msg is None:
                time.sleep(0.005)
                continue

            detections = []
            for d in msg.detections:
                if not (0 <= d.label < len(self._labels)):
                    continue
                # Coordenadas normalizadas [0..1]
                bbox_frac = max(d.xmax - d.xmin, 1e-3)
                cx = (d.xmin + d.xmax) / 2.0
                normalized_x = (cx - 0.5) / 0.5

                # Misma convención pinhole del backend ROS (focal = ancho de imagen):
                # distance = ancho_real * focal_px / bbox_px = 0.20 / bbox_frac.
                # distance_scale permite calibrar contra una medición real en el lab.
                detections.append({
                    'class': self._labels[d.label],
                    'distance': self.distance_scale * 0.20 / bbox_frac,
                    'relative_angle': -normalized_x * (self.camera_fov / 2.0),
                })

            with self._lock:
                self._cached = detections
                self._cached_time = time.time()
                self.frames += 1

    def get_detections(self):
        with self._lock:
            if time.time() - self._cached_time <= self.detection_max_age:
                return list(self._cached)
            return []

    def close(self):
        self._stop = True
        try:
            self._device.close()
        except Exception:
            pass
