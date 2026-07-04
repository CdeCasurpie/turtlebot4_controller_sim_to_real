#!/usr/bin/env python3
"""
Sandbox aislado para validar un modelo YOLO (.blob) corriendo en la VPU (Myriad X)
de la OAK-D Lite via DepthAI.

Modo por default (--source cam, sin --display): la Raspberry Pi NO decodifica frames,
NO dibuja nada y NO corre el modelo. La ColorCamera y la YoloDetectionNetwork corren
enteramente dentro de la OAK-D; la Pi solo recibe mensajes de detección ya resueltos
(un puñado de floats por objeto) por USB. --display/--stats son opcionales y solo para
depurar en un banco de pruebas con monitor conectado.

Aislamiento: este script NO importa ni modifica TurtleBotController/ (control real),
world_map.json de producción, ni ninguna lógica de navegación/telemetría. La única
dependencia opcional sobre Simulator/ es una lectura (import) del mock de cámara,
usada solo como generador de frames sintéticos para el modo --source sim.

Modelo bajo prueba: yolonanov2/ (turtlebot_signals_v2). yolonano/ (v1) se ignora
por completo a propósito.
"""
from __future__ import annotations

import argparse
import base64
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Optional

import numpy as np

try:
    import depthai as dai
except ImportError as exc:
    raise SystemExit(
        "El paquete 'depthai' no está instalado en este entorno.\n"
        "Instala la rama 2.x (este script usa YoloDetectionNetwork/XLinkIn/XLinkOut, que no\n"
        "existen en depthai 3.x): pip install \"depthai<3\"\n"
        f"Error original: {exc}"
    ) from exc

if not hasattr(dai.node, "YoloDetectionNetwork"):
    raise SystemExit(
        f"depthai {getattr(dai, '__version__', '?')} está instalado pero es la rama 3.x, que "
        "rediseñó la API (sin YoloDetectionNetwork/XLinkIn/XLinkOut). Este script necesita la "
        "rama 2.x: pip install \"depthai<3\""
    )

REPO_ROOT = Path(__file__).resolve().parent.parent
CLASSES_FILE = REPO_ROOT / "yolonanov2" / "classes.txt"
DEFAULT_BLOB = Path(__file__).resolve().parent / "models" / "turtlebot_signals_v2.blob"

INPUT_SIZE = (640, 640)  # (ancho, alto) - debe coincidir con el imgsz usado al exportar el ONNX
NUM_CLASSES = 4
COORDINATE_SIZE = 4  # cx, cy, w, h (YOLOv8 es "anchor-free": sin objectness ni anclas)


def load_class_names(path: Path) -> list[str]:
    if not path.exists():
        raise FileNotFoundError(
            f"No se encontró el archivo de clases en {path}. "
            "Se esperaba yolonanov2/classes.txt con una clase por línea."
        )
    names = [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if len(names) != NUM_CLASSES:
        raise ValueError(
            f"classes.txt tiene {len(names)} clases pero el pipeline está configurado para {NUM_CLASSES}."
        )
    return names


@dataclass(frozen=True)
class PipelineConfig:
    blob_path: Path
    confidence_threshold: float
    iou_threshold: float
    fps: int
    source: str      # "cam" | "webcam" | "sim"
    display: bool     # si True, streamea el frame de vuelta y lo muestra con cv2.imshow (gasta CPU/USB extra)
    stats: bool        # si True, imprime uso de CPU/memoria/temperatura DEL CHIP OAK-D (no de la Pi)


def build_pipeline(cfg: PipelineConfig) -> tuple[dai.Pipeline, bool]:
    """
    Construye el pipeline de DepthAI. Retorna (pipeline, usa_camara_onboard).
    Si usa_camara_onboard es False, el pipeline expone una entrada XLinkIn llamada
    'frame_in' que el host debe alimentar manualmente con frames (ver `run`).

    El stream 'frame' (passthrough, para dibujar) y el SystemLogger solo se crean si
    se piden explícitamente (--display / --stats): por default el único stream que
    cruza el USB es 'nn' (los resultados ya decodificados), para minimizar trabajo y
    ancho de banda en el lado de la Raspberry Pi.
    """
    if not cfg.blob_path.exists():
        raise FileNotFoundError(
            f"No se encontró el modelo .blob en {cfg.blob_path}.\n"
            "Genera el .blob siguiendo la guía de despliegue (yolonanov2 -> onnx -> blob) "
            "antes de ejecutar este script."
        )

    pipeline = dai.Pipeline()

    detection_nn = pipeline.create(dai.node.YoloDetectionNetwork)
    detection_nn.setBlobPath(str(cfg.blob_path))
    detection_nn.setConfidenceThreshold(cfg.confidence_threshold)
    detection_nn.setNumClasses(NUM_CLASSES)
    detection_nn.setCoordinateSize(COORDINATE_SIZE)
    detection_nn.setAnchors([])       # YOLOv8 es anchor-free
    detection_nn.setAnchorMasks({})   # idem: sin mascaras de anclas
    detection_nn.setIouThreshold(cfg.iou_threshold)
    detection_nn.setNumInferenceThreads(2)
    detection_nn.input.setBlocking(False)

    using_onboard_camera = cfg.source == "cam"

    if using_onboard_camera:
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(*INPUT_SIZE)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(cfg.fps)
        cam_rgb.preview.link(detection_nn.input)
    else:
        xin_frame = pipeline.create(dai.node.XLinkIn)
        xin_frame.setStreamName("frame_in")
        xin_frame.out.link(detection_nn.input)

    xout_nn = pipeline.create(dai.node.XLinkOut)
    xout_nn.setStreamName("nn")
    detection_nn.out.link(xout_nn.input)

    if cfg.display:
        xout_frame = pipeline.create(dai.node.XLinkOut)
        xout_frame.setStreamName("frame")
        detection_nn.passthrough.link(xout_frame.input)

    if cfg.stats:
        sys_logger = pipeline.create(dai.node.SystemLogger)
        sys_logger.setRate(1.0)  # 1 Hz
        xout_sys = pipeline.create(dai.node.XLinkOut)
        xout_sys.setStreamName("sysinfo")
        sys_logger.out.link(xout_sys.input)

    return pipeline, using_onboard_camera


def to_img_frame(bgr_frame: np.ndarray) -> dai.ImgFrame:
    import cv2  # import diferido: solo hace falta en --source webcam/sim (captura/resize en el host)

    resized = cv2.resize(bgr_frame, INPUT_SIZE)
    planar = resized.transpose(2, 0, 1).flatten()
    img = dai.ImgFrame()
    img.setData(planar)
    img.setType(dai.ImgFrame.Type.BGR888p)
    img.setWidth(INPUT_SIZE[0])
    img.setHeight(INPUT_SIZE[1])
    return img


def webcam_frames(cam_index: int = 0) -> Iterator[np.ndarray]:
    import cv2

    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        raise RuntimeError(f"No se pudo abrir la cámara web índice {cam_index}.")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("No se pudo leer un frame de la cámara web.")
            yield frame
    finally:
        cap.release()


def sim_frames() -> Iterator[np.ndarray]:
    """
    Fuente de frames sintéticos generada por el simulador local. Sirve SOLO para validar
    el cableado del pipeline (frame -> VPU -> parseo de detecciones): las imágenes son
    sintéticas (rectángulos con texto), así que no esperes detecciones reales del modelo
    en este modo.

    Import de solo lectura de Simulator/: no se modifica nada del código de producción.
    """
    import cv2

    sys.path.insert(0, str(REPO_ROOT))
    from Simulator.WorldSim.world import World
    from Simulator.TurtleBotSim.turtlebot import TurtleBotMock

    world = World()
    world_map = REPO_ROOT / "world_map.json"
    if world_map.exists():
        world.load_from_file(str(world_map))
    else:
        world.add_signal("left", 1.0, 0.3)

    robot = TurtleBotMock(
        world,
        initial_x=world.robot_start.get("x", 0.0),
        initial_y=world.robot_start.get("y", 0.0),
        initial_theta=world.robot_start.get("theta", 0.0),
    )

    while True:
        b64_jpeg = robot.get_camera_image_base64()
        jpg_bytes = base64.b64decode(b64_jpeg)
        frame = cv2.imdecode(np.frombuffer(jpg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
        yield frame
        time.sleep(1 / 30)


def format_detections(detections, class_names: list[str]) -> str:
    if not detections:
        return "(sin detecciones)"
    partes = []
    for det in detections:
        label = class_names[det.label] if det.label < len(class_names) else str(det.label)
        partes.append(f"{label}:{det.confidence:.2f}")
    return " | ".join(partes)


def run_headless(device: "dai.Device", class_names: list[str], cfg: PipelineConfig) -> None:
    """
    Modo de despliegue en el robot: NO decodifica frames ni abre ventanas. Usa `.get()`
    bloqueante (no polling) sobre la cola 'nn', así el proceso de la Pi queda dormido
    esperando a la VPU en vez de consumir CPU en un bucle activo.
    """
    q_nn = device.getOutputQueue(name="nn", maxSize=4, blocking=True)
    q_sys = device.getOutputQueue(name="sysinfo", maxSize=4, blocking=False) if cfg.stats else None

    print(
        "[OK] Inferencia corriendo 100% en la VPU Myriad X de la OAK-D. "
        "La CPU de la Raspberry Pi solo recibe los resultados (no frames, no cómputo de red). "
        "Ctrl+C para salir.\n"
    )

    last_stats_print = 0.0
    while True:
        in_nn = q_nn.get()  # bloqueante: cero busy-wait en la Pi
        resumen = format_detections(in_nn.detections, class_names)
        sys.stdout.write(f"\r[VPU] {resumen:<80}")
        sys.stdout.flush()

        if q_sys is not None:
            in_sys = q_sys.tryGet()
            now = time.time()
            if in_sys is not None and now - last_stats_print >= 1.0:
                css = in_sys.leonCssCpuUsage.average * 100
                mss = in_sys.leonMssCpuUsage.average * 100
                temp = in_sys.chipTemperature.average
                print(
                    f"\n[VPU-STATS] Leon CSS: {css:5.1f}% | Leon MSS: {mss:5.1f}% | "
                    f"Temp. chip: {temp:4.1f}°C  (todo esto es interno al OAK-D, no de la Pi)"
                )
                last_stats_print = now


def run_display(device: "dai.Device", class_names: list[str], cfg: PipelineConfig,
                 using_onboard_camera: bool) -> None:
    """Modo de banco de pruebas con monitor: dibuja detecciones sobre el frame y las muestra."""
    import cv2

    q_nn = device.getOutputQueue(name="nn", maxSize=4, blocking=False)
    q_frame = device.getOutputQueue(name="frame", maxSize=4, blocking=False)
    q_sys = device.getOutputQueue(name="sysinfo", maxSize=4, blocking=False) if cfg.stats else None

    input_queue = None
    frame_source: Optional[Iterator[np.ndarray]] = None
    if not using_onboard_camera:
        input_queue = device.getInputQueue(name="frame_in")
        frame_source = webcam_frames() if cfg.source == "webcam" else sim_frames()

    print(f"[OK] Pipeline corriendo (modo --display). Fuente de frames: {cfg.source}. Presiona 'q' para salir.")

    try:
        while True:
            if frame_source is not None:
                host_frame = next(frame_source)
                input_queue.send(to_img_frame(host_frame))

            in_nn = q_nn.tryGet()
            in_frame = q_frame.tryGet()

            if in_frame is not None:
                frame = in_frame.getCvFrame()
                if in_nn is not None:
                    h, w = frame.shape[:2]
                    for det in in_nn.detections:
                        x1, y1 = int(det.xmin * w), int(det.ymin * h)
                        x2, y2 = int(det.xmax * w), int(det.ymax * h)
                        label = class_names[det.label] if det.label < len(class_names) else str(det.label)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(
                            frame, f"{label} {det.confidence:.2f}", (x1, max(0, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
                        )
                cv2.imshow("VPU YOLO (yolonanov2)", frame)

            if q_sys is not None:
                in_sys = q_sys.tryGet()
                if in_sys is not None:
                    print(
                        f"[VPU-STATS] Leon CSS: {in_sys.leonCssCpuUsage.average * 100:5.1f}% | "
                        f"Leon MSS: {in_sys.leonMssCpuUsage.average * 100:5.1f}% | "
                        f"Temp. chip: {in_sys.chipTemperature.average:4.1f}°C"
                    )

            if cv2.waitKey(1) == ord("q"):
                break
    finally:
        cv2.destroyAllWindows()
        if frame_source is not None:
            frame_source.close()


def run(cfg: PipelineConfig) -> None:
    class_names = load_class_names(CLASSES_FILE)
    pipeline, using_onboard_camera = build_pipeline(cfg)

    try:
        device = dai.Device(pipeline)
    except RuntimeError as exc:
        raise SystemExit(
            "No se pudo conectar con el dispositivo OAK-D. Verifica:\n"
            "  - Que la OAK-D Lite esté conectada por USB3 (cable/puerto azul).\n"
            "  - En Linux (Raspberry Pi): reglas udev instaladas (ver guía de despliegue).\n"
            "  - Que ningún otro proceso tenga el dispositivo abierto.\n"
            f"Error original: {exc}"
        ) from exc

    with device:
        if cfg.display:
            run_display(device, class_names, cfg, using_onboard_camera)
        else:
            if not using_onboard_camera:
                raise SystemExit(
                    "--source webcam/sim requieren --display (necesitas ver la ventana para "
                    "confirmar que las detecciones tienen sentido sobre un frame que tú generas). "
                    "El modo headless (sin --display) es exclusivo de --source cam."
                )
            run_headless(device, class_names, cfg)


def parse_args() -> PipelineConfig:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--blob", type=Path, default=DEFAULT_BLOB,
        help="Ruta al modelo .blob derivado de yolonanov2/ (ver guía de despliegue).",
    )
    parser.add_argument(
        "--source", choices=["cam", "webcam", "sim"], default="cam",
        help="cam=cámara integrada de la OAK-D (default, único modo sin --display) | "
             "webcam=cámara del host vía USB (requiere --display) | "
             "sim=frames sintéticos del simulador (requiere --display).",
    )
    parser.add_argument("--conf", type=float, default=0.5, help="Umbral de confianza.")
    parser.add_argument("--iou", type=float, default=0.5, help="Umbral IOU para NMS on-device.")
    parser.add_argument("--fps", type=int, default=15, help="FPS solicitados a la ColorCamera (modo cam).")
    parser.add_argument(
        "--display", action="store_true",
        help="Streamea el frame de vuelta y lo muestra con cv2.imshow. Solo para depurar en un "
             "banco de pruebas con monitor conectado: consume CPU/USB extra en el host. "
             "NO usar en el despliegue final en la Raspberry Pi.",
    )
    parser.add_argument(
        "--stats", action="store_true",
        help="Imprime uso de CPU/temperatura DEL CHIP OAK-D (Leon CSS/MSS, vía SystemLogger). "
             "Útil para comprobar que el trabajo ocurre en la VPU y no en la Raspberry Pi.",
    )
    args = parser.parse_args()

    return PipelineConfig(
        blob_path=args.blob,
        confidence_threshold=args.conf,
        iou_threshold=args.iou,
        fps=args.fps,
        source=args.source,
        display=args.display,
        stats=args.stats,
    )


def main() -> None:
    cfg = parse_args()
    try:
        run(cfg)
    except KeyboardInterrupt:
        print("\nInterrumpido por el usuario.")
    except (FileNotFoundError, ValueError) as exc:
        print(f"[ERROR de configuración] {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
