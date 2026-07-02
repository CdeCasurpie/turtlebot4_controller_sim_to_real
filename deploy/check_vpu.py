#!/usr/bin/env python3
"""Verifica el backend VPU sin ROS: abre la OAK-D, corre YOLO en la Myriad X
y muestra las detecciones durante unos segundos.

Uso (en el robot, con el nodo oakd detenido):
    sudo systemctl stop oakd
    source ~/tb4_controller_venv/bin/activate
    python3 deploy/check_vpu.py [segundos]

Útil para: confirmar que el blob carga, medir el rate de detección y calibrar
distance_scale (pon una señal a 1.0 m y compara con la distancia reportada).
"""
import json
import math
import os
import sys
import time

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, BASE)

def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0

    with open(os.path.join(BASE, "TurtleBotController", "config.json")) as f:
        cfg = json.load(f)
    vis = cfg["vision"]
    vpu_cfg = vis.get("vpu", {})

    ctrl_dir = os.path.join(BASE, "TurtleBotController")
    blob_path = os.path.join(ctrl_dir, vpu_cfg.get("blob_path", "../yolonano/vpu/best.blob"))
    config_path = os.path.join(ctrl_dir, vpu_cfg.get("config_path", "../yolonano/vpu/best.json"))

    print(f"Blob:   {os.path.normpath(blob_path)}")
    print(f"Config: {os.path.normpath(config_path)}")
    for p in (blob_path, config_path):
        if not os.path.exists(p):
            print(f"\nERROR: falta {os.path.normpath(p)}")
            print("Convierte el modelo primero (ver DEPLOY.md, sección VPU).")
            return 1

    try:
        import depthai  # noqa: F401
    except ImportError:
        print("\nERROR: falta el paquete depthai. Instala con: pip install depthai")
        return 1

    from TurtleBotController.vpu_vision import VpuVision

    print("\nAbriendo la OAK-D (si falla con DEVICE_ALREADY_IN_USE: sudo systemctl stop oakd)...")
    vpu = VpuVision(
        blob_path=blob_path,
        config_path=config_path,
        camera_fov=math.radians(vpu_cfg.get("camera_fov_deg", vis.get("camera_fov_deg", 60.0))),
        conf_threshold=vis.get("confidence_threshold", 0.85),
        fps=vpu_cfg.get("fps", 15.0),
        distance_scale=vpu_cfg.get("distance_scale", 1.0),
        detection_max_age=vis.get("detection_max_age", 1.0),
    )

    print(f"Leyendo detecciones por {duration:.0f} s (muéstrale una señal left/right/stop)...\n")
    t_end = time.time() + duration
    try:
        while time.time() < t_end:
            dets = vpu.get_detections()
            if dets:
                txt = " | ".join(
                    f"{d['class']}: {d['distance']:.2f}m @ {math.degrees(d['relative_angle']):+.1f}°"
                    for d in dets
                )
            else:
                txt = "(sin detecciones)"
            sys.stdout.write(f"\r{txt:<70}")
            sys.stdout.flush()
            time.sleep(0.1)
    finally:
        frames = vpu.frames
        vpu.close()

    print(f"\n\nFrames de inferencia recibidos: {frames} (~{frames/duration:.1f} FPS)")
    print("Si la distancia reportada no coincide con la real, ajusta")
    print('vision.vpu.distance_scale en TurtleBotController/config.json:')
    print("  distance_scale_nuevo = distance_scale_actual * distancia_real / distancia_reportada")
    return 0

if __name__ == "__main__":
    sys.exit(main())
