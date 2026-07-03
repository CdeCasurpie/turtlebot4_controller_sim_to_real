#!/usr/bin/env python3
"""Exporta yolonano/best.pt a NCNN para inferencia rápida en ARM (Raspberry Pi).

Genera yolonano/best_ncnn_model/, que TurtleBotController/turtlebot.py detecta
y prefiere automáticamente sobre el .pt.

Uso: python3 deploy/export_ncnn.py [imgsz]
imgsz default: 320 — best.pt fue entrenado a 320 (auditoría 2026-07); exportar a 640
no mejora la precisión y es 3-4x más lento en el Pi.
"""
import os
import sys

from ultralytics import YOLO

def main():
    imgsz = int(sys.argv[1]) if len(sys.argv) > 1 else 320
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    pt_path = os.path.join(base, "yolonano", "best.pt")
    if not os.path.exists(pt_path):
        print(f"No se encontró el modelo: {pt_path}")
        return 1

    print(f"Exportando {pt_path} a NCNN (imgsz={imgsz})...")
    YOLO(pt_path).export(format="ncnn", imgsz=imgsz)
    out_dir = os.path.join(base, "yolonano", "best_ncnn_model")
    if os.path.isdir(out_dir):
        print(f"Export exitoso: {out_dir}")
        return 0
    print("El export terminó pero no se encontró el directorio de salida.")
    return 1

if __name__ == "__main__":
    sys.exit(main())
