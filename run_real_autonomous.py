#!/usr/bin/env python3
"""
Navegación autónoma en el robot real usando el CONTROLADOR UNIFICADO
(TurtleBotController/autonomous_controller.py) — el mismo código que se
valida en el simulador headless, sin re-implementaciones divergentes.

Novedades respecto a la FSM anterior:
  * Crucero follow-the-gap (se auto-centra en el pasillo, no reacciona a
    obstáculos que quedaron detrás).
  * Giros CERRADOS con la odometría del Create 3 (/odom): se gira hasta el
    yaw objetivo, no "w=2.0 durante 0.7 s" en lazo abierto.
  * Giro en la intersección de la señal (ventana lateral del LIDAR +
    memoria de señales ya obedecidas).
  * Sin lidar fresco -> frena. Emergencia por caja de barrido (solo
    obstáculos DENTRO del camino del robot).

Parámetros ajustables en TurtleBotController/config.json, sección "controller".
"""
import json
import os
import sys
import time

from TurtleBotController.turtlebot import TurtleBotReal
from TurtleBotController.autonomous_controller import AutonomousController


def main():
    print("==================================================")
    print(" NAVEGACIÓN AUTÓNOMA (CONTROLADOR UNIFICADO)")
    print("==================================================")

    robot = TurtleBotReal("config.json")

    cfg_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "TurtleBotController", "config.json")
    with open(cfg_path) as f:
        params = json.load(f).get("controller", {})
    ctrl = AutonomousController(params)
    print(f"[CONTROL] v_max={ctrl.p['v_max']} m/s, w_max={ctrl.p['w_max']} rad/s")
    if ctrl.p["v_max"] > 0.31:
        print("[CONTROL] OJO: v_max > 0.306 requiere safety_override=full en el")
        print("          nodo motion_control del Create 3 (ver DEPLOY.md).")

    print("\nRobot listo. Comenzando...")
    dt = 0.05  # 20 Hz

    sin_odom_avisado = False
    try:
        while True:
            t0 = time.time()
            lidar_scan = robot.get_lidar_scan()
            vision_dets = robot.get_vision_detections()
            odom = robot.get_odometry()

            if odom is not None:
                yaw, pose = odom[2], (odom[0], odom[1])
            else:
                # Sin /odom el controlador integra los comandos (menos preciso
                # pero funcional). Avisar una sola vez.
                yaw, pose = None, None
                if not sin_odom_avisado:
                    print("\n[ODOM] /odom no disponible: giros por integración "
                          "de comandos (menos precisos). ¿Está la base encendida?")
                    sin_odom_avisado = True

            v, w, estado = ctrl.step(lidar_scan, vision_dets, dt, yaw=yaw, pose=pose)

            info_vision = ""
            if vision_dets:
                d0 = sorted(vision_dets, key=lambda d: d["distance"])[0]
                info_vision = f"[{d0['class'].upper()}:{d0['distance']:.1f}m]"
            sys.stdout.write(
                f"\r[{estado:<18}] {info_vision:<14} v: {v:.2f} w: {w:>5.2f}   ")
            sys.stdout.flush()

            # move() publica y duerme dt; descontar lo ya gastado en el ciclo
            robot.move(v, w, max(0.005, dt - (time.time() - t0)))

    except KeyboardInterrupt:
        print("\n\nPrograma interrumpido por el usuario (Ctrl+C).")
    except Exception as e:
        print(f"\nOcurrió un error inesperado: {e}")
    finally:
        print("Apagando y frenando el robot de forma segura...")
        try:
            robot.stop()
        except Exception:
            pass


if __name__ == "__main__":
    main()
