#!/usr/bin/env python3
import time
import sys

from TurtleBotController.turtlebot import TurtleBotReal
from controller.navigation import NavigationController


def main():
    print("==================================================")
    print(" NAVEGACIÓN AUTÓNOMA FINAL (IDÉNTICA AL SIMULADOR)")
    print("==================================================")

    robot = TurtleBotReal("config.json")
    print("\nRobot listo. Comenzando...")

    dt = 0.05  # 20 Hz

    controller = NavigationController(
        robot_radius=robot.radius,
        lidar_max_range=robot.lidar_max_range,
        v_max=0.3,             # límite de hardware del robot real
        lidar_min_valid=0.18,  # reflexiones del propio plástico del robot
    )

    try:
        while True:
            # Watchdog (T2): si /scan dejó de llegar, el scan está congelado
            # y manejar con él es manejar a ciegas → robot quieto hasta que vuelva.
            age = robot.scan_age()
            if age > robot.scan_stale_after:
                robot.move(0.0, 0.0, dt)
                sys.stdout.write(
                    f"\r[WATCHDOG] Sin LiDAR hace {min(age, 999.9):5.1f}s — robot detenido        ")
                sys.stdout.flush()
                continue

            lidar_scan = robot.get_lidar_scan()
            vision_dets = robot.get_vision_detections()

            if len(lidar_scan) < 360:
                time.sleep(dt)
                continue

            v_target, w_target = controller.step(lidar_scan, vision_dets, dt)

            info_vision = ""
            if controller.last_signal is not None:
                clase, dist = controller.last_signal
                info_vision = f"[{clase.upper()}:{dist:.1f}m]"

            sys.stdout.write(
                f"\r[{controller.estado:<18}] {info_vision:<12} "
                f"Frente: {controller.dist_frente:.2f}m | v: {v_target:.2f} w: {w_target:>5.2f}  ")
            sys.stdout.flush()

            robot.move(v_target, w_target, dt)

            if controller.estado == "FINALIZADO":
                print("\n\n¡Señal FINISH alcanzada! Deteniendo el robot y terminando el programa.")
                break

    except KeyboardInterrupt:
        print("\n\nPrograma interrumpido por el usuario (Ctrl+C).")
    except Exception as e:
        print(f"\nOcurrió un error inesperado: {e}")
    finally:
        print("Apagando y frenando el robot de forma segura...")
        try:
            robot.stop()
        except:
            pass


if __name__ == "__main__":
    main()
