#!/usr/bin/env python3
import time
import sys
import math

# Importamos el controlador del robot real
from TurtleBotController.turtlebot import TurtleBotReal

def main():
    print("==================================================")
    print(" PRUEBA DE VISIÓN (YOLO NANO) Y ALINEACIÓN")
    print("==================================================")
    
    # Esto levantará la conexión con el robot y cargará automáticamente YOLO
    robot = TurtleBotReal("config.json")
    
    print("\nRobot listo. Buscando señales...")
    print("Presiona Ctrl+C para salir.\n")
    
    dt = 0.1  # 10 Hz
    
    try:
        while True:
            scan = robot.get_lidar_scan()
            detecciones = robot.get_vision_detections()
            
            if len(scan) < 360:
                time.sleep(dt)
                continue
                
            v_target = 0.0
            w_target = 0.0
            
            if len(detecciones) > 0:
                # Tomamos la señal más grande/cercana
                senal = sorted(detecciones, key=lambda d: d['distance'])[0]
                clase = senal['class']
                angulo_rel_rad = senal['relative_angle']
                
                # Convertimos el ángulo a grados para buscarlo en el LiDAR
                ang_deg = math.degrees(angulo_rel_rad)
                
                # El LiDAR tiene el 0 al frente. 
                # Ángulos positivos son a la izquierda (antihorario).
                # Ángulos negativos son a la derecha (horario).
                if ang_deg >= 0:
                    idx_lidar = int(ang_deg) % 360
                else:
                    idx_lidar = int(360 + ang_deg) % 360
                    
                distancia_lidar = scan[idx_lidar]
                
                # Lógica de alineación: Girar proporcionalmente al error angular
                # Si el ángulo es positivo (está a la izquierda), gira a la izquierda (w positivo)
                # Si el ángulo es negativo (está a la derecha), gira a la derecha (w negativo)
                
                # Zona muerta: si ya está centrado (+- 3 grados), no girar
                if abs(ang_deg) > 3.0:
                    w_target = angulo_rel_rad * 1.5  # Constante proporcional
                else:
                    w_target = 0.0
                
                # Imprimir en consola lo que ve
                estado_giro = "CENTRADOR" if w_target == 0.0 else ("GIRANDO IZQ" if w_target > 0 else "GIRANDO DER")
                sys.stdout.write(f"\r👁️  Señal: '{clase.upper()}' | Ángulo: {ang_deg:>5.1f}° | LiDAR Dist: {distancia_lidar:.2f}m | {estado_giro}       ")
                sys.stdout.flush()
                
            else:
                # No hay detecciones
                sys.stdout.write(f"\rBuscando... (Ninguna señal en cámara)                                           ")
                sys.stdout.flush()
                
            # ACTUACIÓN FÍSICA: NUNCA AVANZAR (v = 0.0), SOLO GIRAR
            robot.move(0.0, w_target, dt)
            
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
