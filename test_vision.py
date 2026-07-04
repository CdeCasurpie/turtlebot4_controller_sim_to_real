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
    
    # Tracker temporal para YOLO
    tracker = {
        'class': None,
        'relative_angle': 0.0,
        'distance': float('inf'),
        'frames_lost': 999,
        'max_frames': 15  # Mantiene la señal por ~1.5s (a 10Hz)
    }
    
    try:
        while True:
            scan = robot.get_lidar_scan()
            detecciones = robot.get_vision_detections()
            
            if len(scan) < 360:
                time.sleep(dt)
                continue
                
            # Filtro de ruido del chasis del robot
            scan = [d if d >= 0.18 else robot.lidar_max_range for d in scan]
                
            v_target = 0.0
            w_target = 0.0
            
            if len(detecciones) > 0:
                # Tomamos la señal más centrada
                senal = sorted(detecciones, key=lambda d: abs(d['relative_angle']))[0]
                
                tracker['class'] = senal['class']
                tracker['relative_angle'] = senal['relative_angle']
                
                # Distancia extraída desde el LiDAR en la dirección de la señal (+- 5 grados)
                ang_grados = int(math.degrees(senal['relative_angle']))
                dist_lidar = min([scan[(ang_grados + i) % 360] for i in range(-5, 6)])
                
                tracker['distance'] = dist_lidar
                tracker['frames_lost'] = 0
            else:
                tracker['frames_lost'] += 1
                
            if tracker['frames_lost'] < tracker['max_frames']:
                clase = tracker['class']
                angulo_rel_rad = tracker['relative_angle']
                ang_deg = math.degrees(angulo_rel_rad)
                distancia_lidar = tracker['distance']
                
                # Lógica de alineación: Girar proporcionalmente al error angular
                if abs(ang_deg) > 3.0:
                    w_target = angulo_rel_rad * 1.5  # Constante proporcional
                else:
                    w_target = 0.0
                
                # Imprimir en consola lo que ve
                if tracker['frames_lost'] > 0:
                    estado_giro = f"TRACKING (Lost: {tracker['frames_lost']})" + (" ↺" if w_target > 0 else " ↻" if w_target < 0 else " ✓")
                else:
                    estado_giro = "CENTRADO " if w_target == 0.0 else ("GIRANDO IZQ" if w_target > 0 else "GIRANDO DER")
                    
                sys.stdout.write(f"\r👁️  Señal: '{clase.upper()}' | Ángulo: {ang_deg:>5.1f}° | LiDAR Dist: {distancia_lidar:.2f}m | {estado_giro}       ")
                sys.stdout.flush()
                
            else:
                # No hay detecciones y se perdió el tracker
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
