#!/usr/bin/env python3
import time
import sys
import math
import numpy as np

from TurtleBotController.turtlebot import TurtleBotReal

def buscar_camino_libre(lidar_points, radio_robot, direccion='front', margen_extra=0.10):
    if direccion == 'left':
        angulos = [50, 63, 76, 90, 103, 116, 130]
        M = 5
    elif direccion == 'right':
        angulos = [230, 243, 256, 270, 283, 296, 310]
        M = 5
    elif direccion == 'front':
        angulos = [-20, -13, -6, 0, 6, 13, 20]
        M = 5
    else:
        angulos = range(0, 360, 30)
        M = 3
        
    margen = radio_robot + margen_extra
    paso_inicial = 0.3 
    distancia_paso = (2 * radio_robot) / M
    distancias_prueba = [paso_inicial + i * distancia_paso for i in range(M)]
    
    intentos = []
    mejor_ang = None
    
    for ang_c in angulos:
        ruta_valida = True
        ang_eval = ang_c if ang_c >= 0 else ang_c + 360
        
        for d_c in distancias_prueba:
            cx = d_c * math.cos(math.radians(ang_eval))
            cy = d_c * math.sin(math.radians(ang_eval))
            
            choca = False
            for px, py in lidar_points:
                if math.hypot(px - cx, py - cy) < margen:
                    choca = True
                    break
                    
            if choca:
                ruta_valida = False
                break 
                
        intentos.append({'angulo': ang_eval, 'valido': ruta_valida})
        if ruta_valida and mejor_ang is None:
            mejor_ang = ang_eval
            
    return mejor_ang is not None, mejor_ang, intentos, distancias_prueba, margen


def main():
    print("==================================================")
    print(" NAVEGACIÓN AUTÓNOMA FINAL (IDÉNTICA AL SIMULADOR)")
    print("==================================================")
    
    robot = TurtleBotReal("config.json")
    print("\nRobot listo. Comenzando...")
    
    dt = 0.05  # 20 Hz
    
    estado_actual = "EXPLORANDO"
    tiempo_estado = 0.0
    cooldown_senal = 0.0
    
    try:
        while True:
            lidar_scan = robot.get_lidar_scan()
            vision_dets = robot.get_vision_detections()
            
            if len(lidar_scan) < 360:
                time.sleep(dt)
                continue
                
            if cooldown_senal > 0:
                cooldown_senal -= dt

            # Coordenadas cartesianas del LiDAR para raycasting
            lidar_points = []
            for i, dist_p in enumerate(lidar_scan):
                # IMPORTANTE MUNDO REAL: Ignorar dist_p < 0.18 porque son reflexiones
                # del propio plástico del robot (ruido del sensor) que bloquean el algoritmo.
                if 0.18 < dist_p < robot.lidar_max_range:
                    lidar_points.append((dist_p * math.cos(math.radians(i)), dist_p * math.sin(math.radians(i))))
                    
            # Analizar el abanico frontal estricto
            dist_frente_estricto = min(lidar_scan[0:15] + lidar_scan[345:360])

            v_target = 0.0
            w_target = 0.0
            info_vision = ""

            # ========================================================
            # 1. ACTUALIZAR ESTADO SEGÚN YOLO (Transiciones)
            # ========================================================
            if len(vision_dets) > 0 and cooldown_senal <= 0:
                senal = sorted(vision_dets, key=lambda d: d['distance'])[0]
                clase = senal['class']
                dist = senal['distance']
                
                info_vision = f"[{clase.upper()}:{dist:.1f}m]"
                
                if estado_actual == "EXPLORANDO":
                    if clase == 'left':
                        estado_actual = "BUSCANDO_IZQ"
                    elif clase == 'right':
                        estado_actual = "BUSCANDO_DER"
                    elif clase == 'stop' and dist <= 1.6:
                        estado_actual = "DETENIDO"
                        tiempo_estado = 3.0

            # ========================================================
            # 2. LÓGICA DE CADA ESTADO (Misma matemática del simulador)
            # ========================================================
            if estado_actual == "EXPLORANDO":
                # Control fluido proporcional original de test_autonomous_controller.py
                v_target = max(0.1, min(0.3, (dist_frente_estricto - 0.4) * 0.8)) # max 0.3 por hardware
                
                min_dist = min(lidar_scan)
                min_angle = np.argmin(lidar_scan)
                if min_angle > 180: min_angle -= 360

                if min_dist < 0.7:
                    factor_giro = 1.5 if min_dist < 0.4 else 1.0
                    margen = 0.7
                    if min_angle >= 0:
                        target = 90 + (margen - min_dist) * 80.0 
                        w_target -= math.radians(target - min_angle) * factor_giro
                    else:
                        target = -90 - (margen - min_dist) * 80.0
                        w_target -= math.radians(target - min_angle) * factor_giro

            elif estado_actual in ["BUSCANDO_IZQ", "BUSCANDO_DER"]:
                v_target = max(0.1, min(0.3, (dist_frente_estricto - 0.4) * 0.8))
                
                # Tracking visual original (K=2.5)
                if len(vision_dets) > 0:
                    senal = sorted(vision_dets, key=lambda d: d['distance'])[0]
                    w_target = senal['relative_angle'] * 2.5
                
                # SIMULACIÓN (RAYCASTING) HACIA LOS LADOS
                dir_search = 'left' if estado_actual == "BUSCANDO_IZQ" else 'right'
                espacio, _, _, _, _ = buscar_camino_libre(lidar_points, robot.radius, dir_search, 0.10)
                
                if espacio:
                    estado_actual = "GIRANDO_IZQ" if estado_actual == "BUSCANDO_IZQ" else "GIRANDO_DER"
                    tiempo_estado = 0.0

            elif estado_actual in ["GIRANDO_IZQ", "GIRANDO_DER"]:
                v_target = max(0.1, min(0.3, (dist_frente_estricto - 0.4) * 0.8))
                w_target = 2.0 if estado_actual == "GIRANDO_IZQ" else -2.0
                tiempo_estado += dt
                
                if tiempo_estado >= 0.7: 
                    estado_actual = "EXPLORANDO"
                    cooldown_senal = 0.2

            elif estado_actual == "DETENIDO":
                v_target = 0.0
                w_target = 0.0
                tiempo_estado -= dt
                if tiempo_estado <= 0:
                    estado_actual = "EXPLORANDO"
                    cooldown_senal = 3.0

            # ========================================================
            # 3. ANTI-CHOQUES Y EVASIÓN DE EMERGENCIA
            # ========================================================
            riesgo_inminente = False
            min_dist_frontal = min(lidar_scan[0:45] + lidar_scan[315:360])
            
            if min_dist_frontal < 0.32 and v_target > 0.05:
                riesgo_inminente = True
            
            # Radio físico (17cm) + ruido (2cm) = 19cm
            if min(lidar_scan) < 0.19:
                riesgo_inminente = True 
                
            if riesgo_inminente or estado_actual == "EVASION_EMERGENCIA":
                if estado_actual != "EVASION_EMERGENCIA":
                    tiempo_estado = 0.0 
                
                estado_actual = "EVASION_EMERGENCIA"
                v_target = 0.0 
                tiempo_estado += dt
                
                esp_escape, ang_escape, _, _, _ = buscar_camino_libre(lidar_points, robot.radius, 'front', 0.02)
                
                if esp_escape:
                    ang_rel = ang_escape if ang_escape <= 180 else ang_escape - 360
                    w_target = math.radians(ang_rel) * 4.0
                    
                    if abs(ang_rel) < 15 and dist_frente_estricto > 0.4:
                        estado_actual = "EXPLORANDO" 
                else:
                    w_target = 3.0

            # ========================================================
            # 4. LOGS EN UNA SOLA LÍNEA (con \r)
            # ========================================================
            sys.stdout.write(f"\r[{estado_actual:<18}] {info_vision:<12} Frente: {dist_frente_estricto:.2f}m | v: {v_target:.2f} w: {w_target:>5.2f}  ")
            sys.stdout.flush()

            robot.move(v_target, w_target, dt)

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
