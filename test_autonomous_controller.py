import pygame
import sys
import json
import math
import os
import numpy as np

from Simulator.WorldSim.world import World
from Simulator.TurtleBotSim.turtlebot import TurtleBotMock
import time

use_simulator = "--simulator" in sys.argv

if not use_simulator:
    from TurtleBotController.turtlebot import TurtleBotReal

SCALE = 50.0
WIDTH, HEIGHT = 800, 600
OFFSET_X, OFFSET_Y = WIDTH // 2, HEIGHT // 2

def to_screen(x, y):
    return int(OFFSET_X + x * SCALE), int(OFFSET_Y - y * SCALE)

def buscar_camino_libre(lidar_points, radio_robot, direccion='front', margen_extra=0.10):
    # Aperturas optimizadas: 7 ángulos, M menor
    if direccion == 'left':
        angulos = [85, 90, 95]
        M = 5
    elif direccion == 'right':
        angulos = [265, 270, 275]
        M = 5
    elif direccion == 'front':
        angulos = [-20, -13, -6, 0, 6, 13, 20]
        M = 5
    else:
        angulos = range(0, 360, 30) # 12 ángulos
        M = 3 # Solo 3 pasos para escapar rápido
        
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
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("TurtleBot 4 - Navegación Definitiva Anti-Choques")
    clock = pygame.time.Clock()

    if use_simulator:
        world = World()
        map_file = "world_map.json"

        if os.path.exists(map_file):
            world.load_from_file(map_file)
        else:
            print("No se encontró mapa.")
            sys.exit()

        robot = TurtleBotMock(
            world, 
            initial_x=world.robot_start['x'], 
            initial_y=world.robot_start['y'], 
            initial_theta=world.robot_start['theta']
        )
        dt = 1 / 30.0
    else:
        robot = TurtleBotReal("config.json")
        world = None
        dt = 0.05

    running = True
    paused = False

    estado_actual = "EXPLORANDO"
    ultimo_giro = 'left'
    tiempo_estado = 0.0
    cooldown_senal = 0.0
    choques = 0
    
    # Tracker temporal para YOLO
    tracker = {
        'class': None,
        'relative_angle': 0.0,
        'distance': float('inf'),
        'frames_lost': 999,
        'max_frames': 90,  # 3 segundos a 30 FPS
        'consecutive_frames': 0
    }
    
    v_target = 0.0
    w_target = 0.0

    history = []
    history_index = -1
    time_since_save = 0.0

    def get_current_state():
        if use_simulator:
            rx, ry, rth = robot._get_true_pose()
        else:
            rx, ry, rth = 0.0, 0.0, 0.0
            
        return {
            'x': rx,
            'y': ry,
            'theta': rth,
            'estado_actual': estado_actual,
            'ultimo_giro': ultimo_giro,
            'tiempo_estado': tiempo_estado,
            'cooldown_senal': cooldown_senal,
            'choques': choques,
            'tracker': dict(tracker)
        }
        
    def set_state(st):
        nonlocal estado_actual, ultimo_giro, tiempo_estado, cooldown_senal, choques, tracker
        if use_simulator:
            robot._TurtleBotMock__x = st['x']
            robot._TurtleBotMock__y = st['y']
            robot._TurtleBotMock__theta = st['theta']
        estado_actual = st['estado_actual']
        ultimo_giro = st.get('ultimo_giro', 'left')
        tiempo_estado = st['tiempo_estado']
        cooldown_senal = st['cooldown_senal']
        choques = st.get('choques', choques)
        tracker = dict(st.get('tracker', tracker))

    history.append(get_current_state())
    history_index = 0
    view_mode = "global" if use_simulator else "robot"
    last_time = time.time()
    
    sim_config_path = "sim_config.json"
    sim_config = {}
    config_frames = 0
    
    def load_config():
        nonlocal sim_config
        try:
            with open(sim_config_path, "r") as f:
                sim_config = json.load(f)
            if use_simulator and hasattr(robot, 'update_config'):
                robot.update_config(sim_config)
        except Exception:
            pass
            
    load_config()

    while running:
        if use_simulator:
            config_frames += 1
            if config_frames >= 30:
                config_frames = 0
                load_config()
                
        c_vision_dist = sim_config.get("vision_reliable_dist", 0.45)
        c_stop_dist = sim_config.get("stop_finish_dist", 1.6)
        c_max_v = sim_config.get("max_v_target_sim", 0.8)
        c_min_v = sim_config.get("min_v_target_sim", 0.1)
        c_v_turn = sim_config.get("v_target_turn", 0.3)
        c_w_turn = sim_config.get("w_target_turn", 1.5)
        c_w_appr = sim_config.get("w_target_approach", 2.5)
        c_evas_f = sim_config.get("evasion_frontal_dist", 0.35)
        c_evas_g = sim_config.get("evasion_general_dist", 0.18)
        c_min_frames = sim_config.get("min_consecutive_frames", 4)
        c_cool_post = sim_config.get("cooldown_post_giro", 1.5)
        c_cool_stop = sim_config.get("cooldown_stop", 3.0)
        c_time_stop = sim_config.get("tiempo_espera_stop", 3.0)
        c_rad_amarillo = sim_config.get("radio_amarillo_suave", 0.7)
        c_rad_giro_f = sim_config.get("radio_giro_fuerte", 0.4)
        c_fac_rep_s = sim_config.get("factor_repulsion_suave", 1.0)
        c_fac_rep_f = sim_config.get("factor_repulsion_fuerte", 1.5)
        c_time_min_g = sim_config.get("tiempo_min_giro", 0.8)
        c_time_max_g = sim_config.get("tiempo_max_giro", 2.0)

        if not use_simulator:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_l and use_simulator:
                    view_mode = "robot" if view_mode == "global" else "global"
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                    if not paused and history_index < len(history) - 1:
                        history = history[:history_index+1]
                elif event.key == pygame.K_LEFT and paused:
                    if history_index > 0:
                        history_index -= 1
                        set_state(history[history_index])
                elif event.key == pygame.K_RIGHT and paused:
                    if history_index < len(history) - 1:
                        history_index += 1
                        set_state(history[history_index])

        lidar_scan_raw = robot.get_lidar_scan()
        if len(lidar_scan_raw) < 360:
            if not use_simulator:
                time.sleep(0.05)
            continue
            
        # Filtrar reflexiones propias del chasis simulando el hardware real
        lidar_scan = [d if d >= 0.18 else robot.lidar_max_range for d in lidar_scan_raw]
        vision_dets = robot.get_vision_detections()
        
        # Pre-computar puntos x,y del lidar para que la función sea ultra rápida
        lidar_points = []
        for i, dist_p in enumerate(lidar_scan):
            if dist_p < robot.lidar_max_range:
                lidar_points.append((dist_p * math.cos(math.radians(i)), dist_p * math.sin(math.radians(i))))
                
        intentos_render = []
        render_distancias = []
        render_margen = 0

        if not paused:
            if cooldown_senal > 0:
                cooldown_senal -= dt

            v_target = 0.0
            w_target = 0.0
            
            dist_frente_estricto = min(lidar_scan[0:15] + lidar_scan[345:360])

            # ========================================================
            # 1. ACTUALIZAR TRACKER Y ESTADO SEGÚN YOLO
            # ========================================================
            if len(vision_dets) > 0:
                # Priorizar la señal que ya estamos trackeando si sigue visible para evitar flickering
                if tracker['class'] is not None and tracker['frames_lost'] < tracker['max_frames']:
                    mismas_clase = [d for d in vision_dets if d['class'] == tracker['class']]
                    if len(mismas_clase) > 0:
                        senal = sorted(mismas_clase, key=lambda d: abs(d['relative_angle']))[0]
                    else:
                        senal = sorted(vision_dets, key=lambda d: abs(d['relative_angle']))[0]
                else:
                    senal = sorted(vision_dets, key=lambda d: abs(d['relative_angle']))[0]
                
                if tracker['class'] == senal['class']:
                    tracker['consecutive_frames'] += 1
                else:
                    tracker['consecutive_frames'] = 1
                
                # Distancia extraída desde el LiDAR en la dirección de la señal
                ang_grados = int(math.degrees(senal['relative_angle']))
                dist_lidar = min([lidar_scan[(ang_grados + i) % 360] for i in range(-5, 6)])
                
                tracker['class'] = senal['class']
                tracker['relative_angle'] = senal['relative_angle']
                tracker['distance'] = dist_lidar
                tracker['frames_lost'] = 0
            else:
                tracker['frames_lost'] += 1
                if tracker['frames_lost'] >= tracker['max_frames']:
                    tracker['consecutive_frames'] = 0
                    if estado_actual == "ACERCANDOSE_A_SENAL":
                        estado_actual = "EXPLORANDO"

            if tracker['frames_lost'] < tracker['max_frames'] and cooldown_senal <= 0 and tracker['consecutive_frames'] >= c_min_frames:
                clase = tracker['class']
                dist = tracker['distance']
                
                if estado_actual in ["EXPLORANDO", "ACERCANDOSE_A_SENAL"]:
                    if clase == 'left':
                        if dist <= c_vision_dist:
                            estado_actual = "BUSCANDO_IZQ"
                        else:
                            estado_actual = "ACERCANDOSE_A_SENAL"
                    elif clase == 'right':
                        if dist <= c_vision_dist:
                            estado_actual = "BUSCANDO_DER"
                        else:
                            estado_actual = "ACERCANDOSE_A_SENAL"
                    elif clase == 'stop' and dist <= c_stop_dist:
                        estado_actual = "DETENIDO"
                        tiempo_estado = c_time_stop
                    elif clase == 'finish' and dist <= c_stop_dist:
                        estado_actual = "FINALIZADO"

            # ========================================================
            # 2. LÓGICA DE CADA ESTADO
            # ========================================================
            
            # Función local para evadir paredes con interpolación matemática muy suave
            def calcular_repulsion(scan, rad_amarillo, rad_fuerte, fac_suave, fac_fuerte):
                min_izq = min(scan[0:180])
                min_der = min(scan[180:360])
                
                def calcular_fuerza(dist):
                    if dist >= rad_amarillo:
                        return 0.0
                    
                    # Proporción base lineal (0 en rad_amarillo, 1 en el centro)
                    intensidad = (rad_amarillo - dist) / rad_amarillo
                    
                    if dist <= rad_fuerte:
                        # Sube exponencialmente si está a punto de chocar
                        sobre_paso = (rad_fuerte - dist) / rad_fuerte
                        mult = fac_fuerte + (sobre_paso ** 2) * 5.0
                    else:
                        # Interpolación 100% suave entre suave y fuerte para pasillos normales
                        ratio = (rad_amarillo - dist) / (rad_amarillo - rad_fuerte)
                        mult = fac_suave + ratio * (fac_fuerte - fac_suave)
                        
                    return intensidad * mult

                f_izq = calcular_fuerza(min_izq)
                f_der = calcular_fuerza(min_der)
                return (f_der - f_izq)

            if estado_actual == "EXPLORANDO":
                v_target = max(c_min_v, min(c_max_v, (dist_frente_estricto - 0.4) * c_max_v))
                
                # Multiplicador empírico para convertir la fuerza en velocidad angular (rad/s)
                w_target = calcular_repulsion(lidar_scan, c_rad_amarillo, c_rad_giro_f, c_fac_rep_s, c_fac_rep_f) * 2.5
                
                # Limitar el giro en exploración para que no dé volantazos exagerados
                w_target = max(-1.5, min(1.5, w_target))

            elif estado_actual == "ACERCANDOSE_A_SENAL":
                # Misma velocidad que explorando
                v_target = max(c_min_v, min(c_max_v, (dist_frente_estricto - 0.4) * c_max_v))
                
                w_camara = 0.0
                if tracker['frames_lost'] < tracker['max_frames']:
                    w_camara = tracker['relative_angle'] * c_w_appr
                
                # Mantener la flecha pero evadiendo paredes al mismo tiempo
                w_repulsion = calcular_repulsion(lidar_scan, c_rad_amarillo, c_rad_giro_f, c_fac_rep_s, c_fac_rep_f) * 1.5
                w_target = w_camara + w_repulsion
                    
            elif estado_actual in ["BUSCANDO_IZQ", "BUSCANDO_DER"]:
                # Misma velocidad que explorando, la velocidad varía solo si nos vamos a estrellar
                v_target = max(c_min_v, min(c_max_v, (dist_frente_estricto - 0.4) * c_max_v))
                
                w_camara = 0.0
                if tracker['frames_lost'] < tracker['max_frames']:
                    w_camara = tracker['relative_angle'] * c_w_appr
                    
                w_repulsion = calcular_repulsion(lidar_scan, c_rad_amarillo, c_rad_giro_f, c_fac_rep_s, c_fac_rep_f) * 1.5
                w_target = w_camara + w_repulsion
                
                dir_search = 'left' if estado_actual == "BUSCANDO_IZQ" else 'right'
                
                # Probar con margen grueso, medio, y finalmente el radio exacto
                espacio, _, intentos, dists, marg = buscar_camino_libre(lidar_points, robot.radius, dir_search, 0.10)
                if not espacio:
                    espacio, _, intentos, dists, marg = buscar_camino_libre(lidar_points, robot.radius, dir_search, 0.05)
                if not espacio:
                    espacio, _, intentos, dists, marg = buscar_camino_libre(lidar_points, robot.radius, dir_search, 0.0)
                
                intentos_render = intentos
                render_distancias = dists
                render_margen = marg
                
                if espacio:
                    estado_actual = "GIRANDO_IZQ" if estado_actual == "BUSCANDO_IZQ" else "GIRANDO_DER"
                    ultimo_giro = 'left' if estado_actual == "GIRANDO_IZQ" else 'right'
                    tiempo_estado = 0.0

            elif estado_actual in ["GIRANDO_IZQ", "GIRANDO_DER"]:
                # Avanzamos y giramos más rápido para no chocar con la pared frontal
                v_target = c_v_turn 
                w_target = c_w_turn if estado_actual == "GIRANDO_IZQ" else -c_w_turn
                tiempo_estado += dt
                
                # Verificamos visualmente el frente (para debug en UI)
                espacio_frente, _, intentos, dists, marg = buscar_camino_libre(lidar_points, robot.radius, 'front', 0.10)
                if not espacio_frente:
                    espacio_frente, _, intentos, dists, marg = buscar_camino_libre(lidar_points, robot.radius, 'front', 0.05)
                if not espacio_frente:
                    espacio_frente, _, intentos, dists, marg = buscar_camino_libre(lidar_points, robot.radius, 'front', 0.0)
                
                intentos_render = intentos
                render_distancias = dists
                render_margen = marg
                
                # A c_w_turn rad/s, 90 grados toman cierto tiempo.
                if tiempo_estado >= c_time_min_g and espacio_frente: 
                    estado_actual = "EXPLORANDO"
                    cooldown_senal = c_cool_post
                # Evitar girar infinitamente
                elif tiempo_estado >= c_time_max_g:
                    estado_actual = "EXPLORANDO"
                    cooldown_senal = c_cool_post

            elif estado_actual == "DETENIDO":
                v_target = 0.0
                w_target = 0.0
                tiempo_estado -= dt
                if tiempo_estado <= 0:
                    estado_actual = "EXPLORANDO"
                    cooldown_senal = c_cool_stop

            elif estado_actual == "FINALIZADO":
                # Meta alcanzada: se queda detenido, sin volver a EXPLORANDO.
                v_target = 0.0
                w_target = 0.0

            # ========================================================
            # 3. ANTI-CHOQUES Y EVASIÓN (Giro estático hasta ruta libre)
            # ========================================================
            riesgo_inminente = False
            min_dist_frontal = float('inf')
            for i in list(range(0, 45)) + list(range(315, 360)):
                if lidar_scan[i] < min_dist_frontal:
                    min_dist_frontal = lidar_scan[i]
            
            # Umbral de choque frontal (0.35m por defecto o de config)
            if min_dist_frontal < c_evas_f and v_target > 0.05:
                riesgo_inminente = True
                
            if riesgo_inminente or estado_actual == "EVASION_EMERGENCIA":
                if estado_actual != "EVASION_EMERGENCIA":
                    tiempo_estado = 0.0 # Reset de tiempo de atasco
                estado_actual = "EVASION_EMERGENCIA"
                
                v_target = 0.0 # Detenemos el robot
                tiempo_estado += dt
                
                # Buscamos escape SOLO al frente con un margen muy pequeño (casi el radio exacto)
                esp_escape, ang_escape, intentos, dists, marg = buscar_camino_libre(lidar_points, robot.radius, 'front', 0.02)
                intentos_render = intentos
                render_distancias = dists
                render_margen = marg
                
                if esp_escape:
                    ang_rel = ang_escape if ang_escape <= 180 else ang_escape - 360
                    w_target = math.radians(ang_rel) * 4.0
                    
                    # Si ya estamos alineados y el frente está libre, salimos de emergencia
                    if abs(ang_rel) < 15 and dist_frente_estricto > 0.4:
                        estado_actual = "EXPLORANDO"
                        cooldown_senal = c_cool_post # Evitar volver a leer el cartel que nos metió en problemas
                        v_target = c_min_v # Solución al bug de estancamiento (fuerza un pequeño empuje para escapar del loop)
                else:
                    # Si no hay salida al frente, rotamos físicamente sobre nuestro eje hacia donde fue el último giro
                    w_target = 3.0 if ultimo_giro == 'left' else -3.0

            # ========================================================
            # 4. ACTUACIÓN FÍSICA Y GUARDADO
            # ========================================================
            hubo_choque = robot.move(v_target, w_target, dt)
            if hubo_choque:
                choques += 1

            if not use_simulator and estado_actual == "FINALIZADO":
                print("\n\n¡Señal FINISH alcanzada! Deteniendo el robot y terminando el programa.")
                break

            time_since_save += dt
            if time_since_save >= 0.5:
                history.append(get_current_state())
                history_index = len(history) - 1
                time_since_save = 0.0
        else:
            # En pausa, renderizamos el sweep correspondiente al estado congelado
            v_target = 0.0
            w_target = 0.0
            
            dir_search = 'front'
            margen = 0.10
            if estado_actual == "BUSCANDO_IZQ": dir_search = 'left'
            elif estado_actual == "BUSCANDO_DER": dir_search = 'right'
            elif estado_actual == "EVASION_EMERGENCIA": dir_search = 'any'; margen = 0.15
            
            # Crear lidar_points estático para la pausa
            lidar_points = []
            for i, dist_p in enumerate(lidar_scan):
                if dist_p < robot.lidar_max_range:
                    lidar_points.append((dist_p * math.cos(math.radians(i)), dist_p * math.sin(math.radians(i))))
                    
            esp, _, intentos_render, render_distancias, render_margen = buscar_camino_libre(lidar_points, robot.radius, dir_search, margen)
            if not esp and margen >= 0.10:
                esp, _, intentos_render, render_distancias, render_margen = buscar_camino_libre(lidar_points, robot.radius, dir_search, 0.05)
            if not esp and margen >= 0.05:
                _, _, intentos_render, render_distancias, render_margen = buscar_camino_libre(lidar_points, robot.radius, dir_search, 0.0)

        # ========================================================
        # 5. RENDERIZADO VISUAL
        # ========================================================
        screen.fill((30, 30, 30))
        font = pygame.font.SysFont(None, 24)
        
        if use_simulator:
            rx, ry, rtheta = robot._get_true_pose()
        
        if view_mode == "global" and use_simulator:
            # --- RENDER GLOBAL (Original) ---
            for p1, p2 in world.obstacles:
                pygame.draw.line(screen, (200, 200, 200), to_screen(*p1), to_screen(*p2), 2)

            for sig in world.signals:
                sx, sy = to_screen(sig['x'], sig['y'])
                pygame.draw.circle(screen, (255, 255, 0), (sx, sy), 5)
                img = font.render(sig['type'], True, (255, 255, 0))
                screen.blit(img, (sx + 10, sy - 10))

            rsx, rsy = to_screen(rx, ry)

            # DIBUJAR INTENTOS DEL SWEEP ALGORÍTMICO
            for intento in intentos_render:
                ang_c = intento['angulo']
                es_valido = intento['valido']
                color_circulo = (0, 255, 255) if es_valido else (255, 0, 0)
                
                for d_c in render_distancias:
                    cx = rx + d_c * math.cos(rtheta + math.radians(ang_c))
                    cy = ry + d_c * math.sin(rtheta + math.radians(ang_c))
                    scx, scy = to_screen(cx, cy)
                    pygame.draw.circle(screen, color_circulo, (scx, scy), int((render_margen)*SCALE), 1)

            angle_increment = (2 * math.pi) / robot.lidar_resolution
            for i, dist in enumerate(lidar_scan):
                if dist < robot.lidar_max_range:
                    ray_angle = rtheta + i * angle_increment
                    end_x = rx + dist * math.cos(ray_angle)
                    end_y = ry + dist * math.sin(ray_angle)
                    esx, esy = to_screen(end_x, end_y)
                    
                    if dist < 0.35 and (i < 120 or i > 240):
                        if dist < 0.20:
                            pygame.draw.circle(screen, (255, 0, 0), (esx, esy), 2)
                        else:
                            pygame.draw.circle(screen, (255, 165, 0), (esx, esy), 2)
                    else:
                        pygame.draw.circle(screen, (0, 255, 0), (esx, esy), 1)

            fov_l = rtheta + robot.camera_fov / 2
            fov_r = rtheta - robot.camera_fov / 2
            fl_x, fl_y = to_screen(rx + 2 * math.cos(fov_l), ry + 2 * math.sin(fov_l))
            fr_x, fr_y = to_screen(rx + 2 * math.cos(fov_r), ry + 2 * math.sin(fov_r))
            pygame.draw.line(screen, (0, 100, 255), (rsx, rsy), (fl_x, fl_y), 1)
            pygame.draw.line(screen, (0, 100, 255), (rsx, rsy), (fr_x, fr_y), 1)

            robot_px_radius = int(robot.radius * SCALE)
            
            # Anillo rojo (Límite de colisión lateral)
            pygame.draw.circle(screen, (200, 50, 50), (rsx, rsy), int(c_evas_g * SCALE), 1)
            # Anillo naranja (Límite de evasión frontal)
            pygame.draw.circle(screen, (255, 100, 0), (rsx, rsy), int(c_evas_f * SCALE), 1)
            # Anillo amarillo oscuro (Límite de giro fuerte/repulsión)
            pygame.draw.circle(screen, (200, 180, 50), (rsx, rsy), int(c_rad_giro_f * SCALE), 1)
            # Anillo amarillo claro (Límite de giro suave/exploración)
            pygame.draw.circle(screen, (255, 255, 0), (rsx, rsy), int(c_rad_amarillo * SCALE), 1)
            
            pygame.draw.circle(screen, (50, 255, 100), (rsx, rsy), robot_px_radius)
            hx, hy = to_screen(rx + robot.radius * math.cos(rtheta), ry + robot.radius * math.sin(rtheta))
            pygame.draw.line(screen, (255, 255, 255), (rsx, rsy), (hx, hy), 3)

        else:
            # --- RENDER VISTA ROBOT (Local) ---
            rsx, rsy = OFFSET_X, OFFSET_Y
            
            # Dibujar LiDAR (centrado, mirando hacia arriba)
            angle_increment = (2 * math.pi) / robot.lidar_resolution
            for i, dist in enumerate(lidar_scan):
                if dist < robot.lidar_max_range:
                    # Sumamos el ángulo para invertir el render y que izquierda sea izquierda
                    screen_angle = math.pi / 2 + i * angle_increment
                    esx = int(rsx + dist * math.cos(screen_angle) * SCALE)
                    esy = int(rsy - dist * math.sin(screen_angle) * SCALE)
                    
                    if dist < 0.35 and (i < 120 or i > 240):
                        if dist < 0.20:
                            pygame.draw.circle(screen, (255, 0, 0), (esx, esy), 3)
                        else:
                            pygame.draw.circle(screen, (255, 165, 0), (esx, esy), 2)
                    else:
                        pygame.draw.circle(screen, (0, 255, 0), (esx, esy), 1)

            # Dibujar FOV
            fov_l = math.pi/2 + robot.camera_fov / 2
            fov_r = math.pi/2 - robot.camera_fov / 2
            fl_x = int(rsx + 2 * math.cos(fov_l) * SCALE)
            fl_y = int(rsy - 2 * math.sin(fov_l) * SCALE)
            fr_x = int(rsx + 2 * math.cos(fov_r) * SCALE)
            fr_y = int(rsy - 2 * math.sin(fov_r) * SCALE)
            pygame.draw.line(screen, (0, 100, 255), (rsx, rsy), (fl_x, fl_y), 1)
            pygame.draw.line(screen, (0, 100, 255), (rsx, rsy), (fr_x, fr_y), 1)

            # Dibujar Robot
            robot_px_radius = int(robot.radius * SCALE)
            pygame.draw.circle(screen, (50, 255, 100), (rsx, rsy), robot_px_radius)
            hx, hy = rsx, rsy - robot_px_radius # Frente hacia arriba
            pygame.draw.line(screen, (255, 255, 255), (rsx, rsy), (hx, hy), 3)
            
            # Dibujar señales detectadas por YOLO
            for det in vision_dets:
                dist = det['distance']
                rel_a = det['relative_angle']
                screen_angle = math.pi / 2 + rel_a
                
                sx = int(rsx + dist * math.cos(screen_angle) * SCALE)
                sy = int(rsy - dist * math.sin(screen_angle) * SCALE)
                
                pygame.draw.circle(screen, (255, 255, 0), (sx, sy), 8)
                img = font.render(det['class'], True, (255, 255, 0))
                screen.blit(img, (sx + 15, sy - 10))

        mode_text = "SIMULADOR" if use_simulator else f"ROBOT REAL (FPS: {1.0/max(0.001, dt):.1f})"
        screen.blit(pygame.font.SysFont(None, 36).render(f"[{mode_text}] Algoritmo: {estado_actual}", True, (255, 255, 255)), (10, 10))
        if cooldown_senal > 0:
            screen.blit(pygame.font.SysFont(None, 24).render(f"(Ignorando señales por: {cooldown_senal:.1f}s para evitar bucle)", True, (255, 200, 0)), (400, 15))
            
        screen.blit(pygame.font.SysFont(None, 28).render(f"v={v_target:.2f}, w={w_target:.2f}", True, (200, 200, 200)), (10, 45))

        color_choque = (255, 100, 100) if choques > 0 else (100, 255, 100)
        screen.blit(pygame.font.SysFont(None, 28).render(f"Choques: {choques}", True, color_choque), (10, 75))

        y_offset = 105
        for det in vision_dets:
            text = f"YOLO Ve: '{det['class']}' a {det['distance']:.2f}m (Ang: {math.degrees(det['relative_angle']):.1f}º)"
            img = font.render(text, True, (255, 100, 100))
            screen.blit(img, (10, y_offset))
            y_offset += 25

        if paused:
            pause_text = pygame.font.SysFont(None, 48).render("PAUSADO - USA FLECHAS (<- ->) PARA TIEMPO", True, (255, 100, 100))
            screen.blit(pause_text, (WIDTH//2 - pause_text.get_width()//2, HEIGHT - 80))
            
            state_text = pygame.font.SysFont(None, 36).render(f"Snapshot: {history_index+1} / {len(history)}", True, (255, 200, 100))
            screen.blit(state_text, (WIDTH//2 - state_text.get_width()//2, HEIGHT - 40))

        pygame.display.flip()
        clock.tick(30)

if __name__ == "__main__":
    main()
