import pygame
import sys
import math
import os
import numpy as np

from Simulator.WorldSim.world import World
from Simulator.TurtleBotSim.turtlebot import TurtleBotMock

SCALE = 50.0
WIDTH, HEIGHT = 800, 600
OFFSET_X, OFFSET_Y = WIDTH // 2, HEIGHT // 2

def to_screen(x, y):
    return int(OFFSET_X + x * SCALE), int(OFFSET_Y - y * SCALE)

def buscar_camino_libre(lidar_points, radio_robot, direccion='front', margen_extra=0.10):
    # Aperturas optimizadas: 7 ángulos, M menor
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

    dt = 1 / 60.0
    running = True
    paused = False

    estado_actual = "EXPLORANDO"
    tiempo_estado = 0.0
    cooldown_senal = 0.0
    choques = 0
    
    v_target = 0.0
    w_target = 0.0

    history = []
    history_index = -1
    time_since_save = 0.0

    def get_current_state():
        rx, ry, rth = robot._get_true_pose()
        return {
            'x': rx,
            'y': ry,
            'theta': rth,
            'estado_actual': estado_actual,
            'tiempo_estado': tiempo_estado,
            'cooldown_senal': cooldown_senal,
            'choques': choques
        }
        
    def set_state(st):
        nonlocal estado_actual, tiempo_estado, cooldown_senal, choques
        robot._TurtleBotMock__x = st['x']
        robot._TurtleBotMock__y = st['y']
        robot._TurtleBotMock__theta = st['theta']
        estado_actual = st['estado_actual']
        tiempo_estado = st['tiempo_estado']
        cooldown_senal = st['cooldown_senal']
        choques = st.get('choques', choques)

    history.append(get_current_state())
    history_index = 0
    view_mode = "global"

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_l:
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

        # .tolist() es CRÍTICO: con ndarray, `scan[0:15] + scan[345:360]` SUMA
        # elemento a elemento (distancia frontal ~2x la real); con lista concatena,
        # que es lo que hace el robot real. Sin esto, el sim valida otra matemática.
        lidar_scan = robot.get_lidar_scan().tolist()
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
            # 1. ACTUALIZAR ESTADO SEGÚN YOLO 
            # ========================================================
            if len(vision_dets) > 0 and cooldown_senal <= 0:
                senal = sorted(vision_dets, key=lambda d: d['distance'])[0]
                clase = senal['class']
                dist = senal['distance']
                
                if estado_actual == "EXPLORANDO":
                    if clase == 'left':
                        estado_actual = "BUSCANDO_IZQ"
                    elif clase == 'right':
                        estado_actual = "BUSCANDO_DER"
                    elif clase == 'stop' and dist <= 1.6:
                        estado_actual = "DETENIDO"
                        tiempo_estado = 3.0

            # ========================================================
            # 2. LÓGICA DE CADA ESTADO
            # ========================================================
            if estado_actual == "EXPLORANDO":
                v_target = max(0.1, min(0.8, (dist_frente_estricto - 0.4) * 0.8))
                
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
                # Misma velocidad que explorando, la velocidad varía solo si nos vamos a estrellar
                v_target = max(0.1, min(0.8, (dist_frente_estricto - 0.4) * 0.8))
                
                # Centrar la flecha y avanzar
                if len(vision_dets) > 0:
                    senal = sorted(vision_dets, key=lambda d: d['distance'])[0]
                    w_target = senal['relative_angle'] * 2.5
                
                # Barrido de 40 grados hacia la izquierda o derecha
                dir_search = 'left' if estado_actual == "BUSCANDO_IZQ" else 'right'
                espacio, _, intentos, dists, marg = buscar_camino_libre(lidar_points, robot.radius, dir_search, 0.10)
                
                intentos_render = intentos
                render_distancias = dists
                render_margen = marg
                
                if espacio:
                    estado_actual = "GIRANDO_IZQ" if estado_actual == "BUSCANDO_IZQ" else "GIRANDO_DER"
                    tiempo_estado = 0.0

            elif estado_actual in ["GIRANDO_IZQ", "GIRANDO_DER"]:
                # Mantenemos la velocidad alta para no ir lento en el giro
                v_target = max(0.1, min(0.8, (dist_frente_estricto - 0.4) * 0.8))
                w_target = 2.0 if estado_actual == "GIRANDO_IZQ" else -2.0
                tiempo_estado += dt
                
                # Verificamos visualmente el frente (para debug en UI)
                espacio_frente, _, intentos, dists, marg = buscar_camino_libre(lidar_points, robot.radius, 'front', 0.10)
                intentos_render = intentos
                render_distancias = dists
                render_margen = marg
                
                # Girar exactamente 80 grados. 
                # 80 grados = 1.396 rad. A w=2.0 rad/s, tiempo = 0.7s
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
            # 3. ANTI-CHOQUES Y EVASIÓN (Giro estático hasta ruta libre)
            # ========================================================
            riesgo_inminente = False
            min_dist_frontal = float('inf')
            for i in list(range(0, 45)) + list(range(315, 360)):
                if lidar_scan[i] < min_dist_frontal:
                    min_dist_frontal = lidar_scan[i]
            
            # Umbral de choque frontal (0.32m)
            if min_dist_frontal < 0.32 and v_target > 0.05:
                riesgo_inminente = True
            # Umbral general reducido a 0.19m (0.17 radio + 0.02 ruido) para evitar falsos positivos por ruido
            if min(lidar_scan) < 0.19:
                riesgo_inminente = True # Peligro de roce físico real
                
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
                else:
                    # Si no hay salida al frente, rotamos físicamente sobre nuestro eje
                    w_target = 3.0

            # ========================================================
            # 4. ACTUACIÓN FÍSICA Y GUARDADO
            # ========================================================
            hubo_choque = robot.move(v_target, w_target, dt)
            if hubo_choque:
                choques += 1

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
                    
            _, _, intentos_render, render_distancias, render_margen = buscar_camino_libre(lidar_points, robot.radius, dir_search, margen)

        # ========================================================
        # 5. RENDERIZADO VISUAL
        # ========================================================
        screen.fill((30, 30, 30))
        font = pygame.font.SysFont(None, 24)
        
        rx, ry, rtheta = robot._get_true_pose()
        
        if view_mode == "global":
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
                    
                    if dist < 0.7 and (i < 120 or i > 240):
                        if dist < 0.4:
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
                    # En vista local, theta=0 es hacia arriba. (en pygame y crece hacia abajo)
                    screen_angle = math.pi / 2 - i * angle_increment
                    esx = int(rsx + dist * math.cos(screen_angle) * SCALE)
                    esy = int(rsy - dist * math.sin(screen_angle) * SCALE)
                    
                    if dist < 0.7 and (i < 120 or i > 240):
                        if dist < 0.4:
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
                screen_angle = math.pi / 2 - rel_a
                
                sx = int(rsx + dist * math.cos(screen_angle) * SCALE)
                sy = int(rsy - dist * math.sin(screen_angle) * SCALE)
                
                pygame.draw.circle(screen, (255, 255, 0), (sx, sy), 8)
                img = font.render(det['class'], True, (255, 255, 0))
                screen.blit(img, (sx + 15, sy - 10))

        screen.blit(pygame.font.SysFont(None, 36).render(f"Algoritmo: {estado_actual}", True, (255, 255, 255)), (10, 10))
        if cooldown_senal > 0:
            screen.blit(pygame.font.SysFont(None, 24).render(f"(Ignorando señales por: {cooldown_senal:.1f}s para evitar bucle)", True, (255, 200, 0)), (350, 15))
            
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
        clock.tick(60)

if __name__ == "__main__":
    main()
