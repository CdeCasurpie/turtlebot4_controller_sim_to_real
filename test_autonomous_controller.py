import pygame
import sys
import math
import os
import numpy as np

from Simulator.WorldSim.world import World
from Simulator.TurtleBotSim.turtlebot import TurtleBotMock
from controller.navigation import NavigationController, buscar_camino_libre

SCALE = 50.0
WIDTH, HEIGHT = 800, 600
OFFSET_X, OFFSET_Y = WIDTH // 2, HEIGHT // 2

def to_screen(x, y):
    return int(OFFSET_X + x * SCALE), int(OFFSET_Y - y * SCALE)

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

    controller = NavigationController(
        robot_radius=robot.radius,
        lidar_max_range=robot.lidar_max_range,
        v_max=0.8,
        lidar_min_valid=None,
        collect_debug=True,  # barridos extra para el render de debug
    )

    dt = 1 / 60.0
    running = True
    paused = False

    choques = 0

    v_target = 0.0
    w_target = 0.0

    history = []
    history_index = -1
    time_since_save = 0.0

    def get_current_state():
        rx, ry, rth = robot._get_true_pose()
        st = {
            'x': rx,
            'y': ry,
            'theta': rth,
            'choques': choques
        }
        st.update(controller.snapshot())
        return st

    def set_state(st):
        nonlocal choques
        robot._TurtleBotMock__x = st['x']
        robot._TurtleBotMock__y = st['y']
        robot._TurtleBotMock__theta = st['theta']
        controller.restore(st)
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

        lidar_scan = robot.get_lidar_scan()
        vision_dets = robot.get_vision_detections()

        intentos_render = []
        render_distancias = []
        render_margen = 0

        if not paused:
            v_target, w_target = controller.step(lidar_scan, vision_dets, dt)

            intentos_render = controller.debug['intentos']
            render_distancias = controller.debug['distancias']
            render_margen = controller.debug['margen']

            # ========================================================
            # ACTUACIÓN FÍSICA Y GUARDADO
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
            if controller.estado == "BUSCANDO_IZQ": dir_search = 'left'
            elif controller.estado == "BUSCANDO_DER": dir_search = 'right'
            elif controller.estado == "EVASION_EMERGENCIA": dir_search = 'any'; margen = 0.15

            # Crear lidar_points estático para la pausa
            lidar_points = []
            for i, dist_p in enumerate(lidar_scan):
                if dist_p < robot.lidar_max_range:
                    lidar_points.append((dist_p * math.cos(math.radians(i)), dist_p * math.sin(math.radians(i))))

            _, _, intentos_render, render_distancias, render_margen = buscar_camino_libre(lidar_points, robot.radius, dir_search, margen)

        # ========================================================
        # RENDERIZADO VISUAL
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

        screen.blit(pygame.font.SysFont(None, 36).render(f"Algoritmo: {controller.estado}", True, (255, 255, 255)), (10, 10))
        if controller.cooldown_senal > 0:
            screen.blit(pygame.font.SysFont(None, 24).render(f"(Ignorando señales por: {controller.cooldown_senal:.1f}s para evitar bucle)", True, (255, 200, 0)), (350, 15))

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
