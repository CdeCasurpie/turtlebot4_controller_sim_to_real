import pygame
import sys
import math
import os
import argparse

from Simulator.WorldSim.world import World
from Simulator.TurtleBotSim.turtlebot import TurtleBotMock

# Configuraciones de renderizado
SCALE = 50.0  # Píxeles por metro
WIDTH, HEIGHT = 800, 600
OFFSET_X, OFFSET_Y = WIDTH // 2, HEIGHT // 2

def to_screen(x, y):
    """Convierte coordenadas en metros a coordenadas de pantalla."""
    return int(OFFSET_X + x * SCALE), int(OFFSET_Y - y * SCALE)

def to_world(sx, sy):
    """Convierte coordenadas de pantalla a metros en el mundo."""
    return (sx - OFFSET_X) / SCALE, (OFFSET_Y - sy) / SCALE

def point_to_segment_distance(px, py, x1, y1, x2, y2):
    """Calcula la distancia mínima de un punto a un segmento y el punto de proyección."""
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return math.hypot(px - x1, py - y1), (x1, y1)
    
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    closest_x = x1 + t * dx
    closest_y = y1 + t * dy
    return math.hypot(px - closest_x, py - closest_y), (closest_x, closest_y)

def editor_mode(screen, world, clock):
    font = pygame.font.SysFont(None, 24)
    running = True
    current_polygon = []
    
    signal_mode = None

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_c and current_polygon:
                    world.add_obstacle_polygon(current_polygon)
                    current_polygon = []
                elif event.key == pygame.K_s or event.key == pygame.K_RETURN:
                    # Guardar el mapa y salir del editor
                    world.save_to_file("world_map.json")
                    return
                elif event.key == pygame.K_1: signal_mode = 'left'
                elif event.key == pygame.K_2: signal_mode = 'right'
                elif event.key == pygame.K_3: signal_mode = 'stop'
                elif event.key == pygame.K_r: signal_mode = 'robot_pos'
                elif event.key == pygame.K_t: signal_mode = 'robot_theta'
                
            elif event.type == pygame.KEYUP:
                if event.key in [pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_r, pygame.K_t]:
                    signal_mode = None
                    
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1: # Click Izquierdo
                    wx, wy = to_world(event.pos[0], event.pos[1])
                    
                    if signal_mode == 'robot_pos':
                        world.robot_start['x'] = wx
                        world.robot_start['y'] = wy
                    elif signal_mode == 'robot_theta':
                        dx = wx - world.robot_start['x']
                        dy = wy - world.robot_start['y']
                        world.robot_start['theta'] = math.atan2(dy, dx)
                    elif signal_mode in ['left', 'right', 'stop'] and world.obstacles:
                        # Pegar la flecha al muro más cercano
                        min_dist = float('inf')
                        best_pt = None
                        for p1, p2 in world.obstacles:
                            dist, pt = point_to_segment_distance(wx, wy, p1[0], p1[1], p2[0], p2[1])
                            if dist < min_dist:
                                min_dist = dist
                                best_pt = pt
                        if best_pt:
                            world.add_signal(signal_mode, best_pt[0], best_pt[1])
                    elif signal_mode is None:
                        current_polygon.append((wx, wy))

        screen.fill((20, 20, 20))
        
        # Dibujar muros ya creados
        for p1, p2 in world.obstacles:
            pygame.draw.line(screen, (200, 200, 200), to_screen(*p1), to_screen(*p2), 2)
            
        # Dibujar el muro en construcción
        if len(current_polygon) > 0:
            pts = [to_screen(*pt) for pt in current_polygon]
            for pt in pts:
                pygame.draw.circle(screen, (100, 255, 100), pt, 4)
            if len(pts) > 1:
                pygame.draw.lines(screen, (100, 255, 100), False, pts, 2)
                
        # Dibujar señales
        for sig in world.signals:
            sx, sy = to_screen(sig['x'], sig['y'])
            pygame.draw.circle(screen, (255, 255, 0), (sx, sy), 5)
            img = font.render(sig['type'], True, (255, 255, 0))
            screen.blit(img, (sx + 10, sy - 10))
            
        # Dibujar el punto de inicio del robot (transparente/hueco)
        rx = world.robot_start['x']
        ry = world.robot_start['y']
        rtheta = world.robot_start['theta']
        rsx, rsy = to_screen(rx, ry)
        robot_px_radius = int(0.17 * SCALE)
        pygame.draw.circle(screen, (255, 100, 100), (rsx, rsy), robot_px_radius, 1)
        pygame.draw.circle(screen, (255, 100, 100), (rsx, rsy), 3) # Centro
        hx, hy = to_screen(rx + 0.17 * math.cos(rtheta), ry + 0.17 * math.sin(rtheta))
        pygame.draw.line(screen, (255, 255, 255), (rsx, rsy), (hx, hy), 2)
            
        # Textos informativos
        text = "EDITOR | Click: Muro | C: Cerrar | Enter: Guardar y Jugar"
        text2 = "[1,2,3]+Click: Señales | [R]+Click: Pos Robot | [T]+Click: Orientación Robot"
        if signal_mode:
            if signal_mode == 'robot_pos': text2 = "Modificando Posición del Robot (Haz click)"
            elif signal_mode == 'robot_theta': text2 = "Modificando Orientación del Robot (Haz click hacia donde mirará)"
            else: text2 = f"Poniendo señal: {signal_mode} (Haz click cerca de un muro)"
            
        img1 = font.render(text, True, (255, 255, 255))
        img2 = font.render(text2, True, (200, 200, 255))
        screen.blit(img1, (10, HEIGHT - 50))
        screen.blit(img2, (10, HEIGHT - 25))
        
        pygame.display.flip()
        clock.tick(60)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--new', action='store_true', help="Abre el editor vacío para dibujar un nuevo mapa.")
    parser.add_argument('--edit', action='store_true', help="Abre el editor con el mapa actual cargado para añadir cosas.")
    args = parser.parse_args()

    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("TurtleBot 4 - Simulador 2D")
    clock = pygame.time.Clock()

    world = World()
    map_file = "world_map.json"

    # Lógica de carga y editor
    if args.new:
        print("Iniciando mapa vacío...")
        editor_mode(screen, world, clock)
    elif args.edit:
        if os.path.exists(map_file):
            print(f"Cargando {map_file} para edición...")
            world.load_from_file(map_file)
        else:
            print("No existe un mapa anterior, iniciando vacío.")
        editor_mode(screen, world, clock)
    elif not os.path.exists(map_file):
        print("No existe un mapa inicial, abriendo editor...")
        editor_mode(screen, world, clock)
    else:
        world.load_from_file(map_file)
        print(f"Mapa cargado exitosamente desde {map_file}")

    # Inicializar el Robot Mock con la pose del mundo
    robot = TurtleBotMock(
        world, 
        initial_x=world.robot_start['x'], 
        initial_y=world.robot_start['y'], 
        initial_theta=world.robot_start['theta']
    )

    # Parámetros del bucle
    dt = 1 / 60.0
    running = True
    lidar_mode = 1 # 0 = Oculto, 1 = Líneas, 2 = Puntos
    
    # Velocidades actuales
    v = 0.0
    omega = 0.0

    print("Controles de Simulación:")
    print(" - Flecha ARRIBA / ABAJO : Velocidad Lineal")
    print(" - Flecha IZQ / DER : Velocidad Angular")
    print(" - Tecla 'L' : Cambiar modo de visualización LiDAR (Líneas/Puntos/Oculto)")

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_l:
                    lidar_mode = (lidar_mode + 1) % 3

        # Control manual (Teclado)
        keys = pygame.key.get_pressed()
        v_target = 0.0
        omega_target = 0.0
        
        if keys[pygame.K_UP]: v_target = 0.3
        elif keys[pygame.K_DOWN]: v_target = -0.3
            
        if keys[pygame.K_LEFT]: omega_target = 1.9
        elif keys[pygame.K_RIGHT]: omega_target = -1.9

        # Asignación directa de velocidad
        v = v_target
        omega = omega_target

        # Planta
        robot.move(v, omega, dt)

        # Sensores
        lidar_scan = robot.get_lidar_scan()
        vision_dets = robot.get_vision_detections()

        # Renderizado Visual
        screen.fill((30, 30, 30))

        # 1. Dibujar Obstáculos
        for p1, p2 in world.obstacles:
            pygame.draw.line(screen, (200, 200, 200), to_screen(*p1), to_screen(*p2), 2)

        # 2. Dibujar Señales
        font = pygame.font.SysFont(None, 24)
        for sig in world.signals:
            sx, sy = to_screen(sig['x'], sig['y'])
            pygame.draw.circle(screen, (255, 255, 0), (sx, sy), 5)
            img = font.render(sig['type'], True, (255, 255, 0))
            screen.blit(img, (sx + 10, sy - 10))

        rx, ry, rtheta = robot._get_true_pose()
        rsx, rsy = to_screen(rx, ry)

        # 3. Dibujar LiDAR
        if lidar_mode > 0:
            angle_increment = (2 * math.pi) / robot.lidar_resolution
            for i, dist in enumerate(lidar_scan):
                if dist < robot.lidar_max_range:
                    ray_angle = rtheta + i * angle_increment
                    end_x = rx + dist * math.cos(ray_angle)
                    end_y = ry + dist * math.sin(ray_angle)
                    esx, esy = to_screen(end_x, end_y)
                    
                    if lidar_mode == 1:
                        # Modo líneas
                        pygame.draw.line(screen, (0, 150, 0), (rsx, rsy), (esx, esy), 1)
                    elif lidar_mode == 2:
                        # Modo puntos (hits)
                        pygame.draw.circle(screen, (0, 255, 0), (esx, esy), 2)

        # 4. Dibujar FOV
        fov_l = rtheta + robot.camera_fov / 2
        fov_r = rtheta - robot.camera_fov / 2
        fl_x, fl_y = to_screen(rx + 2 * math.cos(fov_l), ry + 2 * math.sin(fov_l))
        fr_x, fr_y = to_screen(rx + 2 * math.cos(fov_r), ry + 2 * math.sin(fov_r))
        pygame.draw.line(screen, (0, 100, 255), (rsx, rsy), (fl_x, fl_y), 1)
        pygame.draw.line(screen, (0, 100, 255), (rsx, rsy), (fr_x, fr_y), 1)

        # 5. Dibujar el Robot
        robot_px_radius = int(robot.radius * SCALE)
        pygame.draw.circle(screen, (255, 50, 50), (rsx, rsy), robot_px_radius)
        hx, hy = to_screen(rx + robot.radius * math.cos(rtheta), ry + robot.radius * math.sin(rtheta))
        pygame.draw.line(screen, (255, 255, 255), (rsx, rsy), (hx, hy), 3)

        # 6. Interfaz HUD
        y_offset = 10
        
        # HUD Velocidades
        vel_text = f"Velocidad: v = {v:.2f} m/s | omega = {omega:.2f} rad/s"
        img_vel = font.render(vel_text, True, (255, 255, 255))
        screen.blit(img_vel, (10, y_offset))
        y_offset += 25
        
        # HUD LiDAR Mode
        mode_str = ["Oculto", "Líneas", "Puntos"][lidar_mode]
        img_lidar = font.render(f"LiDAR Mode (L): {mode_str}", True, (200, 200, 200))
        screen.blit(img_lidar, (10, y_offset))
        y_offset += 30

        # HUD Visión
        for det in vision_dets:
            text = f"YOLO Ve: '{det['class']}' a {det['distance']:.2f}m (Ang: {math.degrees(det['relative_angle']):.1f}º)"
            img = font.render(text, True, (255, 100, 100))
            screen.blit(img, (10, y_offset))
            y_offset += 25

        pygame.display.flip()
        clock.tick(60)

if __name__ == "__main__":
    main()
