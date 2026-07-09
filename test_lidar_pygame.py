import sys
import pygame
import math
import time

# Importamos el controlador del robot real (sin inicializar YOLO)
from TurtleBotController.turtlebot import TurtleBotReal

def main():
    pygame.init()
    WIDTH, HEIGHT = 800, 800
    OFFSET_X, OFFSET_Y = WIDTH // 2, HEIGHT // 2
    SCALE = 100.0  # 100 pixeles = 1 metro (bastante zoom para ver detalles de cerca)
    
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Visualizador Puro de LiDAR - TurtleBot")
    clock = pygame.time.Clock()

    print("Conectando al ROS 2 del TurtleBot...")
    # Aseguramos que YOLO no inicie para evitar sobrecargar la Raspberry Pi o la cámara
    robot = TurtleBotReal("config.json", use_yolo=False)
    
    print("¡Listo! Mostrando ventana de Pygame.")
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                
        # 1. Obtener la data cruda del LiDAR (Arreglo de 360 valores en metros)
        lidar_scan = robot.get_lidar_scan()
        
        # Si todavía no ha llegado el primer mensaje de ROS, esperamos
        if len(lidar_scan) < 360:
            time.sleep(0.05)
            continue
            
        # 2. Empezar a dibujar
        screen.fill((10, 15, 10)) # Fondo oscuro tipo radar
        
        # Dibujar anillos de distancia como referencia (0.5m, 1m, 1.5m, 2m)
        for d in [0.5, 1.0, 1.5, 2.0]:
            pygame.draw.circle(screen, (30, 50, 30), (OFFSET_X, OFFSET_Y), int(d * SCALE), 1)
            
        # Dibujar centro (representa al robot)
        robot_px_radius = int(robot.radius * SCALE)
        pygame.draw.circle(screen, (60, 220, 60), (OFFSET_X, OFFSET_Y), robot_px_radius)
        # Línea indicando el FRENTE del robot
        pygame.draw.line(screen, (10, 15, 10), (OFFSET_X, OFFSET_Y), (OFFSET_X, OFFSET_Y - robot_px_radius), 3)
        
        # Dibujar los puntos del LiDAR
        angle_increment = (2 * math.pi) / robot.lidar_resolution
        for i, dist in enumerate(lidar_scan):
            if dist < robot.lidar_max_range:
                # El ángulo 0 es el frente (índice 0 en la lista devuelta por get_lidar_scan)
                # En matemáticas de pantalla, el frente (Y negativo) es Pi/2. 
                screen_angle = math.pi / 2 + i * angle_increment
                
                # Coordenadas polares a cartesianas para Pygame
                esx = int(OFFSET_X + dist * math.cos(screen_angle) * SCALE)
                esy = int(OFFSET_Y - dist * math.sin(screen_angle) * SCALE)
                
                # Colorear los puntos según la cercanía (Alerta visual)
                if dist < 0.35: # Muy cerca
                    if dist < 0.20: # Riesgo Inminente de Choque
                        pygame.draw.circle(screen, (255, 50, 50), (esx, esy), 4) 
                    else: # Cerca
                        pygame.draw.circle(screen, (255, 165, 0), (esx, esy), 3) 
                else: # Seguro
                    pygame.draw.circle(screen, (50, 255, 50), (esx, esy), 2) 
                    
                # Dibujar una línea tenue desde el robot al punto para que parezca un láser real
                pygame.draw.line(screen, (20, 50, 20), (OFFSET_X, OFFSET_Y), (esx, esy), 1)

        # Mostrar FPS en pantalla
        fps = clock.get_fps()
        font = pygame.font.SysFont(None, 36)
        screen.blit(font.render(f"RADAR LiDAR (FPS: {fps:.1f})", True, (60, 220, 60)), (10, 10))
        
        pygame.display.flip()
        clock.tick(30)
        
if __name__ == "__main__":
    main()
