# -*- coding: utf-8 -*-
"""
Visualiza el CONTROLADOR UNIFICADO (TurtleBotController/autonomous_controller.py)
en el simulador pygame — exactamente el mismo código que corre en el robot real
(run_real_autonomous.py). Si se ajusta un parámetro aquí y funciona, funciona
igual a bordo: no hay dos implementaciones.

Controles: ESPACIO pausa | R reinicia | ESC sale.
"""
import math
import os
import sys

import numpy as np
import pygame

from Simulator.WorldSim.world import World
from Simulator.TurtleBotSim.turtlebot import TurtleBotMock
from TurtleBotController.autonomous_controller import AutonomousController

SCALE = 50.0
WIDTH, HEIGHT = 800, 600
OFFSET_X, OFFSET_Y = WIDTH // 2, HEIGHT // 2


def to_screen(x, y):
    return int(OFFSET_X + x * SCALE), int(OFFSET_Y - y * SCALE)


def crear_robot(world):
    return TurtleBotMock(
        world,
        initial_x=world.robot_start['x'],
        initial_y=world.robot_start['y'],
        initial_theta=world.robot_start['theta'],
    )


def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("TurtleBot 4 - Controlador Unificado (sim = robot)")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 16)

    world = World()
    map_file = "world_map.json"
    if not os.path.exists(map_file):
        print("No se encontró world_map.json")
        sys.exit(1)
    world.load_from_file(map_file)

    robot = crear_robot(world)
    ctrl = AutonomousController({"v_max": 0.30})

    dt = 0.05  # 20 Hz: el mismo periodo de control que en el robot real
    running, paused = True, False
    choques = 0
    t_sim = 0.0

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_r:
                    robot = crear_robot(world)
                    ctrl = AutonomousController({"v_max": 0.30})
                    choques, t_sim = 0, 0.0

        if not paused:
            # .tolist(): el controlador espera lista/array 1D; en el robot real
            # get_lidar_scan ya retorna lista.
            scan = robot.get_lidar_scan().tolist()
            dets = robot.get_vision_detections()
            x, y, theta = robot._get_true_pose()  # hace de /odom en el mock

            v, w, estado = ctrl.step(scan, dets, dt, yaw=theta, pose=(x, y))
            if robot.move(v, w, dt):
                choques += 1
            t_sim += dt

        # ---------------- render ----------------
        screen.fill((25, 25, 30))
        for p1, p2 in world.obstacles:
            pygame.draw.line(screen, (200, 200, 200), to_screen(*p1), to_screen(*p2), 2)
        for sig in world.signals:
            color = {'left': (0, 200, 0), 'right': (60, 120, 255), 'stop': (230, 40, 40)}.get(sig['type'], (255, 255, 0))
            pygame.draw.circle(screen, color, to_screen(sig['x'], sig['y']), 6)

        x, y, theta = robot._get_true_pose()
        px, py = to_screen(x, y)
        pygame.draw.circle(screen, (255, 180, 0), (px, py), int(robot.radius * SCALE))
        hx, hy = to_screen(x + 0.3 * math.cos(theta), y + 0.3 * math.sin(theta))
        pygame.draw.line(screen, (255, 255, 255), (px, py), (hx, hy), 3)

        hud = f"t={t_sim:6.1f}s  estado={ctrl.estado:<18} choques={choques}"
        screen.blit(font.render(hud, True, (255, 255, 255)), (10, 10))
        if paused:
            screen.blit(font.render("PAUSA (ESPACIO)", True, (255, 100, 100)), (10, 30))

        pygame.display.flip()
        clock.tick(60 if paused else 20)

    pygame.quit()


if __name__ == "__main__":
    main()
