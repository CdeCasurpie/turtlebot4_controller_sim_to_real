import cProfile
import pstats
import math
import random
import numpy as np
import json

def cast_ray(origin, angle, obstacles, max_dist=3.5):
    ox, oy = origin
    dx, dy = math.cos(angle), math.sin(angle)
    closest_dist = max_dist
    for p1, p2 in obstacles:
        x1, y1 = p1
        x2, y2 = p2
        den = (x1 - x2) * dy - (y1 - y2) * dx
        if den == 0:
            continue
        t = ((x1 - ox) * dy - (y1 - oy) * dx) / den
        u = -((x1 - x2) * (y1 - oy) - (y1 - y2) * (x1 - ox)) / den
        if 0 <= t <= 1 and u > 0:
            if u < closest_dist:
                closest_dist = u
    return closest_dist

with open('world_map.json', 'r') as f:
    world_map = json.load(f)
obstacles = world_map['obstacles']

def run_lidar():
    for _ in range(30): # 30 frames (1 second of sim)
        for i in range(360):
            ray_angle = i * math.pi / 180
            cast_ray((0, 0), ray_angle, obstacles, 3.5)

cProfile.run('run_lidar()', 'profile_stats')
p = pstats.Stats('profile_stats')
p.sort_stats('time').print_stats(10)
