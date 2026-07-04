import cProfile
import pstats
import math
import numpy as np

# Mock data
lidar_scan = [1.0] * 360
robot_radius = 0.17

def run_algo():
    for _ in range(100):
        # Lidar points conversion
        lidar_points = []
        for i, dist_p in enumerate(lidar_scan):
            if dist_p < 3.5:
                lidar_points.append((dist_p * math.cos(math.radians(i)), dist_p * math.sin(math.radians(i))))
        
        # Calcular repulsion
        fx, fy = 0.0, 0.0
        for px, py in lidar_points:
            dist = math.hypot(px, py)
            if dist < 0.35:
                pass
                
        # min dist frontal
        min_dist_frontal = float('inf')
        for i in list(range(0, 21)) + list(range(339, 360)):
            if lidar_scan[i] < min_dist_frontal:
                min_dist_frontal = lidar_scan[i]

cProfile.run('run_algo()', 'profile_stats')
p = pstats.Stats('profile_stats')
p.sort_stats('time').print_stats(10)
