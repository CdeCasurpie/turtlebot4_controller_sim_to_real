import math
from typing import List, Tuple, Optional

def ray_segment_intersection(
    ray_origin: Tuple[float, float],
    ray_angle: float,
    segment_p1: Tuple[float, float],
    segment_p2: Tuple[float, float],
    max_range: float
) -> Optional[float]:
    x1, y1 = ray_origin
    x2, y2 = x1 + math.cos(ray_angle), y1 + math.sin(ray_angle)
    x3, y3 = segment_p1
    x4, y4 = segment_p2

    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if den == 0:
        return None

    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
    u = ((x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2)) / den

    # u should be between 0 and 1, t > 0
    if t > 0 and 0 <= u <= 1:
        if t <= max_range:
            return t
    return None

def cast_ray(
    ray_origin: Tuple[float, float],
    ray_angle: float,
    obstacles: List[Tuple[Tuple[float, float], Tuple[float, float]]],
    max_range: float
) -> float:
    min_dist = max_range
    for p1, p2 in obstacles:
        dist = ray_segment_intersection(ray_origin, ray_angle, p1, p2, max_range)
        if dist is not None and dist < min_dist:
            min_dist = dist
    return min_dist
