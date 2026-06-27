import math

def distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(x2 - x1, y2 - y1)

def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))
