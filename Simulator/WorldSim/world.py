import json
from typing import List, Tuple, Dict, Any

class World:
    def __init__(self):
        # Geometría dura (muros, cajas) representada como segmentos de línea
        # list of line segments: ((x1, y1), (x2, y2))
        self.obstacles: List[Tuple[Tuple[float, float], Tuple[float, float]]] = [] 
        
        # Objetos semánticos: {'type': 'right', 'x': 2.0, 'y': 3.0}
        self.signals: List[Dict[str, Any]] = []   
        
        # Posición inicial del robot
        self.robot_start: Dict[str, float] = {'x': 0.0, 'y': 0.0, 'theta': 0.0}

    def add_obstacle_segment(self, x1: float, y1: float, x2: float, y2: float):
        self.obstacles.append(((x1, y1), (x2, y2)))

    def add_obstacle_polygon(self, points: List[Tuple[float, float]]):
        if len(points) < 2: return
        for i in range(len(points)):
            p1 = points[i]
            p2 = points[(i + 1) % len(points)]
            self.obstacles.append((p1, p2))

    def add_signal(self, signal_type: str, x: float, y: float):
        self.signals.append({'type': signal_type, 'x': x, 'y': y})

    def save_to_file(self, filename: str):
        data = {
            'obstacles': self.obstacles,
            'signals': self.signals,
            'robot_start': self.robot_start
        }
        with open(filename, 'w') as f:
            json.dump(data, f, indent=4)

    def load_from_file(self, filename: str):
        with open(filename, 'r') as f:
            data = json.load(f)
            # Reconstruir las tuplas
            self.obstacles = [((p1[0], p1[1]), (p2[0], p2[1])) for p1, p2 in data.get('obstacles', [])]
            self.signals = data.get('signals', [])
            self.robot_start = data.get('robot_start', {'x': 0.0, 'y': 0.0, 'theta': 0.0})
