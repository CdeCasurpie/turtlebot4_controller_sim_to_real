import math
import numpy as np

from Simulator.core.geometry import normalize_angle
from Simulator.core.raycasting import cast_ray
from Simulator.WorldSim.world import World

class TurtleBotMock:
    def __init__(self, world: World, initial_x: float = 0.0, initial_y: float = 0.0, initial_theta: float = 0.0):
        self._world = world
        self.__x = initial_x
        self.__y = initial_y
        self.__theta = initial_theta
        
        self.lidar_resolution = 360
        self.lidar_max_range = 12.0
        self.camera_fov = math.radians(60)
        self.camera_min_range = 0.1
        self.camera_max_range = 5.0
        self.radius = 0.17
        
        self.__command_buffer = [(0.0, 0.0)] * 3
        self.v_actual = 0.0
        self.omega_actual = 0.0
        
        self.sim_config = {
            "vision_reliable_dist": 0.45,
            "yolo_error_max_prob": 0.95
        }

    def update_config(self, config_dict):
        self.sim_config.update(config_dict)

    def move(self, v: float, omega: float, dt: float) -> bool:
        import random
        
        self.__command_buffer.append((v, omega))
        v_delayed, omega_delayed = self.__command_buffer.pop(0)
        
        self.v_actual = v_delayed + random.gauss(0, 0.02) if v_delayed != 0 else 0.0
        self.omega_actual = omega_delayed + random.gauss(0, 0.05) if omega_delayed != 0 else 0.0
        
        self.__x += self.v_actual * math.cos(self.__theta) * dt
        self.__y += self.v_actual * math.sin(self.__theta) * dt
        self.__theta += self.omega_actual * dt
        self.__theta = normalize_angle(self.__theta)
        
        chocado = False
        for p1, p2 in self._world.obstacles:
            x1, y1 = p1
            x2, y2 = p2
            
            px, py = x2 - x1, y2 - y1
            norm = px * px + py * py
            if norm == 0: continue
            
            u = ((self.__x - x1) * px + (self.__y - y1) * py) / float(norm)
            u = max(0.0, min(1.0, u))
            cx_wall, cy_wall = x1 + u * px, y1 + u * py
            dist = math.hypot(self.__x - cx_wall, self.__y - cy_wall)

            if dist < self.radius:
                chocado = True
                overlap = self.radius - dist
                if dist > 0:
                    self.__x += ((self.__x - cx_wall) / dist) * overlap
                    self.__y += ((self.__y - cy_wall) / dist) * overlap
                    
        return chocado

    def get_lidar_scan(self) -> np.ndarray:
        raw_scan = np.zeros(self.lidar_resolution)
        angle_increment = (2 * math.pi) / self.lidar_resolution
        import random
        
        offset_hardware = -math.pi / 2
        
        for i in range(self.lidar_resolution):
            ray_angle = self.__theta + offset_hardware + i * angle_increment
            dist = cast_ray((self.__x, self.__y), ray_angle, self._world.obstacles, self.lidar_max_range)
            if dist < self.lidar_max_range:
                dist += random.gauss(0, 0.015)
                
            if random.random() < 0.02:
                dist = random.uniform(0.05, 0.17)
                
            raw_scan[i] = max(0.0, dist)
            
        raw_scan_list = raw_scan.tolist()
        corrected_scan = raw_scan_list[90:] + raw_scan_list[:90]
            
        return np.array(corrected_scan)

    def get_vision_detections(self):
        import random
        detections = []
        for signal in self._world.signals:
            if random.random() > 0.95:
                continue
                
            dx = signal['x'] - self.__x
            dy = signal['y'] - self.__y
            angle_to_signal = math.atan2(dy, dx)
            relative_angle = normalize_angle(angle_to_signal - self.__theta)
            
            if abs(relative_angle) <= (self.camera_fov / 2):
                true_dist = math.hypot(dx, dy)
                if not (self.camera_min_range <= true_dist <= self.camera_max_range):
                    continue
                
                obstacle_dist = cast_ray((self.__x, self.__y), angle_to_signal, self._world.obstacles, self.lidar_max_range)
                if obstacle_dist >= true_dist - 0.1:
                    noisy_angle = relative_angle + random.gauss(0, math.radians(2.0))
                    noisy_distance = true_dist + random.gauss(0, 0.1 * true_dist)
                    detected_class = signal['type']
                    
                    reliable_dist = self.sim_config.get("vision_reliable_dist", 0.45)
                    max_prob = self.sim_config.get("yolo_error_max_prob", 0.95)
                    if true_dist > reliable_dist:
                        prob_error = min(max_prob, (true_dist - reliable_dist) * 0.4)
                        
                        if random.random() < prob_error:
                            continue
                            
                        if detected_class in ['left', 'right'] and random.random() < prob_error:
                            detected_class = 'right' if detected_class == 'left' else 'left'

                    detections.append({
                        'class': detected_class,
                        'distance': noisy_distance,
                        'relative_angle': noisy_angle
                    })
                
        return detections

    def get_qr_detections(self):
        """
        Simula la lectura de un código QR. 
        Implementa restricciones realistas de rango, FOV y blur de movimiento.
        """
        import random
        detections = []
        
        # Simulación de Motion Blur: Si el robot gira a más de 0.8 rad/s o avanza muy rápido,
        # la cámara captura una imagen borrosa y el decodificador falla.
        if abs(self.omega_actual) > 0.8 or abs(self.v_actual) > 0.4:
            return detections
            
        for qr in getattr(self._world, 'qrs', []):
            dx = qr['x'] - self.__x
            dy = qr['y'] - self.__y
            angle_to_qr = math.atan2(dy, dx)
            relative_angle = normalize_angle(angle_to_qr - self.__theta)
            
            # El QR debe estar en el campo de visión de la cámara
            if abs(relative_angle) <= (self.camera_fov / 2):
                true_dist = math.hypot(dx, dy)
                
                # Rango de enfoque (0.2m a 1.2m máximo para resolución típica)
                if 0.2 <= true_dist <= 1.2:
                    # Validar que ninguna pared tape el QR
                    obstacle_dist = cast_ray((self.__x, self.__y), angle_to_qr, self._world.obstacles, self.lidar_max_range)
                    if obstacle_dist >= true_dist - 0.1:
                        # 15% de probabilidad de fallo por iluminación/reflejos
                        if random.random() > 0.15:
                            detections.append({
                                'content': qr['content'],
                                'distance': true_dist,
                                'relative_angle': relative_angle
                            })
        return detections

    def get_camera_image_base64(self) -> str:
        import cv2
        import base64
        
        frame = np.zeros((120, 160, 3), dtype=np.uint8)
        frame[:] = (50, 50, 50) 
        
        detections = self.get_vision_detections()
        for det in detections:
            d_theta = det['relative_angle']
            img_insert = np.zeros((60, 60, 3), dtype=np.uint8)
            img_insert[:] = (255, 255, 255)
            
            texto = det['class'][:3].upper()
            cv2.putText(img_insert, texto, (5, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            
            h_i, w_i = 60, 60
            fov_half = self.camera_fov / 2
            cx_screen = int(80 + (d_theta / fov_half) * 80)
            
            x_offset = cx_screen - w_i // 2
            y_offset = 30
            
            if x_offset >= 0 and x_offset + w_i <= 160:
                frame[y_offset : y_offset + h_i, x_offset : x_offset + w_i] = img_insert
                
        _, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        return base64.b64encode(buffer).decode("utf-8")

    def _get_true_pose(self):
        return self.__x, self.__y, self.__theta