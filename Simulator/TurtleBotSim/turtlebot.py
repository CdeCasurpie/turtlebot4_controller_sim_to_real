import math
import numpy as np

from Simulator.core.geometry import normalize_angle
from Simulator.core.raycasting import cast_ray
from Simulator.WorldSim.world import World

class TurtleBotMock:
    def __init__(self, world: World, initial_x: float = 0.0, initial_y: float = 0.0, initial_theta: float = 0.0):
        self._world = world
        # Pose oculta (Ground Truth)
        self.__x = initial_x
        self.__y = initial_y
        self.__theta = initial_theta
        
        # Especificaciones del sensor
        self.lidar_resolution = 360 # Rayos
        self.lidar_max_range = 12.0 # Metros
        self.camera_fov = math.radians(60) # Campo de visión del YOLO
        self.camera_min_range = 0.1 # Rango mínimo ciego
        self.camera_max_range = 3.0 # Límite físico de detección de YOLO (aumentado)
        self.radius = 0.17 # Radio aproximado del robot (Create 3)
        
        # Simulación de imperfecciones de hardware real
        self.__command_buffer = [(0.0, 0.0)] * 3 # Delay sutil de 3 frames (~50ms)
        self.v_actual = 0.0
        self.omega_actual = 0.0

    # ==========================================
    # INTERFAZ DE ACTUADORES (Planta Cinemática)
    # ==========================================
    def move(self, v: float, omega: float, dt: float) -> bool:
        """
        Integra el modelo diferencial estricto de forma numérica.
        Resuelve colisiones matemáticamente (repulsión) y retorna True si hubo choque.
        """
        import random
        
        # Añadir al buffer y extraer el comando retrasado
        self.__command_buffer.append((v, omega))
        v_delayed, omega_delayed = self.__command_buffer.pop(0)
        
        # Ruido de actuadores (Gaussiana suave)
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

    # ==========================================
    # INTERFAZ DE SENSORES (Observaciones)
    # ==========================================
    def get_lidar_scan(self) -> np.ndarray:
        """
        Retorna un array de 360 distancias usando el motor de raycasting.
        """
        scan = np.zeros(self.lidar_resolution)
        angle_increment = (2 * math.pi) / self.lidar_resolution
        import random
        
        for i in range(self.lidar_resolution):
            ray_angle = self.__theta + i * angle_increment
            dist = cast_ray((self.__x, self.__y), ray_angle, self._world.obstacles, self.lidar_max_range)
            # Añadir ruido Gaussiano al LiDAR (~1.5cm de desviación típica)
            if dist < self.lidar_max_range:
                dist += random.gauss(0, 0.015)
            scan[i] = max(0.0, dist)
            
        return scan

    def get_vision_detections(self):
        """
        Mock de YOLO + Sensor Fusion con LiDAR.
        Retorna una lista de detecciones simulando lo que YOLO Nano vería y 
        fusionando la distancia del LiDAR.
        """
        detections = []
        for signal in self._world.signals:
            # 1. Transformación al marco de referencia del robot
            dx = signal['x'] - self.__x
            dy = signal['y'] - self.__y
            
            # Ángulo absoluto del objeto
            angle_to_signal = math.atan2(dy, dx)
            
            # Ángulo relativo (bearing) respecto a la orientación del robot
            relative_angle = normalize_angle(angle_to_signal - self.__theta)
            
            # 2. Condición de Campo de Visión (Culling frustum)
            if abs(relative_angle) <= (self.camera_fov / 2):
                true_dist = math.hypot(dx, dy)
                
                # Deficiencia del sensor YOLO: Sólo detecta entre min y max metros
                if not (self.camera_min_range <= true_dist <= self.camera_max_range):
                    continue
                
                # 3. Validar Oclusión y Fusión con LiDAR mock
                # Simulamos enviar un rayo directamente a la señal
                obstacle_dist = cast_ray((self.__x, self.__y), angle_to_signal, self._world.obstacles, self.lidar_max_range)
                
                # Si no hay obstáculo entre el robot y la señal (o el obstáculo está detrás de la señal)
                if obstacle_dist >= true_dist - 0.1: # Tolerancia pequeña
                    detections.append({
                        'class': signal['type'],
                        'distance': true_dist,
                        'relative_angle': relative_angle
                    })
                
        return detections

    def get_camera_image_base64(self) -> str:
        """
        Genera una imagen JPEG codificada en Base64 con las señales vistas.
        Compatible con el protocolo de telemetría de red.
        """
        import cv2
        import base64
        
        frame = np.zeros((120, 160, 3), dtype=np.uint8)
        frame[:] = (50, 50, 50) # Fondo oscuro
        
        detections = self.get_vision_detections()
        for det in detections:
            d_theta = det['relative_angle']
            
            # Crear una pequeña imagen de la señal
            img_insert = np.zeros((60, 60, 3), dtype=np.uint8)
            img_insert[:] = (255, 255, 255)
            
            texto = det['class'][:3].upper()
            cv2.putText(
                img_insert,
                texto,
                (5, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2,
            )
            
            h_i, w_i = 60, 60
            fov_half = self.camera_fov / 2
            cx_screen = int(80 + (d_theta / fov_half) * 80)
            
            x_offset = cx_screen - w_i // 2
            y_offset = 30
            
            if x_offset >= 0 and x_offset + w_i <= 160:
                frame[y_offset : y_offset + h_i, x_offset : x_offset + w_i] = img_insert
                
        _, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        return base64.b64encode(buffer).decode("utf-8")

    # Funcionalidad interna EXCLUSIVA para el renderizado (no para control)
    def _get_true_pose(self):
        return self.__x, self.__y, self.__theta
