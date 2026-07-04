import os
import json
import math
import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan

from .vpu_vision import VpuYoloDetector
from .lidar_processing import process_scan

class _TurtleBotRosNode(Node):
    def __init__(self, config):
        super().__init__('turtlebot_controller_node')
        self.config = config

        self.cmd_topic = config['ros'].get('cmd_vel_topic', '/cmd_vel')
        self.scan_topic = config['ros'].get('scan_topic', '/scan')
        self.use_twist_stamped = config['ros'].get('use_twist_stamped', True)

        # QoS Profile BEST_EFFORT para coincidir con la base del Create 3
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Publishers
        if self.use_twist_stamped:
            self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, qos)
        else:
            self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, qos)

        # Subscribers (Default QoS, igual que tu enviador.py)
        # Nota: ya no hay suscripción de imagen por ROS. La visión corre directo
        # sobre la VPU de la OAK-D vía DepthAI (ver VpuYoloDetector), sin pasar
        # por un tópico de imagen ni por la CPU de la Raspberry Pi.
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self._scan_callback, 10)

        self.latest_scan = None
        self.latest_scan_time = None  # time.monotonic() de recepción (watchdog)

    def _scan_callback(self, msg):
        self.latest_scan = msg
        self.latest_scan_time = time.monotonic()


class TurtleBotReal:
    def __init__(self, config_path="config.json"):
        """
        Inicializa el robot real conectándose a los tópicos de ROS 2 en background.
        """
        # Cargar configuración
        base_dir = os.path.dirname(os.path.abspath(__file__))
        config_file = os.path.join(base_dir, config_path)
        with open(config_file, 'r') as f:
            self.config = json.load(f)
            
        # Atributos básicos (idénticos a la simulación para compatibilidad)
        self.radius = 0.17
        self.lidar_resolution = self.config['robot'].get('lidar_resolution', 360)
        self.lidar_max_range = self.config['robot'].get('lidar_max_range', 12.0)
        self.camera_fov = math.radians(self.config['vision'].get('camera_fov_deg', 60.0))
        
        self.max_linear = self.config['robot'].get('max_linear', 1.0)
        self.max_angular = self.config['robot'].get('max_angular', 3.0)

        # Higiene del LiDAR (T2): validez, montaje y watchdog
        self.lidar_min_valid = self.config['robot'].get('lidar_min_valid', 0.18)
        self.lidar_front_angle = math.radians(
            self.config['robot'].get('lidar_front_angle_deg', 90.0))
        self.scan_stale_after = self.config['robot'].get('scan_stale_after', 0.3)
        
        # Configurar explícitamente el ROS_DOMAIN_ID en las variables de entorno 
        # ANTES de inicializar ROS 2.
        domain_id = str(self.config['ros'].get('domain_id', 77))
        os.environ["ROS_DOMAIN_ID"] = domain_id
        
        # Iniciar YOLO en la VPU (Myriad X) de la OAK-D vía DepthAI
        vision_cfg = self.config['vision']
        self.conf_threshold = vision_cfg.get('confidence_threshold', 0.85)
        blob_path = os.path.join(base_dir, vision_cfg.get('vpu_blob_path', '../vpu_deployment/models/turtlebot_signals_v2.blob'))
        classes_path = os.path.join(base_dir, vision_cfg.get('classes_path', '../yolonanov2/classes.txt'))
        self.vpu_detector = None
        try:
            self.vpu_detector = VpuYoloDetector(
                blob_path=blob_path,
                classes_path=classes_path,
                num_classes=vision_cfg.get('num_classes', 4),
                confidence_threshold=self.conf_threshold,
                iou_threshold=vision_cfg.get('iou_threshold', 0.5),
                fps=vision_cfg.get('fps', 15),
                camera_fov_rad=self.camera_fov,
            )
            print(f"[VISION-VPU] Pipeline DepthAI iniciado: {blob_path}")
        except Exception as e:
            print(f"[VISION-VPU] Error al iniciar la VPU: {e}")

        # Iniciar ROS 2 en un hilo separado
        print(f"[TurtleBotController] Iniciando ROS 2 en el DOMAIN_ID: {domain_id}")
        if not rclpy.ok():
            rclpy.init(args=None)
            
        self.node = _TurtleBotRosNode(self.config)
        self.ros_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self.ros_thread.start()
        
        # Dar un pequeño tiempo de gracia para que lleguen los primeros mensajes
        print("[TurtleBotController] Esperando sensores (1 segundo)...")
        time.sleep(1.0)
        print("[TurtleBotController] ¡Robot Real Listo para actuar!")
        
    def _spin_ros(self):
        rclpy.spin(self.node)
        
    def move(self, v: float, omega: float, dt: float) -> bool:
        """
        Publica las velocidades deseadas en ROS y duerme por 'dt' segundos.
        A diferencia del simulador, siempre retorna False (porque las físicas reales
        dependerían de leer un bumper o el LIDAR).
        """
        # Limitar la velocidad por seguridad
        v = max(-self.max_linear, min(self.max_linear, v))
        omega = max(-self.max_angular, min(self.max_angular, omega))
        
        if self.node.use_twist_stamped:
            msg = TwistStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = float(v)
            msg.twist.angular.z = float(omega)
        else:
            msg = Twist()
            msg.linear.x = float(v)
            msg.angular.z = float(omega)
            
        self.node.cmd_pub.publish(msg)
        time.sleep(dt)
        
        return False
        
    def get_lidar_scan(self) -> list:
        """
        Retorna una lista de `lidar_resolution` distancias, índice 0 = frente,
        antihorario. Rayos inválidos (NaN/inf/0.0/reflexiones del chasis) ya
        vienen saneados a `lidar_max_range`, y la rotación del montaje se
        deriva de angle_min/angle_increment del mensaje (ver lidar_processing).
        """
        msg = self.node.latest_scan
        if msg is None:
            return [self.lidar_max_range] * self.lidar_resolution

        return process_scan(
            msg.ranges, msg.angle_min, msg.angle_increment,
            resolution=self.lidar_resolution,
            max_range=self.lidar_max_range,
            min_valid=self.lidar_min_valid,
            front_offset_rad=self.lidar_front_angle,
        )

    def scan_age(self) -> float:
        """
        Segundos desde el último scan recibido (inf si nunca llegó ninguno).
        El loop de control debe detener el robot si esto supera
        `scan_stale_after` — manejar sobre un scan congelado es manejar a ciegas.
        """
        if self.node.latest_scan_time is None:
            return float('inf')
        return time.monotonic() - self.node.latest_scan_time

    def get_vision_detections(self):
        """
        Retorna las últimas detecciones resueltas por la VPU de la OAK-D, en formato:
        [{'class': 'left', 'distance': 1.5, 'relative_angle': 0.1}, ...]
        """
        if self.vpu_detector is None:
            return []
        return self.vpu_detector.get_detections()

    def stop(self):
        """Detiene el robot completamente."""
        self.move(0.0, 0.0, 0.1)
