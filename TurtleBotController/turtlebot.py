import os
import json
import math
import time
import threading
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

class _TurtleBotRosNode(Node):
    def __init__(self, config):
        super().__init__('turtlebot_controller_node')
        self.config = config
        
        self.cmd_topic = config['ros'].get('cmd_vel_topic', '/cmd_vel')
        self.scan_topic = config['ros'].get('scan_topic', '/scan')
        self.image_topic = config['ros'].get('image_topic', '/oakd/rgb/preview/image_raw')
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
        self.bridge = CvBridge()
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self._scan_callback, 10)
        self.image_sub = self.create_subscription(Image, self.image_topic, self._image_callback, 10)
        
        self.latest_scan = None
        self.latest_image = None
        
    def _scan_callback(self, msg):
        self.latest_scan = msg
        
    def _image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")


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
        
        # Configurar explícitamente el ROS_DOMAIN_ID en las variables de entorno 
        # ANTES de inicializar ROS 2.
        domain_id = str(self.config['ros'].get('domain_id', 77))
        os.environ["ROS_DOMAIN_ID"] = domain_id
        
        # Iniciar YOLO
        model_path = os.path.join(base_dir, self.config['vision'].get('yolo_model_path', '../yolonano/best.pt'))
        self.conf_threshold = self.config['vision'].get('confidence_threshold', 0.85)
        self.yolo_model = None
        if YOLO is not None and os.path.exists(model_path):
            try:
                self.yolo_model = YOLO(model_path)
                print(f"[VISION] YOLO cargado exitosamente: {model_path}")
            except Exception as e:
                print(f"[VISION] Error al cargar YOLO: {e}")
        else:
            print("[VISION] Advertencia: YOLO no disponible o modelo no encontrado.")
            
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
        Retorna una lista/array de distancias, homogeneizado a `lidar_resolution`.
        """
        scan = np.zeros(self.lidar_resolution)
        scan.fill(self.lidar_max_range)
        
        msg = self.node.latest_scan
        if msg is not None:
            ranges = np.array(msg.ranges)
            # Limpiar Infs o NaNs generados por el sensor real
            ranges = np.nan_to_num(ranges, posinf=self.lidar_max_range, neginf=0.0)
            
            n_ranges = len(ranges)
            if n_ranges > 0:
                # Interpolar al número de rayos del simulador (ej. 360)
                indices = np.linspace(0, n_ranges - 1, self.lidar_resolution).astype(int)
                scan = ranges[indices]
                # Asegurar que nada pase el rango máximo
                scan = np.clip(scan, 0.0, self.lidar_max_range).tolist()
                
                # CORRECCIÓN DE CALIBRACIÓN DE HARDWARE:
                # El sensor físico del TurtleBot tiene su 0 apuntando hacia la DERECHA del robot.
                # Como el escáner gira antihorario, el verdadero FRENTE está en el índice 90.
                # Rotamos el arreglo para que el índice 0 sea siempre el frente.
                scan = scan[90:] + scan[:90]
                
        return scan

    def get_vision_detections(self):
        """
        Pasa la última imagen por el modelo YOLO y retorna una lista en formato:
        [{'class': 'left', 'distance': 1.5, 'relative_angle': 0.1}, ...]
        """
        detections = []
        frame = self.node.latest_image
        
        if frame is not None and self.yolo_model is not None:
            h, w = frame.shape[:2]
            results = self.yolo_model(frame, verbose=False, conf=self.conf_threshold)
            
            if len(results) > 0:
                boxes = results[0].boxes
                for box in boxes:
                    cls_id = int(box.cls[0])
                    class_name = self.yolo_model.names[cls_id]
                    
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cx = (x1 + x2) / 2.0
                    
                    # Calcular ángulo relativo
                    # Pixeles mapeados desde -1 (izq) a 1 (der)
                    normalized_x = (cx - (w / 2.0)) / (w / 2.0)
                    relative_angle = -normalized_x * (self.camera_fov / 2.0)
                    
                    # Estimar distancia asumiendo un tamaño de señal estándar (aprox 20 cm)
                    # Formula empírica de Pinhole Camera:
                    bbox_width = max(x2 - x1, 1.0)
                    focal_length = w  # Asumimos que el FOV es aproximadamente de 1 radian para este cálculo simple
                    real_width = 0.20 # 20cm
                    distance = (real_width * focal_length) / bbox_width
                    
                    detections.append({
                        'class': class_name,
                        'distance': distance,
                        'relative_angle': relative_angle
                    })
                    
        return detections

    def stop(self):
        """Detiene el robot completamente."""
        self.move(0.0, 0.0, 0.1)
