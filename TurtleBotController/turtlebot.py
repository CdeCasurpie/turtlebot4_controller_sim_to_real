import os

# Límite de hilos de inferencia ANTES de importar numpy/cv2/ultralytics:
# la Pi 4 tiene 4 núcleos y también corre los drivers del lidar/cámara y el
# bucle de control. Si OpenMP (NCNN/torch) toma los 4, el sistema entero se
# estrangula (load >10, throttling térmico, el driver del lidar pierde el
# serial y detiene el motor).
os.environ.setdefault("OMP_NUM_THREADS", "2")

import json
import math
import time
import threading
import numpy as np
import cv2

cv2.setNumThreads(2)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
    import torch
    torch.set_num_threads(2)
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
        # /odom del Create 3: BEST_EFFORT matchea tanto publishers RELIABLE
        # como BEST_EFFORT (la compatibilidad QoS exige pub >= sub).
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_callback, qos)

        self.latest_scan = None
        self.latest_scan_time = 0.0
        self.latest_image = None
        self.latest_image_time = 0.0
        self.latest_odom = None      # (x, y, yaw)
        self.latest_odom_time = 0.0

    def _scan_callback(self, msg):
        self.latest_scan = msg
        self.latest_scan_time = time.time()

    def _odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # yaw desde el cuaternión (el Create 3 se mueve en el plano)
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.latest_odom = (p.x, p.y, yaw)
        self.latest_odom_time = time.time()

    def _image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_image_time = time.time()
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
        
        # ROS_DOMAIN_ID: si ya está definido en el entorno (en el robot lo define
        # /etc/turtlebot4/setup.bash y debe coincidir con la base Create 3), se respeta.
        # El valor del config es el fallback para PCs donde la variable no existe.
        domain_id = os.environ.get("ROS_DOMAIN_ID")
        if domain_id:
            print(f"[TurtleBotController] ROS_DOMAIN_ID={domain_id} (tomado del entorno)")
        else:
            domain_id = str(self.config['ros'].get('domain_id', 77))
            os.environ["ROS_DOMAIN_ID"] = domain_id
            print(f"[TurtleBotController] ROS_DOMAIN_ID={domain_id} (tomado de config.json)")

        # Backend de visión:
        #   "vpu"      -> YOLO corre dentro de la cámara OAK-D (Myriad X). Requiere el
        #                 blob convertido y el nodo oakd de ROS detenido (ver DEPLOY.md).
        #   "ros_yolo" -> imágenes por ROS + YOLO (torch/NCNN) en el CPU del Pi.
        #   "auto"     -> intenta VPU si el blob existe; si falla, cae a ros_yolo.
        self.conf_threshold = self.config['vision'].get('confidence_threshold', 0.85)
        backend = self.config['vision'].get('backend', 'auto')
        self.vpu = None
        self.yolo_model = None

        if backend in ('auto', 'vpu'):
            self.vpu = self._try_init_vpu(base_dir, required=(backend == 'vpu'))

        if self.vpu is None:
            # Backend ROS + YOLO en CPU
            model_path = os.path.join(base_dir, self.config['vision'].get('yolo_model_path', '../yolonano/best.pt'))
            # En ARM (Raspberry Pi) un export NCNN es varias veces más rápido que torch.
            # Si existe el directorio exportado junto al .pt, se prefiere automáticamente.
            ncnn_dir = os.path.splitext(model_path)[0] + "_ncnn_model"
            if os.path.isdir(ncnn_dir):
                print(f"[VISION] Usando modelo NCNN optimizado: {ncnn_dir}")
                model_path = ncnn_dir
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

        # Visión asíncrona: en la Raspberry Pi la inferencia YOLO tarda cientos de ms;
        # si corre dentro del bucle de control de 20 Hz, la evasión de obstáculos se
        # degrada. Un hilo aparte infiere continuamente y cachea las detecciones.
        self.async_vision = self.config['vision'].get('async_vision', True)
        self.detection_max_age = self.config['vision'].get('detection_max_age', 1.0)
        # Ritmo del hilo de visión: para señales de tránsito bastan ~5 Hz.
        # Sin pausa entre inferencias el hilo satura la CPU de la Pi.
        self.vision_period = self.config['vision'].get('vision_period', 0.2)
        # Confirmación temporal: exigir la misma clase en N inferencias seguidas
        # protege contra falsos positivos al bajar confidence_threshold.
        self.min_consecutive = self.config['vision'].get('min_consecutive_detections', 2)
        self.yolo_imgsz = self.config['vision'].get('imgsz', 320)
        # Frescura de sensores: datos más viejos que esto se consideran caídos.
        self.scan_max_age = self.config['vision'].get('scan_max_age', 0.5)
        self._vision_lock = threading.Lock()
        self._cached_detections = []
        self._cached_detections_time = 0.0
        self._consec_counts = {}
        self._vision_frames = 0
        if self.yolo_model is not None and self.async_vision:
            self._vision_thread = threading.Thread(target=self._vision_loop, daemon=True)
            self._vision_thread.start()

        # Dar un pequeño tiempo de gracia para que lleguen los primeros mensajes
        print("[TurtleBotController] Esperando sensores (1 segundo)...")
        time.sleep(1.0)
        print("[TurtleBotController] ¡Robot Real Listo para actuar!")

    def _try_init_vpu(self, base_dir, required=False):
        """Intenta crear el backend VPU. Retorna la instancia o None (con fallback)."""
        vpu_cfg = self.config['vision'].get('vpu', {})
        blob_path = os.path.join(base_dir, vpu_cfg.get('blob_path', '../yolonano/vpu/best.blob'))
        config_path = os.path.join(base_dir, vpu_cfg.get('config_path', '../yolonano/vpu/best.json'))

        if not required and not (os.path.exists(blob_path) and os.path.exists(config_path)):
            # Modo auto sin blob convertido: silenciosamente usamos ros_yolo.
            return None

        try:
            from .vpu_vision import VpuVision
            vpu = VpuVision(
                blob_path=blob_path,
                config_path=config_path,
                camera_fov=math.radians(vpu_cfg.get('camera_fov_deg', self.config['vision'].get('camera_fov_deg', 60.0))),
                conf_threshold=self.conf_threshold,
                fps=vpu_cfg.get('fps', 15.0),
                distance_scale=vpu_cfg.get('distance_scale', 1.0),
                detection_max_age=self.config['vision'].get('detection_max_age', 1.0),
            )
            print("[VISION] Backend activo: VPU (inferencia dentro de la cámara)")
            return vpu
        except Exception as e:
            print(f"[VISION] No se pudo iniciar la VPU: {e}")
            print("[VISION] ¿El nodo oakd sigue corriendo? (sudo systemctl stop oakd). "
                  "Cayendo al backend ros_yolo...")
            return None

    def _spin_ros(self):
        rclpy.spin(self.node)

    def _vision_loop(self):
        while True:
            frame = self.node.latest_image
            # Imagen ausente o congelada (cámara caída): no inferir ni confirmar.
            if frame is None or (time.time() - self.node.latest_image_time) > 1.0:
                self._consec_counts = {}
                with self._vision_lock:
                    self._cached_detections = []
                time.sleep(0.05)
                continue
            t0 = time.time()
            detections = self._run_yolo(frame)
            elapsed = time.time() - t0
            self._vision_frames += 1
            # Se loguea el 2do frame: el 1ro incluye el warmup del modelo y
            # reporta un tiempo ~5-10x mayor que el real.
            if self._vision_frames == 2:
                print(f"\n[VISION] Inferencia YOLO: {elapsed*1000:.0f} ms/frame (~{1.0/max(elapsed, 1e-6):.1f} Hz)")

            # Confirmación temporal por clase
            presentes = {d['class'] for d in detections}
            for clase in list(self._consec_counts):
                if clase not in presentes:
                    self._consec_counts[clase] = 0
            for clase in presentes:
                self._consec_counts[clase] = self._consec_counts.get(clase, 0) + 1
            confirmadas = [d for d in detections
                           if self._consec_counts[d['class']] >= self.min_consecutive]

            with self._vision_lock:
                self._cached_detections = confirmadas
                self._cached_detections_time = time.time()

            # Ceder CPU: el control y los drivers necesitan núcleos libres.
            time.sleep(max(0.0, self.vision_period - elapsed))
        
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
        Retorna una LISTA de distancias homogeneizada a `lidar_resolution`.

        Si el lidar no ha publicado nunca, o su último mensaje es más viejo que
        `scan_max_age` (sensor caído), retorna [] — el llamador debe FRENAR.
        Nunca se inventan lecturas "libres": un array relleno de max_range haría
        que el robot avance ciego a velocidad máxima.
        """
        scan = []

        msg = self.node.latest_scan
        if msg is not None and (time.time() - self.node.latest_scan_time) <= self.scan_max_age:
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

    def get_odometry(self):
        """
        Retorna (x, y, yaw) de /odom, o None si aún no llegó ningún mensaje
        (o el último tiene más de 1 s — odometría caída).

        El controlador solo usa DIFERENCIAS de pose/yaw (giros cerrados y
        distancia recorrida), así que el drift lento del odom no le afecta.
        """
        if self.node.latest_odom is None:
            return None
        if (time.time() - self.node.latest_odom_time) > 1.0:
            return None
        return self.node.latest_odom

    def get_vision_detections(self):
        """
        Retorna las detecciones YOLO en formato:
        [{'class': 'left', 'distance': 1.5, 'relative_angle': 0.1}, ...]

        Con async_vision (default) retorna la caché del hilo de visión; si las
        detecciones son más viejas que detection_max_age se consideran caducas.
        Con backend VPU, las detecciones vienen directo de la cámara.
        """
        if self.vpu is not None:
            return self.vpu.get_detections()

        if self.yolo_model is None:
            return []

        if self.async_vision:
            with self._vision_lock:
                if time.time() - self._cached_detections_time <= self.detection_max_age:
                    return list(self._cached_detections)
                return []

        frame = self.node.latest_image
        if frame is None:
            return []
        return self._run_yolo(frame)

    def _run_yolo(self, frame):
        detections = []
        h, w = frame.shape[:2]
        # imgsz debe coincidir con el del entrenamiento (320). Sin él, ultralytics
        # infiere a 640 por defecto: 4x el cómputo para el mismo modelo.
        results = self.yolo_model(frame, verbose=False, conf=self.conf_threshold, imgsz=self.yolo_imgsz)

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
