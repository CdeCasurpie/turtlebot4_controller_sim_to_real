import sys
use_ui = "--no-ui" not in sys.argv
if use_ui:
    import pygame
import json
import math
import os
import numpy as np
import time
import threading
import cv2

try:
    import depthai as dai
except ImportError:
    dai = None

from Simulator.WorldSim.world import World
from Simulator.TurtleBotSim.turtlebot import TurtleBotMock

use_simulator = "--simulator" in sys.argv
use_yolo = "--no-yolo" not in sys.argv

if not use_simulator:
    from TurtleBotController.turtlebot import TurtleBotReal

SCALE = 50.0
WIDTH, HEIGHT = 800, 600
OFFSET_X, OFFSET_Y = WIDTH // 2, HEIGHT // 2

# ========================================================
# HILO EN SEGUNDO PLANO PARA EL ESCÁNER QR
# ========================================================
class QRScannerThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.daemon = True 
        self.running = True
        
        if dai is None:
            print("[QR Thread] Error: depthai no está instalado.")
            self.detector = None
            return

        try:
            self.detector = cv2.QRCodeDetector()
            print("[QR Thread] Motor QRCodeDetector clásico inicializado (Versión Liviana).")
        except Exception as e:
            print(f"[QR Thread] Error al inicializar QRCodeDetector: {e}")
            self.detector = None

    def run(self):
        if dai is None or self.detector is None:
            return

        pipeline = dai.Pipeline()
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(640, 480)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(15)

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        contador_qr = 0
        ultimo_tiempo = 0
        qr_en_pantalla = False

        try:
            with dai.Device(pipeline) as device:
                q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
                
                while self.running:
                    in_rgb = q_rgb.get()
                    if in_rgb is None:
                        continue
                        
                    frame = in_rgb.getCvFrame()
                    datos, puntos, _ = self.detector.detectAndDecode(frame)
                    tiempo_actual = time.time()

                    if puntos is not None and len(puntos) > 0 and datos and datos != "":
                        if not qr_en_pantalla:
                            contador_qr += 1
                            print(f"\n---> [NUEVO QR DETECTADO] #{contador_qr} | Contenido: {datos} <---")
                            qr_en_pantalla = True
                        
                        ultimo_tiempo = tiempo_actual
                    else:
                        qr_en_pantalla = False
                        if contador_qr > 0 and (tiempo_actual - ultimo_tiempo) >= 5.0:
                            contador_qr = 0
                            
        except RuntimeError as e:
            print(f"[QR Thread] Conflicto de cámara (probablemente en uso por YOLO): {e}")
        except Exception as e:
            print(f"[QR Thread] Error en el pipeline del OAK-D: {e}")

    def stop(self):
        self.running = False


# ========================================================
# FUNCIONES AUXILIARES
# ========================================================
def to_screen(x, y):
    return int(OFFSET_X + x * SCALE), int(OFFSET_Y - y * SCALE)

def buscar_camino_libre(lidar_points, radio_robot, direccion='front', margen_extra=0.10):
    if direccion == 'left':
        angulos = [85, 90, 95]
        M = 5
    elif direccion == 'right':
        angulos = [265, 270, 275]
        M = 5
    elif direccion == 'front':
        angulos = [-20, -13, -6, 0, 6, 13, 20]
        M = 5
    else:
        angulos = [30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180] 
        M = 3 
        
    margen = radio_robot + margen_extra
    margen_sq = margen * margen
    paso_inicial = 0.3 
    distancia_paso = (2 * radio_robot) / M
    distancias_prueba = [paso_inicial + i * distancia_paso for i in range(M)]
    
    intentos = []
    mejor_ang = None
    
    for ang_c in angulos:
        ruta_valida = True
        ang_eval = ang_c if ang_c >= 0 else ang_c + 360
        
        for d_c in distancias_prueba:
            cx = d_c * math.cos(math.radians(ang_eval))
            cy = d_c * math.sin(math.radians(ang_eval))
            
            choca = False
            for px, py in lidar_points:
                if (px - cx)*(px - cx) + (py - cy)*(py - cy) < margen_sq:
                    choca = True
                    break
                    
            if choca:
                ruta_valida = False
                break 
                
        intentos.append({'angulo': ang_eval, 'valido': ruta_valida})
        if ruta_valida and mejor_ang is None:
            mejor_ang = ang_eval
            
    return mejor_ang is not None, mejor_ang, intentos, distancias_prueba, margen


def main():
    if use_ui:
        pygame.init()
        screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("TurtleBot 4 - Navegación Definitiva Anti-Choques")
        clock = pygame.time.Clock()
    else:
        screen = None
        clock = None

    qr_thread = None

    if use_simulator:
        world = World()
        map_file = "world_map.json"

        if os.path.exists(map_file):
            world.load_from_file(map_file)
        else:
            print("No se encontró mapa.")
            sys.exit()

        robot = TurtleBotMock(
            world, 
            initial_x=world.robot_start['x'], 
            initial_y=world.robot_start['y'], 
            initial_theta=world.robot_start['theta']
        )
        dt = 1 / 30.0
    else:
        robot = TurtleBotReal("config.json")
        world = None
        dt = 0.05
        
        qr_thread = QRScannerThread()
        qr_thread.start()

    running = True
    paused = False

    estado_actual = "EXPLORANDO"
    ultimo_giro = 'left'
    tiempo_estado = 0.0
    cooldown_senal = 0.0
    clase_ignorada = None  # NUEVO: Para evitar bucles de señales
    choques = 0
    
    last_log_time = 0.0
    tracker = {
        'class': None,
        'relative_angle': 0.0,
        'distance': float('inf'),
        'frames_lost': 999,
        'max_frames': 90,
        'consecutive_frames': 0
    }
    
    sim_contador_qr = 0
    sim_ultimo_tiempo_qr = 0
    sim_qr_en_pantalla = False
    sim_qrs = []
    
    v_target = 0.0
    w_target = 0.0

    history = []
    history_index = -1
    time_since_save = 0.0

    def get_current_state():
        if use_simulator:
            rx, ry, rth = robot._get_true_pose()
        else:
            rx, ry, rth = 0.0, 0.0, 0.0
            
        return {
            'x': rx,
            'y': ry,
            'theta': rth,
            'estado_actual': estado_actual,
            'ultimo_giro': ultimo_giro,
            'tiempo_estado': tiempo_estado,
            'cooldown_senal': cooldown_senal,
            'clase_ignorada': clase_ignorada,
            'choques': choques,
            'tracker': dict(tracker)
        }
        
    def set_state(st):
        nonlocal estado_actual, ultimo_giro, tiempo_estado, cooldown_senal, clase_ignorada, choques, tracker
        if use_simulator:
            robot._TurtleBotMock__x = st['x']
            robot._TurtleBotMock__y = st['y']
            robot._TurtleBotMock__theta = st['theta']
        estado_actual = st['estado_actual']
        ultimo_giro = st.get('ultimo_giro', 'left')
        tiempo_estado = st['tiempo_estado']
        cooldown_senal = st['cooldown_senal']
        clase_ignorada = st.get('clase_ignorada', None)
        choques = st.get('choques', choques)
        tracker = dict(st.get('tracker', tracker))

    history.append(get_current_state())
    history_index = 0
    view_mode = "global" if use_simulator else "robot"
    last_time = time.time()
    
    sim_config_path = "sim_config.json"
    sim_config = {}
    config_frames = 0
    
    def load_config():
        nonlocal sim_config
        try:
            with open(sim_config_path, "r") as f:
                sim_config = json.load(f)
            if use_simulator and hasattr(robot, 'update_config'):
                robot.update_config(sim_config)
        except Exception:
            pass
            
    load_config()

    while running:
        if use_simulator:
            config_frames += 1
            if config_frames >= 30:
                config_frames = 0
                load_config()
                
        c_vision_dist = sim_config.get("vision_reliable_dist", 0.45)
        c_stop_dist = sim_config.get("stop_finish_dist", 1.6)
        c_max_v = sim_config.get("max_v_target_sim", 0.8)
        c_min_v = sim_config.get("min_v_target_sim", 0.1)
        c_v_turn = sim_config.get("v_target_turn", 0.3)
        c_w_turn = sim_config.get("w_target_turn", 1.5)
        c_w_appr = sim_config.get("w_target_approach", 2.5)
        c_evas_f = sim_config.get("evasion_frontal_dist", 0.35)
        c_evas_g = sim_config.get("evasion_general_dist", 0.18)
        c_min_frames = sim_config.get("min_consecutive_frames", 4)
        c_cool_post = sim_config.get("cooldown_post_giro", 1.5)
        c_cool_stop = sim_config.get("cooldown_stop", 3.0)
        c_time_stop = sim_config.get("tiempo_espera_stop", 3.0)
        c_rad_amarillo = sim_config.get("radio_amarillo_suave", 0.7)
        c_rad_giro_f = sim_config.get("radio_giro_fuerte", 0.4)
        c_fac_rep_s = sim_config.get("factor_repulsion_suave", 1.0)
        c_fac_rep_f = sim_config.get("factor_repulsion_fuerte", 1.5)
        c_time_min_g = sim_config.get("tiempo_min_giro", 0.8)
        c_time_max_g = sim_config.get("tiempo_max_giro", 2.0)

        if not use_simulator:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
        if use_ui:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_l and use_simulator:
                        view_mode = "robot" if view_mode == "global" else "global"
                    elif event.key == pygame.K_SPACE:
                        paused = not paused
                        if paused:
                            history_index = len(history) - 1
                    elif event.key == pygame.K_LEFT and paused:
                        history_index = max(0, history_index - 1)
                        set_state(history[history_index])
                    elif event.key == pygame.K_RIGHT and paused:
                        history_index = min(len(history) - 1, history_index + 1)
                        set_state(history[history_index])

        # ========================================================
        # 1. PERCEPCIÓN (SIMULADA O REAL)
        # ========================================================
        lidar_scan_raw = robot.get_lidar_scan()
        if len(lidar_scan_raw) < 360:
            if not use_simulator:
                time.sleep(0.05)
            continue
            
        lidar_scan = [d if d >= 0.18 else robot.lidar_max_range for d in lidar_scan_raw]
        if use_yolo:
            vision_dets = robot.get_vision_detections()
        else:
            vision_dets = []
        
        lidar_points = []
        for i, dist_p in enumerate(lidar_scan):
            if dist_p < robot.lidar_max_range:
                lidar_points.append((dist_p * math.cos(math.radians(i)), dist_p * math.sin(math.radians(i))))
                
        intentos_render = []
        render_distancias = []
        render_margen = 0

        if not paused:
            if cooldown_senal > 0:
                cooldown_senal -= dt
            else:
                clase_ignorada = None  # Recuperamos la vista
                
            v_target = 0.0
            w_target = 0.0
            dist_frente_estricto = min(lidar_scan[0:15] + lidar_scan[345:360])

            # Filtramos la visión si el robot está ignorando alguna señal
            vision_dets_crudo = list(vision_dets)
            if clase_ignorada:
                vision_dets = [d for d in vision_dets if d['class'] != clase_ignorada]

            # --- Capturar QR ---
            if use_simulator:
                sim_qrs = robot.get_qr_detections()
                tiempo_sim_actual = time.time()
                
                if len(sim_qrs) > 0:
                    if not sim_qr_en_pantalla:
                        sim_contador_qr += 1
                        print(f"\n---> [SIM QR DETECTADO] #{sim_contador_qr} | Contenido: {sim_qrs[0]['content']} <---")
                        sim_qr_en_pantalla = True
                    sim_ultimo_tiempo_qr = tiempo_sim_actual
                else:
                    sim_qr_en_pantalla = False
                    if sim_contador_qr > 0 and (tiempo_sim_actual - sim_ultimo_tiempo_qr) >= 5.0:
                        sim_contador_qr = 0

            # ========================================================
            # 2. ACTUALIZAR TRACKER Y ESTADO SEGÚN YOLO
            # ========================================================
            if len(vision_dets) > 0:
                if tracker['class'] is not None and tracker['frames_lost'] < tracker['max_frames']:
                    mismas_clase = [d for d in vision_dets if d['class'] == tracker['class']]
                    if len(mismas_clase) > 0:
                        senal = sorted(mismas_clase, key=lambda d: abs(d['relative_angle']))[0]
                        tracker['consecutive_frames'] += 1
                        
                        ang_grados = int(math.degrees(senal['relative_angle']))
                        dist_lidar = min([lidar_scan[(ang_grados + i) % 360] for i in range(-5, 6)])
                        
                        tracker['relative_angle'] = senal['relative_angle']
                        tracker['distance'] = dist_lidar
                        tracker['frames_lost'] = 0
                    else:
                        tracker['frames_lost'] += 1
                else:
                    senal = sorted(vision_dets, key=lambda d: abs(d['relative_angle']))[0]
                    tracker['class'] = senal['class']
                    tracker['consecutive_frames'] = 1
                    
                    ang_grados = int(math.degrees(senal['relative_angle']))
                    dist_lidar = min([lidar_scan[(ang_grados + i) % 360] for i in range(-5, 6)])
                    tracker['relative_angle'] = senal['relative_angle']
                    tracker['distance'] = dist_lidar
                    tracker['frames_lost'] = 0
            else:
                tracker['frames_lost'] += 1

            if tracker['frames_lost'] >= tracker['max_frames']:
                tracker['consecutive_frames'] = 0
                tracker['class'] = None
                if estado_actual == "ACERCANDOSE_A_SENAL":
                    estado_actual = "EXPLORANDO"

            if tracker['frames_lost'] < tracker['max_frames'] and tracker['consecutive_frames'] >= c_min_frames:
                clase = tracker['class']
                dist = tracker['distance']
                
                if estado_actual in ["EXPLORANDO", "ACERCANDOSE_A_SENAL"]:
                    if clase == 'left':
                        if dist <= c_vision_dist:
                            estado_actual = "BUSCANDO_IZQ"
                        else:
                            estado_actual = "ACERCANDOSE_A_SENAL"
                    elif clase == 'right':
                        if dist <= c_vision_dist:
                            estado_actual = "BUSCANDO_DER"
                        else:
                            estado_actual = "ACERCANDOSE_A_SENAL"
                    elif clase == 'stop':
                        # EVITA EL CALLEJÓN: Si está cerca del Stop, prepara un giro de 180º
                        if dist <= c_stop_dist + 0.5:
                            estado_actual = "DETENIDO_PRE_180"
                            tiempo_estado = 1.0 # 1 Segundo de pausa
                        else:
                            estado_actual = "EXPLORANDO"
                    elif clase == 'finish':
                        if dist <= c_stop_dist:
                            estado_actual = "FINALIZADO"
                        else:
                            estado_actual = "ACERCANDOSE_A_SENAL"

            # ========================================================
            # 3. LÓGICA DE CADA ESTADO
            # ========================================================
            def calcular_repulsion(scan, rad_amarillo, rad_fuerte, fac_suave, fac_fuerte):
                min_izq = min(scan[0:180])
                min_der = min(scan[180:360])
                
                def calcular_fuerza(dist):
                    if dist >= rad_amarillo:
                        return 0.0
                    
                    intensidad = (rad_amarillo - dist) / rad_amarillo
                    
                    if dist <= rad_fuerte:
                        sobre_paso = (rad_fuerte - dist) / rad_fuerte
                        mult = fac_fuerte + (sobre_paso ** 2) * 5.0
                    else:
                        ratio = (rad_amarillo - dist) / (rad_amarillo - rad_fuerte)
                        mult = fac_suave + ratio * (fac_fuerte - fac_suave)
                        
                    return intensidad * mult

                f_izq = calcular_fuerza(min_izq)
                f_der = calcular_fuerza(min_der)
                return (f_der - f_izq)

            if estado_actual == "EXPLORANDO":
                v_target = max(c_min_v, min(c_max_v, (dist_frente_estricto - 0.4) * c_max_v))
                w_target = calcular_repulsion(lidar_scan, c_rad_amarillo, c_rad_giro_f, c_fac_rep_s, c_fac_rep_f) * 2.5
                w_target = max(-1.5, min(1.5, w_target))

            elif estado_actual == "ACERCANDOSE_A_SENAL":
                v_target = max(c_min_v, min(c_max_v, (dist_frente_estricto - 0.4) * c_max_v))
                w_camara = 0.0
                if tracker['frames_lost'] < tracker['max_frames']:
                    w_camara = tracker['relative_angle'] * c_w_appr
                w_repulsion = calcular_repulsion(lidar_scan, c_rad_amarillo, c_rad_giro_f, c_fac_rep_s, c_fac_rep_f) * 1.5
                w_target = w_camara + w_repulsion
                    
            elif estado_actual in ["BUSCANDO_IZQ", "BUSCANDO_DER"]:
                v_target = max(c_min_v, min(c_max_v, (dist_frente_estricto - 0.4) * c_max_v))
                w_camara = 0.0
                if tracker['frames_lost'] < tracker['max_frames']:
                    w_camara = tracker['relative_angle'] * c_w_appr
                w_repulsion = calcular_repulsion(lidar_scan, c_rad_amarillo, c_rad_giro_f, c_fac_rep_s, c_fac_rep_f) * 1.5
                w_target = w_camara + w_repulsion
                
                dir_search = 'left' if estado_actual == "BUSCANDO_IZQ" else 'right'
                
                espacio, _, intentos, dists, marg = buscar_camino_libre(lidar_points, robot.radius, dir_search, 0.10)
                if not espacio:
                    espacio, _, intentos, dists, marg = buscar_camino_libre(lidar_points, robot.radius, dir_search, 0.05)
                if not espacio:
                    espacio, _, intentos, dists, marg = buscar_camino_libre(lidar_points, robot.radius, dir_search, 0.0)
                
                intentos_render = intentos
                render_distancias = dists
                render_margen = marg
                
                if espacio:
                    estado_actual = "GIRANDO_IZQ" if estado_actual == "BUSCANDO_IZQ" else "GIRANDO_DER"
                    ultimo_giro = 'left' if estado_actual == "GIRANDO_IZQ" else 'right'
                    tiempo_estado = 0.0

            elif estado_actual in ["GIRANDO_IZQ", "GIRANDO_DER"]:
                v_target = c_v_turn 
                w_target = c_w_turn if estado_actual == "GIRANDO_IZQ" else -c_w_turn
                tiempo_estado += dt
                
                # Mantenemos buscar_camino_libre solo para dibujar el radar verde en la UI
                espacio_frente, _, intentos, dists, marg = buscar_camino_libre(lidar_points, robot.radius, 'front', 0.10)
                intentos_render = intentos
                render_distancias = dists
                render_margen = marg
                
                # --- NUEVA LÓGICA: Giro Matemático Exacto de 90 Grados ---
                tiempo_giro_90 = (math.pi / 2) / c_w_turn
                
                if tiempo_estado >= tiempo_giro_90: 
                    estado_actual = "EXPLORANDO"
                    cooldown_senal = c_cool_post
                    clase_ignorada = 'left' if ultimo_giro == 'left' else 'right'
                    tracker['frames_lost'] = 999
                    tracker['class'] = None

            elif estado_actual == "DETENIDO_PRE_180":
                v_target = 0.0
                w_target = 0.0
                tiempo_estado -= dt
                if tiempo_estado <= 0:
                    estado_actual = "GIRANDO_180"
                    # Giro Matemático de 180 Grados (PI)
                    tiempo_estado = math.pi / c_w_turn 

            elif estado_actual == "GIRANDO_180":
                # Al mantener v_target = 0, el robot gira sobre su propio eje sin rozar paredes
                v_target = 0.0 
                w_target = c_w_turn 
                tiempo_estado -= dt
                if tiempo_estado <= 0:
                    estado_actual = "EXPLORANDO"
                    cooldown_senal = c_cool_post
                    clase_ignorada = 'stop' # Ignora la señal STOP mientras se aleja de regreso
                    tracker['frames_lost'] = 999
                    tracker['class'] = None

            elif estado_actual == "FINALIZADO":
                v_target = 0.0
                w_target = 0.0

            # ========================================================
            # 4. ANTI-CHOQUES Y EVASIÓN
            # ========================================================
            riesgo_inminente = False
            min_dist_frontal = float('inf')
            for i in list(range(0, 21)) + list(range(339, 360)):
                if lidar_scan[i] < min_dist_frontal:
                    min_dist_frontal = lidar_scan[i]
            
            if min_dist_frontal < c_evas_f and v_target > 0.05:
                riesgo_inminente = True
                
            if riesgo_inminente or estado_actual == "EVASION_EMERGENCIA":
                if estado_actual != "EVASION_EMERGENCIA":
                    tiempo_estado = 0.0
                estado_actual = "EVASION_EMERGENCIA"
                
                v_target = 0.0 
                tiempo_estado += dt
                
                esp_escape, ang_escape, intentos, dists, marg = buscar_camino_libre(lidar_points, robot.radius, 'front', 0.02)
                intentos_render = intentos
                render_distancias = dists
                render_margen = marg
                
                if esp_escape:
                    ang_rel = ang_escape if ang_escape <= 180 else ang_escape - 360
                    w_target = math.radians(ang_rel) * 4.0
                    
                    if abs(ang_rel) < 15 and dist_frente_estricto > 0.4:
                        estado_actual = "EXPLORANDO"
                        cooldown_senal = c_cool_post
                        clase_ignorada = None 
                        tracker['frames_lost'] = 999
                        tracker['class'] = None
                        v_target = c_min_v
                else:
                    if cooldown_senal > 0:
                        w_target = 3.0 if ultimo_giro == 'left' else -3.0
                    else:
                        esp_escape_all, ang_escape_all, _, _, _ = buscar_camino_libre(lidar_points, robot.radius, 'all', 0.0)
                        if esp_escape_all:
                            ang_rel_all = ang_escape_all if ang_escape_all <= 180 else ang_escape_all - 360
                            w_target = 3.0 if ang_rel_all > 0 else -3.0
                        else:
                            w_target = 3.0

            # ========================================================
            # 5. ACTUACIÓN FÍSICA Y GUARDADO
            # ========================================================
            hubo_choque = robot.move(v_target, w_target, dt)
            if hubo_choque:
                choques += 1

            if not use_simulator and estado_actual == "FINALIZADO":
                print("\n\n¡Señal FINISH alcanzada! Deteniendo el robot y terminando el programa.")
                break

            time_since_save += dt
            if time_since_save >= 0.5:
                history.append(get_current_state())
                history_index = len(history) - 1
                time_since_save = 0.0
        else:
            v_target = 0.0
            w_target = 0.0
            vision_dets_crudo = []
            
            dir_search = 'front'
            margen = 0.10
            if estado_actual == "BUSCANDO_IZQ": dir_search = 'left'
            elif estado_actual == "BUSCANDO_DER": dir_search = 'right'
            elif estado_actual == "EVASION_EMERGENCIA": dir_search = 'any'; margen = 0.15
            
            lidar_points = []
            for i, dist_p in enumerate(lidar_scan):
                if dist_p < robot.lidar_max_range:
                    lidar_points.append((dist_p * math.cos(math.radians(i)), dist_p * math.sin(math.radians(i))))
                    
            esp, _, intentos_render, render_distancias, render_margen = buscar_camino_libre(lidar_points, robot.radius, dir_search, margen)
            if not esp and margen >= 0.10:
                esp, _, intentos_render, render_distancias, render_margen = buscar_camino_libre(lidar_points, robot.radius, dir_search, 0.05)
            if not esp and margen >= 0.05:
                _, _, intentos_render, render_distancias, render_margen = buscar_camino_libre(lidar_points, robot.radius, dir_search, 0.0)

        # ========================================================
        # 6. DIBUJADO Y TELEMETRÍA (LOGS MEJORADOS)
        # ========================================================
        tiempo_actual_log = time.time() if use_simulator else current_time
        if tiempo_actual_log - last_log_time > 0.33: 
            yolo_strs = [f"[{d['class'].upper()}] a {d['distance']:.2f}m (Ang: {math.degrees(d['relative_angle']):.0f}º)" for d in vision_dets_crudo]
            yolo_str = " | ".join(yolo_strs) if yolo_strs else "Nada a la vista"
            
            estado_str = estado_actual
            if cooldown_senal > 0 and clase_ignorada:
                estado_str += f" (IGNORA: {clase_ignorada.upper()})"
                
            print(f"[*] {estado_str:<35} -> OJOS VEN: {yolo_str}")
            last_log_time = tiempo_actual_log

        if not use_ui:
            time.sleep(1.0 / 30.0)
            continue
            
        screen.fill((10, 15, 10))
        font = pygame.font.SysFont(None, 24)
        
        if use_simulator:
            rx, ry, rtheta = robot._get_true_pose()
        
        if view_mode == "global" and use_simulator:
            for p1, p2 in world.obstacles:
                pygame.draw.line(screen, (100, 150, 100), to_screen(*p1), to_screen(*p2), 2)

            for sig in world.signals:
                sx, sy = to_screen(sig['x'], sig['y'])
                pygame.draw.circle(screen, (255, 255, 0), (sx, sy), 5)
                img = font.render(sig['type'], True, (255, 255, 0))
                screen.blit(img, (sx + 10, sy - 10))

            if hasattr(world, 'qrs'):
                for qr in world.qrs:
                    qx, qy = to_screen(qr['x'], qr['y'])
                    pygame.draw.rect(screen, (0, 255, 255), (qx - 5, qy - 5, 10, 10))
                    img = font.render(qr['content'], True, (0, 255, 255))
                    screen.blit(img, (qx + 10, qy - 10))

            rsx, rsy = to_screen(rx, ry)

            for intento in intentos_render:
                ang_c = intento['angulo']
                es_valido = intento['valido']
                color_circulo = (50, 200, 150) if es_valido else (200, 50, 50)
                
                for d_c in render_distancias:
                    cx = rx + d_c * math.cos(rtheta + math.radians(ang_c))
                    cy = ry + d_c * math.sin(rtheta + math.radians(ang_c))
                    scx, scy = to_screen(cx, cy)
                    pygame.draw.circle(screen, color_circulo, (scx, scy), int((render_margen)*SCALE), 1)

            angle_increment = (2 * math.pi) / robot.lidar_resolution
            for i, dist in enumerate(lidar_scan):
                if dist < robot.lidar_max_range:
                    ray_angle = rtheta + i * angle_increment
                    end_x = rx + dist * math.cos(ray_angle)
                    end_y = ry + dist * math.sin(ray_angle)
                    esx, esy = to_screen(end_x, end_y)
                    
                    if dist < 0.35 and (i < 120 or i > 240):
                        if dist < 0.20:
                            pygame.draw.circle(screen, (255, 0, 0), (esx, esy), 2)
                        else:
                            pygame.draw.circle(screen, (255, 165, 0), (esx, esy), 2)
                    else:
                        pygame.draw.circle(screen, (0, 255, 0), (esx, esy), 1)

            fov_l = rtheta + robot.camera_fov / 2
            fov_r = rtheta - robot.camera_fov / 2
            fl_x, fl_y = to_screen(rx + 2 * math.cos(fov_l), ry + 2 * math.sin(fov_l))
            fr_x, fr_y = to_screen(rx + 2 * math.cos(fov_r), ry + 2 * math.sin(fov_r))
            pygame.draw.line(screen, (0, 100, 255), (rsx, rsy), (fl_x, fl_y), 1)
            pygame.draw.line(screen, (0, 100, 255), (rsx, rsy), (fr_x, fr_y), 1)

            robot_px_radius = int(robot.radius * SCALE)
            
            pygame.draw.circle(screen, (200, 50, 50), (rsx, rsy), int(c_evas_g * SCALE), 1)
            pygame.draw.circle(screen, (255, 100, 0), (rsx, rsy), int(c_evas_f * SCALE), 1)
            pygame.draw.circle(screen, (200, 180, 50), (rsx, rsy), int(c_rad_giro_f * SCALE), 1)
            pygame.draw.circle(screen, (255, 255, 0), (rsx, rsy), int(c_rad_amarillo * SCALE), 1)
            
            pygame.draw.circle(screen, (60, 220, 60), (rsx, rsy), robot_px_radius)
            hx, hy = to_screen(rx + robot.radius * math.cos(rtheta), ry + robot.radius * math.sin(rtheta))
            pygame.draw.line(screen, (10, 15, 10), (rsx, rsy), (hx, hy), 3)

        else:
            rsx, rsy = OFFSET_X, OFFSET_Y
            
            angle_increment = (2 * math.pi) / robot.lidar_resolution
            for i, dist in enumerate(lidar_scan):
                if dist < robot.lidar_max_range:
                    screen_angle = math.pi / 2 + i * angle_increment
                    esx = int(rsx + dist * math.cos(screen_angle) * SCALE)
                    esy = int(rsy - dist * math.sin(screen_angle) * SCALE)
                    
                    if dist < 0.35 and (i < 120 or i > 240):
                        if dist < 0.20:
                            pygame.draw.circle(screen, (255, 0, 0), (esx, esy), 3)
                        else:
                            pygame.draw.circle(screen, (255, 165, 0), (esx, esy), 2)
                    else:
                        pygame.draw.circle(screen, (0, 255, 0), (esx, esy), 1)

            fov_l = math.pi/2 + robot.camera_fov / 2
            fov_r = math.pi/2 - robot.camera_fov / 2
            fl_x = int(rsx + 2 * math.cos(fov_l) * SCALE)
            fl_y = int(rsy - 2 * math.sin(fov_l) * SCALE)
            fr_x = int(rsx + 2 * math.cos(fov_r) * SCALE)
            fr_y = int(rsy - 2 * math.sin(fov_r) * SCALE)
            pygame.draw.line(screen, (50, 150, 50), (rsx, rsy), (fl_x, fl_y), 1)
            pygame.draw.line(screen, (50, 150, 50), (rsx, rsy), (fr_x, fr_y), 1)

            robot_px_radius = int(robot.radius * SCALE)
            pygame.draw.circle(screen, (60, 220, 60), (rsx, rsy), robot_px_radius)
            hx, hy = rsx, rsy - robot_px_radius
            pygame.draw.line(screen, (10, 15, 10), (rsx, rsy), (hx, hy), 3)
            
            for det in vision_dets_crudo:
                dist = det['distance']
                rel_a = det['relative_angle']
                screen_angle = math.pi / 2 + rel_a
                
                sx = int(rsx + dist * math.cos(screen_angle) * SCALE)
                sy = int(rsy - dist * math.sin(screen_angle) * SCALE)
                
                pygame.draw.circle(screen, (255, 255, 0), (sx, sy), 8)
                img = font.render(det['class'], True, (255, 255, 0))
                screen.blit(img, (sx + 15, sy - 10))

            for sqr in sim_qrs:
                dist = sqr['distance']
                rel_a = sqr['relative_angle']
                screen_angle = math.pi / 2 + rel_a
                
                sx = int(rsx + dist * math.cos(screen_angle) * SCALE)
                sy = int(rsy - dist * math.sin(screen_angle) * SCALE)
                
                pygame.draw.rect(screen, (0, 255, 255), (sx - 6, sy - 6, 12, 12))
                img = font.render(sqr['content'], True, (0, 255, 255))
                screen.blit(img, (sx + 15, sy - 10))

        fps = clock.get_fps()
        mode_text = f"SIMULADOR (FPS: {fps:.1f})" if use_simulator else f"ROBOT REAL (FPS: {1.0/max(0.001, dt):.1f})"
        screen.blit(pygame.font.SysFont(None, 36).render(f"[{mode_text}] Algoritmo: {estado_actual}", True, (60, 220, 60)), (10, 10))
        if cooldown_senal > 0 and clase_ignorada:
            screen.blit(pygame.font.SysFont(None, 24).render(f"(Ignorando {clase_ignorada.upper()} por: {cooldown_senal:.1f}s)", True, (255, 200, 0)), (400, 15))
            
        screen.blit(pygame.font.SysFont(None, 28).render(f"v={v_target:.2f}, w={w_target:.2f}", True, (150, 200, 150)), (10, 45))

        color_choque = (255, 100, 100) if choques > 0 else (60, 220, 60)
        screen.blit(pygame.font.SysFont(None, 28).render(f"Choques: {choques}", True, color_choque), (10, 75))

        y_offset = 105
        for det in vision_dets_crudo:
            text = f"YOLO Ve: '{det['class']}' a {det['distance']:.2f}m (Ang: {math.degrees(det['relative_angle']):.1f}º)"
            color_txt = (100, 100, 100) if det['class'] == clase_ignorada else (200, 255, 200)
            img = font.render(text, True, color_txt)
            screen.blit(img, (10, y_offset))
            y_offset += 25
            
        for sqr in sim_qrs:
            text = f"QR Escaneado: '{sqr['content']}' a {sqr['distance']:.2f}m (Ang: {math.degrees(sqr['relative_angle']):.1f}º)"
            img = font.render(text, True, (0, 255, 255))
            screen.blit(img, (10, y_offset))
            y_offset += 25

        if paused:
            pause_text = pygame.font.SysFont(None, 48).render("PAUSADO - USA FLECHAS (<- ->) PARA TIEMPO", True, (255, 100, 100))
            screen.blit(pause_text, (WIDTH//2 - pause_text.get_width()//2, HEIGHT - 80))
            
            state_text = pygame.font.SysFont(None, 36).render(f"Snapshot: {history_index+1} / {len(history)}", True, (255, 200, 100))
            screen.blit(state_text, (WIDTH//2 - state_text.get_width()//2, HEIGHT - 40))

        pygame.display.flip()
        clock.tick(30)

    if qr_thread is not None:
        print("\nApagando escáner QR...")
        qr_thread.stop()
        qr_thread.join(timeout=1.0)
        print("Escáner QR detenido correctamente.")

if __name__ == "__main__":
    main()