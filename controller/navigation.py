"""
Controlador de navegación compartido entre el simulador y el robot real (T1 del ROADMAP).

Contiene la máquina de estados (EXPLORANDO / BUSCANDO_* / GIRANDO_* / DETENIDO /
FINALIZADO / EVASION_EMERGENCIA) y `buscar_camino_libre`, antes duplicadas verbatim
en `test_autonomous_controller.py` y `run_real_autonomous.py`.

Es puro: sin pygame, sin ROS, sin I/O. Recibe sensores, devuelve (v, w).
Las diferencias legítimas entre plataformas son parámetros del constructor:
  - v_max: 0.8 en el simulador, 0.3 en el robot real (límite de hardware).
  - lidar_min_valid: None en el simulador, 0.18 en el real (reflexiones del chasis).
Todos los números mágicos están expuestos como parámetros para la optimización
automática (T8).
"""
import math

import numpy as np


def buscar_camino_libre(lidar_points, radio_robot, direccion='front', margen_extra=0.10):
    """
    Barrido geométrico: ¿existe un rumbo por el que avanzar en línea recta sin chocar
    con nada que ya ve el LiDAR? Prueba un abanico discreto de ángulos y, para cada
    uno, M puntos a lo largo del rayo contra los puntos del LiDAR inflados por
    `radio_robot + margen_extra`. Devuelve el PRIMER ángulo libre según el orden
    de prioridad de la lista.
    """
    if direccion == 'left':
        angulos = [50, 63, 76, 90, 103, 116, 130]
        M = 5
    elif direccion == 'right':
        angulos = [230, 243, 256, 270, 283, 296, 310]
        M = 5
    elif direccion == 'front':
        angulos = [-20, -13, -6, 0, 6, 13, 20]
        M = 5
    else:
        angulos = range(0, 360, 30)
        M = 3

    margen = radio_robot + margen_extra
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
                if math.hypot(px - cx, py - cy) < margen:
                    choca = True
                    break

            if choca:
                ruta_valida = False
                break

        intentos.append({'angulo': ang_eval, 'valido': ruta_valida})
        if ruta_valida and mejor_ang is None:
            mejor_ang = ang_eval

    return mejor_ang is not None, mejor_ang, intentos, distancias_prueba, margen


class NavigationController:
    def __init__(
        self,
        robot_radius: float,
        lidar_max_range: float,
        # Velocidad de crucero: v = clip((frente - v_offset) * v_gain, v_min, v_max)
        v_min: float = 0.1,
        v_max: float = 0.8,          # 0.3 en el robot real (límite de hardware)
        v_gain: float = 0.8,
        v_offset: float = 0.4,
        # Filtro de validez del LiDAR al construir lidar_points (None = sin filtro)
        lidar_min_valid: float = None,  # 0.18 en el real: reflexiones del chasis
        # Repulsión de paredes en EXPLORANDO
        repulsion_range: float = 0.7,
        repulsion_boost_dist: float = 0.4,
        repulsion_gain: float = 80.0,
        # Tracking visual de la señal en BUSCANDO_*
        track_gain: float = 2.5,
        # Giro de ~80° (a lazo abierto; T5 lo cerrará con odometría)
        turn_speed: float = 2.0,
        turn_duration: float = 0.7,
        post_turn_cooldown: float = 0.2,
        # Señales stop / finish
        stop_sign_max_dist: float = 1.6,
        stop_duration: float = 3.0,
        post_stop_cooldown: float = 3.0,
        finish_max_dist: float = 1.6,
        # Evasión de emergencia
        front_risk_dist: float = 0.32,
        contact_risk_dist: float = 0.19,
        escape_gain: float = 4.0,
        escape_align_deg: float = 15.0,
        escape_min_front: float = 0.4,
        escape_spin: float = 3.0,
        # Márgenes de buscar_camino_libre
        search_margin: float = 0.10,
        escape_margin: float = 0.02,
        # Si True, corre barridos extra solo para visualización (sim)
        collect_debug: bool = False,
    ):
        self.robot_radius = robot_radius
        self.lidar_max_range = lidar_max_range
        self.v_min = v_min
        self.v_max = v_max
        self.v_gain = v_gain
        self.v_offset = v_offset
        self.lidar_min_valid = lidar_min_valid
        self.repulsion_range = repulsion_range
        self.repulsion_boost_dist = repulsion_boost_dist
        self.repulsion_gain = repulsion_gain
        self.track_gain = track_gain
        self.turn_speed = turn_speed
        self.turn_duration = turn_duration
        self.post_turn_cooldown = post_turn_cooldown
        self.stop_sign_max_dist = stop_sign_max_dist
        self.stop_duration = stop_duration
        self.post_stop_cooldown = post_stop_cooldown
        self.finish_max_dist = finish_max_dist
        self.front_risk_dist = front_risk_dist
        self.contact_risk_dist = contact_risk_dist
        self.escape_gain = escape_gain
        self.escape_align_deg = escape_align_deg
        self.escape_min_front = escape_min_front
        self.escape_spin = escape_spin
        self.search_margin = search_margin
        self.escape_margin = escape_margin
        self.collect_debug = collect_debug

        # Estado interno de la máquina de estados
        self.estado = "EXPLORANDO"
        self.tiempo_estado = 0.0
        self.cooldown_senal = 0.0

        # Última señal considerada en este paso (para logs de la shell)
        self.last_signal = None
        # Distancia frontal del último paso (para logs de la shell)
        self.dist_frente = 0.0
        # Datos del último barrido, para render de debug en el simulador
        self.debug = {'intentos': [], 'distancias': [], 'margen': 0.0}

    # ------------------------------------------------------------------
    # Snapshot / restore: soporte del time-travel del simulador
    # ------------------------------------------------------------------
    def snapshot(self) -> dict:
        return {
            'estado': self.estado,
            'tiempo_estado': self.tiempo_estado,
            'cooldown_senal': self.cooldown_senal,
        }

    def restore(self, snap: dict):
        self.estado = snap['estado']
        self.tiempo_estado = snap['tiempo_estado']
        self.cooldown_senal = snap['cooldown_senal']

    # ------------------------------------------------------------------
    # Cálculos auxiliares
    # ------------------------------------------------------------------
    def _front_clearance(self, scan: np.ndarray) -> float:
        """Mínimo del cono frontal estricto de ±15°."""
        return float(min(np.min(scan[0:15]), np.min(scan[345:360])))

    def _cruise_speed(self, dist_frente: float) -> float:
        return max(self.v_min, min(self.v_max, (dist_frente - self.v_offset) * self.v_gain))

    def _lidar_points(self, scan: np.ndarray) -> list:
        points = []
        for i, dist_p in enumerate(scan):
            if dist_p >= self.lidar_max_range:
                continue
            if self.lidar_min_valid is not None and dist_p <= self.lidar_min_valid:
                continue
            points.append((dist_p * math.cos(math.radians(i)), dist_p * math.sin(math.radians(i))))
        return points

    def _set_debug(self, intentos, distancias, margen):
        self.debug = {'intentos': intentos, 'distancias': distancias, 'margen': margen}

    # ------------------------------------------------------------------
    # Un paso de control
    # ------------------------------------------------------------------
    def step(self, lidar_scan, vision_dets, dt: float):
        """
        lidar_scan: 360 distancias (m), índice 0 = frente, creciendo CCW.
        vision_dets: [{'class', 'distance', 'relative_angle'}, ...]
        Devuelve (v, w) objetivo.
        """
        scan = np.asarray(lidar_scan, dtype=float)

        if self.cooldown_senal > 0:
            self.cooldown_senal -= dt

        lidar_points = self._lidar_points(scan)
        dist_frente_estricto = self._front_clearance(scan)
        self.dist_frente = dist_frente_estricto

        v_target = 0.0
        w_target = 0.0
        self.last_signal = None
        self._set_debug([], [], 0.0)

        # ========================================================
        # 1. ACTUALIZAR ESTADO SEGÚN VISIÓN (transiciones)
        # ========================================================
        if len(vision_dets) > 0 and self.cooldown_senal <= 0:
            senal = sorted(vision_dets, key=lambda d: d['distance'])[0]
            clase = senal['class']
            dist = senal['distance']
            self.last_signal = (clase, dist)

            if self.estado == "EXPLORANDO":
                if clase == 'left':
                    self.estado = "BUSCANDO_IZQ"
                elif clase == 'right':
                    self.estado = "BUSCANDO_DER"
                elif clase == 'stop' and dist <= self.stop_sign_max_dist:
                    self.estado = "DETENIDO"
                    self.tiempo_estado = self.stop_duration
                elif clase == 'finish' and dist <= self.finish_max_dist:
                    self.estado = "FINALIZADO"

        # ========================================================
        # 2. LÓGICA DE CADA ESTADO
        # ========================================================
        if self.estado == "EXPLORANDO":
            v_target = self._cruise_speed(dist_frente_estricto)

            min_dist = min(scan)
            min_angle = np.argmin(scan)
            if min_angle > 180:
                min_angle -= 360

            if min_dist < self.repulsion_range:
                factor_giro = 1.5 if min_dist < self.repulsion_boost_dist else 1.0
                margen = self.repulsion_range
                if min_angle >= 0:
                    target = 90 + (margen - min_dist) * self.repulsion_gain
                    w_target -= math.radians(target - min_angle) * factor_giro
                else:
                    target = -90 - (margen - min_dist) * self.repulsion_gain
                    w_target -= math.radians(target - min_angle) * factor_giro

        elif self.estado in ("BUSCANDO_IZQ", "BUSCANDO_DER"):
            v_target = self._cruise_speed(dist_frente_estricto)

            # Centrar la señal en la imagen mientras avanzamos
            if len(vision_dets) > 0:
                senal = sorted(vision_dets, key=lambda d: d['distance'])[0]
                w_target = senal['relative_angle'] * self.track_gain

            dir_search = 'left' if self.estado == "BUSCANDO_IZQ" else 'right'
            espacio, _, intentos, dists, marg = buscar_camino_libre(
                lidar_points, self.robot_radius, dir_search, self.search_margin)
            self._set_debug(intentos, dists, marg)

            if espacio:
                self.estado = "GIRANDO_IZQ" if self.estado == "BUSCANDO_IZQ" else "GIRANDO_DER"
                self.tiempo_estado = 0.0

        elif self.estado in ("GIRANDO_IZQ", "GIRANDO_DER"):
            v_target = self._cruise_speed(dist_frente_estricto)
            w_target = self.turn_speed if self.estado == "GIRANDO_IZQ" else -self.turn_speed
            self.tiempo_estado += dt

            if self.collect_debug:
                # Barrido frontal solo para visualización en el simulador
                _, _, intentos, dists, marg = buscar_camino_libre(
                    lidar_points, self.robot_radius, 'front', self.search_margin)
                self._set_debug(intentos, dists, marg)

            if self.tiempo_estado >= self.turn_duration:
                self.estado = "EXPLORANDO"
                self.cooldown_senal = self.post_turn_cooldown

        elif self.estado == "DETENIDO":
            v_target = 0.0
            w_target = 0.0
            self.tiempo_estado -= dt
            if self.tiempo_estado <= 0:
                self.estado = "EXPLORANDO"
                self.cooldown_senal = self.post_stop_cooldown

        elif self.estado == "FINALIZADO":
            # Meta alcanzada: se queda detenido permanentemente.
            v_target = 0.0
            w_target = 0.0

        # ========================================================
        # 3. ANTI-CHOQUES Y EVASIÓN DE EMERGENCIA (última palabra)
        # ========================================================
        riesgo_inminente = False
        min_dist_frontal = float(min(np.min(scan[0:45]), np.min(scan[315:360])))

        if min_dist_frontal < self.front_risk_dist and v_target > 0.05:
            riesgo_inminente = True
        if min(scan) < self.contact_risk_dist:
            riesgo_inminente = True

        if riesgo_inminente or self.estado == "EVASION_EMERGENCIA":
            if self.estado != "EVASION_EMERGENCIA":
                self.tiempo_estado = 0.0
            self.estado = "EVASION_EMERGENCIA"

            v_target = 0.0
            self.tiempo_estado += dt

            esp_escape, ang_escape, intentos, dists, marg = buscar_camino_libre(
                lidar_points, self.robot_radius, 'front', self.escape_margin)
            self._set_debug(intentos, dists, marg)

            if esp_escape:
                ang_rel = ang_escape if ang_escape <= 180 else ang_escape - 360
                w_target = math.radians(ang_rel) * self.escape_gain

                if abs(ang_rel) < self.escape_align_deg and dist_frente_estricto > self.escape_min_front:
                    self.estado = "EXPLORANDO"
            else:
                w_target = self.escape_spin

        return v_target, w_target
