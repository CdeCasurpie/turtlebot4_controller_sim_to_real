# -*- coding: utf-8 -*-
"""
Controlador autónomo UNIFICADO — el mismo código corre en el simulador y en el
robot real (run_real_autonomous.py). Esto elimina la clase de bugs donde la
sintonización en pygame usaba una semántica distinta a la desplegada.

Diseño (reemplaza a la FSM reactiva anterior):

  * Crucero por FOLLOW-THE-GAP (estilo F1TENTH): en vez de "huir del punto más
    cercano" (que reacciona incluso a obstáculos DETRÁS del robot), se busca el
    hueco libre más profundo dentro del abanico frontal y se apunta a su centro.
    En un pasillo, el centro del hueco ES el centro del pasillo: el robot se
    auto-centra sin lógica extra.

  * Giros CERRADOS por odometría: al girar se fija un yaw objetivo
    (yaw_actual ± 90°) y se controla en lazo cerrado hasta alcanzarlo, en vez
    del giro por tiempo (w=2.0 por 0.7 s) en lazo abierto, que con el tope real
    del Create 3 (1.9 rad/s) giraba ~75° en sim y menos en el robot.

  * Giro EN LA INTERSECCIÓN CORRECTA: la señal arma el giro, pero éste solo se
    dispara cuando (a) el robot ya recorrió la distancia estimada hasta la
    señal (odometría) y (b) la ventana lateral del LIDAR ve una abertura
    profunda sostenida — no "el primer hueco libre de 60 cm" como antes.

Interfaz:
    ctrl = AutonomousController(params_dict_opcional)
    v, w, estado = ctrl.step(scan, detections, dt, yaw=..., pose=(x, y))

  - scan: lista/array de 360 distancias, índice 0 = FRENTE, antihorario
    (90 = izquierda). Puede venir vacío (lidar caído) -> frena.
  - detections: [{'class': 'left'|'right'|'stop', 'distance': m,
                  'relative_angle': rad}, ...]
  - yaw: orientación de odometría en radianes (relativa; solo se usan
    DIFERENCIAS, el drift lento no afecta). None -> se integra el comando.
  - pose: (x, y) de odometría para medir distancia recorrida. None -> se
    integra v comandada.
"""
import math
import numpy as np

DEFAULT_PARAMS = {
    # --- límites físicos -------------------------------------------------
    "v_max": 0.30,            # 0.306 seguro / 0.46 con safety_override=full
    "v_min": 0.08,
    "w_max": 1.80,            # tope real Create 3: 1.90 rad/s
    "lidar_max_range": 12.0,
    "min_valid": 0.18,        # < esto = reflejo del propio chasis -> descartar
    # --- crucero follow-the-gap ------------------------------------------
    "horizon": 3.0,           # se recorta el scan a esto (evita que un rayo
                              # lejano domine la elección del hueco)
    "cruise_sector": 70,      # medio-abanico (grados) donde se buscan huecos
    "gap_min_depth": 0.85,    # un rayo cuenta como "libre" si ve más que esto
    "bubble_margin": 0.04,    # holgura lateral extra sobre el radio del robot.
                              # OJO: el circuito tiene pasillos de ~0.6 m para un
                              # robot de 0.34 m; con márgenes generosos NINGÚN
                              # camino es "transitable" y el robot se congela.
    "kp_gap": 1.5,            # w = kp * ángulo_al_centro_del_hueco
    "min_gap_deg": 6,         # descartar huecos más angostos que esto: una
                              # rendija de 1-2 grados entre paredes puede verse
                              # profundísima pero es físicamente infranqueable
    "v_kp": 0.9,              # v crece con el espacio frontal libre
    "v_offset": 0.30,
    "w_slowdown": 0.9,        # frena en curvas: v /= (1 + w_slowdown*|w|)
    # --- detección de la abertura lateral (intersección) ------------------
    "side_lo": 70,            # ventana lateral: 70..110 grados (izq; der espejo)
    "side_hi": 110,
    "opening_depth": 1.30,    # mediana de la ventana > esto = abertura real
    "opening_consec": 3,      # frames de control consecutivos para confirmar
    # Distancia a la señal a la que se "arma" el giro. En el circuito las
    # señales están colocadas EN o DETRÁS de su intersección (la 'left' de
    # x=3.89 corresponde a la abertura de x=2.9, verificado con la vuelta
    # exitosa de referencia), así que la regla correcta es: señal vista ->
    # tomar la SIGUIENTE abertura de ese lado. Armado inmediato (valor grande).
    "arm_dist": 99.0,
    "search_timeout": 15.0,   # s en BUSCANDO sin abertura -> volver a explorar
    "sign_memory_radius": 0.9,  # detecciones a < esto de una señal ya actuada se ignoran
    "sign_memory_time": 120.0,  # s que dura esa memoria. Larga a propósito: si
                                # expira en ~1 periodo de bucle, la misma señal
                                # re-dispara el mismo giro y el robot orbita
    # --- giro cerrado por odometría ---------------------------------------
    "turn_deg": 90.0,
    "kp_turn": 2.5,
    "w_turn_min": 0.5,        # magnitud mínima para no "morir" cerca del objetivo
    "turn_exit_deg": 10.0,
    "turn_v": 0.12,           # avance suave durante el giro (entra a la bocacalle)
    "cooldown_after_turn": 1.5,
    # --- señal de stop -----------------------------------------------------
    "stop_dist": 1.6,
    "stop_time": 3.0,
    "cooldown_after_stop": 3.0,
    "stop_ignore_time": 12.0, # tras parar, ignorar 'stop' hasta dejar atrás la
                              # MISMA señal (si no: parar-avanzar-parar eterno)
    # --- emergencia --------------------------------------------------------
    # Caja de BARRIDO, no sector circular: riesgo = hay un obstáculo dentro
    # del ancho del robot (|lateral| < emerg_lat) y a menos de emerg_dist por
    # delante. Un sector circular castigaba ir junto a una pared lateral que
    # no está en el camino, y en pasillos angostos eso paraliza al robot.
    "emerg_dist": 0.30,       # metros hacia adelante
    "emerg_lat": 0.185,       # medio-ancho vigilado. Apenas radio + 1.5 cm:
                              # las ranuras del circuito dejan ~3 cm por lado;
                              # con margen más gordo son "infranqueables" y el
                              # robot no puede cruzarlas nunca
    "emerg_exit": 0.45,       # histéresis: salir cuando el camino esté libre hasta aquí
    "contact_dist": 0.19,     # radio físico (17 cm) + ruido
}


class AutonomousController:
    def __init__(self, params=None):
        self.p = dict(DEFAULT_PARAMS)
        if params:
            self.p.update({k: v for k, v in params.items() if k in self.p})

        self.estado = "EXPLORANDO"
        self._cooldown = 0.0
        self._t_estado = 0.0
        # BUSCANDO_*
        self._dist_senal = 0.0     # distancia estimada a la señal al verla
        self._recorrido = 0.0      # avanzado desde la última vez que se vio
        self._consec_abertura = 0
        self._stop_ignore = 0.0
        # Memoria de señales YA ACTUADAS: [(x, y, clase, t_expira)]. Tras
        # ejecutar un giro, se ignoran las detecciones que apunten a ESA misma
        # señal (posición estimada = pose + rumbo + distancia), pero no a otra
        # señal de la misma clase. Sin esto, la señal recién obedecida sigue
        # visible tras el giro y encadena giro-tras-giro ("piruetas"); con un
        # bloqueo por clase entera, en cambio, se pierden los dobles giros
        # legítimos (left seguida de otra left) que este circuito sí tiene.
        self._senales_actuadas = []
        self._senal_xy = None      # posición estimada de la señal en curso
        self._t_total = 0.0
        # GIRANDO_*
        self._yaw_objetivo = 0.0
        self._estado_previo = "EXPLORANDO"
        # Cronómetro PROPIO del giro: corre también durante las evasiones que
        # lo interrumpen. Si compartiera _t_estado, cada evasión lo resetearía
        # y un giro imposible (hacia una pared) ciclaría giro<->evasión eterno.
        self._t_giro = 0.0
        # fallbacks de odometría (si el llamador no pasa yaw/pose)
        self._yaw_int = 0.0
        self._pose_int = (0.0, 0.0)
        self._pose_prev = None
        self._last_cmd = (0.0, 0.0)

    # ------------------------------------------------------------------
    # utilidades de scan (índice 0 = frente, antihorario)
    # ------------------------------------------------------------------
    def _sanear(self, scan):
        s = np.asarray(scan, dtype=float)
        # Filtro de mediana-3: un rayo espurio AISLADO (reflejo del chasis,
        # ruido del driver) toma el valor de sus vecinos. Sin esto, un reflejo
        # convertido a max_range se vuelve el "rayo más profundo" fantasma y
        # sesga la elección de huecos.
        s = np.median(np.stack([np.roll(s, -1), s, np.roll(s, 1)]), axis=0)
        # reflejos residuales del chasis -> "sin obstáculo"
        s = np.where(s < self.p["min_valid"], self.p["lidar_max_range"], s)
        return s

    def _abanico(self, s, half_deg):
        """Rayos desde -half_deg hasta +half_deg (índice 0 del resultado = -half)."""
        idx = np.arange(-half_deg, half_deg + 1) % 360
        return s[idx]

    def _mejor_hueco(self, s, half_deg):
        """Follow-the-gap: retorna (ángulo_rad al centro del mejor hueco,
        profundidad frontal) o (None, prof) si no hay hueco transitable."""
        d = np.minimum(self._abanico(s, half_deg), self.p["horizon"])

        # Un rayo es "libre" si ve más que gap_min_depth. Cada rayo bloqueado
        # además EROSIONA a sus vecinos en el ángulo que el robot necesitaría
        # para pasar junto a ese obstáculo sin rozarlo (más cerca el obstáculo,
        # más ancho el cono prohibido). Esto reemplaza a la "burbuja" clásica,
        # que con una pared lateral cercana anulaba el abanico entero.
        # Umbral ADAPTATIVO: en una esquina cerrada del pasillo puede que
        # ningún rayo supere gap_min_depth; exigirlo en absoluto congela al
        # robot. Siempre hay "hueco" hacia la región más profunda visible.
        umbral = min(self.p["gap_min_depth"], 0.70 * float(np.max(d)))
        base = d >= umbral
        holgura_max = 0.17 + self.p["bubble_margin"]

        # RELAJACIÓN PROGRESIVA de la holgura lateral: en los tramos angostos
        # del circuito (~0.6 m para un robot de 0.34 m) la erosión con margen
        # nominal puede anular todos los huecos. Congelarse es peor que
        # avanzar despacio con menos margen: la capa de emergencia sigue
        # cubriendo la colisión real.
        mejor = None
        for factor in (1.0, 0.6, 0.3, 0.0):
            holgura = holgura_max * factor
            mask = base.copy()
            if holgura > 0:
                for i in np.flatnonzero(~base):
                    # una trayectoria recta a |beta| grados del rayo bloqueado
                    # pasa a d*sin(beta) del obstáculo -> prohibir
                    # |beta| < asin(h/d). Tope de 45°: una pared PEGADA al
                    # costado daría 90° y anularía el abanico entero.
                    ratio = min(1.0, holgura / max(d[i], holgura))
                    half_b = min(45, int(math.degrees(math.asin(ratio))))
                    mask[max(0, i - half_b):i + half_b + 1] = False
            if not mask.any():
                continue

            # Huecos = corridas contiguas de rayos libres.
            cambios = np.flatnonzero(np.diff(mask.astype(int)))
            inicios = [0] if mask[0] else []
            inicios += [c + 1 for c in cambios if mask[c + 1]]
            finales = [c for c in cambios if mask[c]]
            if mask[-1]:
                finales.append(len(mask) - 1)

            # Puntuar por PROFUNDIDAD ante todo (follow-the-gap clásico: ir
            # hacia el punto más lejano); ancho y centralidad desempatan. En
            # un pasillo recto el hueco frontal ya es el más profundo; en un
            # rincón, la abertura profunda ES la ruta. El ancho mínimo se
            # exige DENTRO de la relajación: si en este nivel solo quedan
            # rendijas infranqueables, se relaja otro escalón.
            mejor_score = -1e9
            for a, b in zip(inicios, finales):
                ancho = b - a + 1
                if ancho < self.p["min_gap_deg"]:
                    continue
                centro = (a + b) / 2.0 - half_deg      # grados relativos al frente
                prof = float(np.max(d[a:b + 1]))
                # centralidad 0.012/°: suficiente para NO desviarse a huecos
                # laterales en cruces sin señal (seguir el pasillo), pero un
                # frente sin salida (prof<1m) sigue perdiendo contra una
                # abertura lateral profunda (3.0 - 90*0.012 = 1.9 > 1.0)
                score = prof + 0.01 * ancho - 0.012 * abs(centro)
                if score > mejor_score:
                    mejor_score, mejor = score, (a, b, centro)
            if mejor is not None:
                break

        if mejor is None:
            return None, float(np.min(self._abanico(s, 15)))
        a, b, centro = mejor
        # Apuntar al punto MÁS PROFUNDO del hueco suavizado hacia su centro:
        # en un pasillo recto ambos coinciden; en una curva anticipa el giro.
        tramo = d[a:b + 1]
        i_prof = a + int(np.argmax(tramo))
        objetivo = 0.65 * ((a + b) / 2.0) + 0.35 * i_prof - half_deg
        frente = float(np.min(self._abanico(s, 15)))
        return math.radians(objetivo), frente

    # ------------------------------------------------------------------
    def _v_crucero(self, frente, w):
        v = self.p["v_kp"] * (frente - self.p["v_offset"])
        v = max(self.p["v_min"], min(self.p["v_max"], v))
        return v / (1.0 + self.p["w_slowdown"] * abs(w))

    @staticmethod
    def _ang_diff(a, b):
        return math.atan2(math.sin(a - b), math.cos(a - b))

    @staticmethod
    def _min_movil(s, half=2):
        """Mínimo móvil de (2*half+1)°: la profundidad 'pesimista' sobre el
        ancho del robot. Una rendija de 1-2° profundísima desaparece; un
        pasillo real conserva su profundidad."""
        return np.min(np.stack([np.roll(s, k) for k in range(-half, half + 1)]), axis=0)

    def _claro_por_heading(self, s, paso=3):
        """Para cada heading candidato (cada `paso` grados), la distancia que
        el robot podría avanzar RECTO en esa dirección sin que nada invada su
        ancho (|lateral| < emerg_lat). Es el MISMO criterio que usa la capa de
        emergencia, así la evasión nunca elige un rumbo que la emergencia
        vetaría un frame después."""
        K = np.arange(0, 360, paso)
        rel = np.radians(np.arange(360)[None, :] - K[:, None])
        cos_rel = np.cos(rel)
        fwd = s[None, :] * cos_rel
        lat = np.abs(s[None, :] * np.sin(rel))
        en_camino = (lat < self.p["emerg_lat"]) & (cos_rel > 0)
        return K, np.where(en_camino, fwd, np.inf).min(axis=1)

    # ------------------------------------------------------------------
    def step(self, scan, detections, dt, yaw=None, pose=None):
        p = self.p

        # ---------- odometría (o fallback integrado) ----------
        if yaw is None:
            yaw = self._yaw_int
        if pose is None:
            pose = self._pose_int
        if self._pose_prev is not None:
            paso = math.hypot(pose[0] - self._pose_prev[0], pose[1] - self._pose_prev[1])
        else:
            paso = abs(self._last_cmd[0]) * dt
        self._pose_prev = pose
        self._t_total += dt

        # ---------- sin lidar: frenar ----------
        if scan is None or len(scan) < 360:
            self._last_cmd = (0.0, 0.0)
            return 0.0, 0.0, "SIN_LIDAR"

        s = self._sanear(scan)
        if self._cooldown > 0:
            self._cooldown -= dt
        if self._stop_ignore > 0:
            self._stop_ignore -= dt

        # ---------- filtrado de señales ya actuadas ----------
        self._senales_actuadas = [sa for sa in self._senales_actuadas
                                  if sa[3] > self._t_total]

        def _pos_estimada(det):
            a = yaw + det["relative_angle"]
            return (pose[0] + det["distance"] * math.cos(a),
                    pose[1] + det["distance"] * math.sin(a))

        def _ya_actuada(det):
            ex, ey = _pos_estimada(det)
            return any(c == det["class"] and
                       math.hypot(ex - sx, ey - sy) < p["sign_memory_radius"]
                       for sx, sy, c, _ in self._senales_actuadas)

        # ---------- transiciones por visión ----------
        utiles = [d for d in detections
                  if d["class"] == "stop" or not _ya_actuada(d)]
        if utiles and self._cooldown <= 0 and self.estado in ("EXPLORANDO", "BUSCANDO_IZQ", "BUSCANDO_DER"):
            det = sorted(utiles, key=lambda d: d["distance"])[0]
            clase, dist = det["class"], det["distance"]
            if clase == "stop" and dist <= p["stop_dist"] and self._stop_ignore <= 0:
                self.estado, self._t_estado = "DETENIDO", p["stop_time"]
            elif clase == "left" and self.estado == "EXPLORANDO":
                self.estado, self._t_estado = "BUSCANDO_IZQ", 0.0
                self._dist_senal, self._recorrido, self._consec_abertura = dist, 0.0, 0
                self._senal_xy = _pos_estimada(det)
            elif clase == "right" and self.estado == "EXPLORANDO":
                self.estado, self._t_estado = "BUSCANDO_DER", 0.0
                self._dist_senal, self._recorrido, self._consec_abertura = dist, 0.0, 0
                self._senal_xy = _pos_estimada(det)
            elif self.estado in ("BUSCANDO_IZQ", "BUSCANDO_DER") and clase in ("left", "right"):
                # la señal sigue visible: refrescar distancia y posición
                self._dist_senal, self._recorrido = dist, 0.0
                self._senal_xy = _pos_estimada(det)

        v, w = 0.0, 0.0

        # Claro barrido hacia el frente: distancia que se puede avanzar recto
        # sin que nada invada el ancho del robot. Gobierna la velocidad y el
        # disparo de emergencia (mismo criterio en ambos).
        rel_f = np.radians(np.arange(-90, 91))
        dd_f = self._abanico(s, 90)
        lat_f = np.abs(dd_f * np.sin(rel_f))
        fwd_f = dd_f * np.cos(rel_f)
        en_cam = (lat_f < p["emerg_lat"]) & (fwd_f > 0)
        dist_camino = float(np.min(np.where(en_cam, fwd_f, np.inf)))

        # ---------- lógica por estado ----------
        if self.estado == "EXPLORANDO":
            ang, frente = self._mejor_hueco(s, p["cruise_sector"])
            if ang is None:
                # Sin hueco transitable adelante (bolsillo/callejón): rotar en
                # el sitio hacia la dirección más profunda de TODO el barrido
                # de 360° — la salida de un callejón está DETRÁS del robot.
                i_prof = int(np.argmax(self._min_movil(s)))
                ang_prof = i_prof if i_prof <= 180 else i_prof - 360
                w = 1.2 if ang_prof >= 0 else -1.2
                v = 0.0
            else:
                w = max(-p["w_max"], min(p["w_max"], p["kp_gap"] * ang))
                v = self._v_crucero(frente, w)

        elif self.estado in ("BUSCANDO_IZQ", "BUSCANDO_DER"):
            self._t_estado += dt
            self._recorrido += paso
            izquierda = self.estado == "BUSCANDO_IZQ"

            # crucero con abanico reducido Y objetivo acotado a ±35°: seguir
            # el pasillo hacia la señal, sin tomar desvíos por cuenta propia
            ang, frente = self._mejor_hueco(s, 45)
            if ang is not None:
                ang = max(-math.radians(35), min(math.radians(35), ang))
                w = max(-p["w_max"], min(p["w_max"], p["kp_gap"] * ang))
                v = self._v_crucero(frente, w)

            # ¿abertura lateral real, y ya cerca de la señal?
            armado = self._recorrido >= self._dist_senal - p["arm_dist"]
            if izquierda:
                ventana = s[p["side_lo"]:p["side_hi"] + 1]
            else:
                ventana = s[360 - p["side_hi"]:360 - p["side_lo"] + 1]
            hay_abertura = float(np.median(ventana)) > p["opening_depth"]
            self._consec_abertura = self._consec_abertura + 1 if (armado and hay_abertura) else 0

            if self._consec_abertura >= p["opening_consec"]:
                # Registrar la señal obedecida ANTES de girar: tras el giro
                # seguirá visible y sin memoria dispararía otro giro (pirueta).
                if self._senal_xy is not None:
                    self._senales_actuadas.append(
                        (self._senal_xy[0], self._senal_xy[1],
                         "left" if izquierda else "right",
                         self._t_total + p["sign_memory_time"]))
                self.estado = "GIRANDO_IZQ" if izquierda else "GIRANDO_DER"
                # Apuntar al RAYO MÁS PROFUNDO del lado (el eje del corredor
                # destino), no a un +90° ciego: girar 90° exactos desde el
                # yaw actual deja al robot mirando la esquina interior de la
                # abertura, dispara la emergencia y la evasión puede
                # reorientarlo de vuelta por donde vino.
                suave = self._min_movil(s)
                if izquierda:
                    idxs = np.arange(50, 131)
                else:
                    idxs = np.arange(230, 311)
                j = int(idxs[int(np.argmax(suave[idxs]))])
                rel = j if j <= 180 else j - 360
                # acotar a un giro razonable (40°..140° de magnitud)
                rel = max(40.0, min(140.0, abs(rel))) * (1 if izquierda else -1)
                self._yaw_objetivo = yaw + math.radians(rel)
                self._t_estado = 0.0
                self._t_giro = 0.0
            elif self._t_estado > p["search_timeout"]:
                self.estado, self._cooldown = "EXPLORANDO", 1.0

        elif self.estado in ("GIRANDO_IZQ", "GIRANDO_DER"):
            self._t_estado += dt
            self._t_giro += dt
            err = self._ang_diff(self._yaw_objetivo, yaw)
            if abs(err) < math.radians(p["turn_exit_deg"]) or self._t_giro > 6.0:
                self.estado, self._cooldown = "EXPLORANDO", p["cooldown_after_turn"]
            else:
                w = p["kp_turn"] * err
                signo = 1.0 if w >= 0 else -1.0
                w = signo * max(p["w_turn_min"], min(p["w_max"], abs(w)))
                frente = float(np.min(self._abanico(s, 15)))
                v = p["turn_v"] if frente > 0.45 else 0.0

        elif self.estado == "DETENIDO":
            self._t_estado -= dt
            if self._t_estado <= 0:
                self.estado, self._cooldown = "EXPLORANDO", p["cooldown_after_stop"]
                self._stop_ignore = p["stop_ignore_time"]

        # Frenado preventivo: acercarse al umbral de emergencia DESPACIO en
        # vez de entrar en pánico al cruzarlo a velocidad de crucero.
        if v > 0:
            v = min(v, max(0.06, 0.9 * (dist_camino - p["emerg_dist"])))

        # ---------- emergencia (por encima de todo) ----------
        riesgo = dist_camino < p["emerg_dist"] or float(np.min(s)) < p["contact_dist"]

        if riesgo or self.estado == "EVASION_EMERGENCIA":
            if self.estado != "EVASION_EMERGENCIA":
                # Si la emergencia interrumpe un giro señalizado, hay que
                # RETOMARLO al salir: abandonar el giro = saltarse la
                # intersección correcta y perderse en el circuito.
                self._estado_previo = self.estado if self.estado.startswith("GIRANDO") else "EXPLORANDO"
                self._t_estado = 0.0
            self.estado = "EVASION_EMERGENCIA"
            self._t_estado += dt
            if self._estado_previo.startswith("GIRANDO"):
                self._t_giro += dt
                if self._t_giro > 6.0:
                    # el giro interrumpido lleva demasiado: abortarlo
                    self._estado_previo = "EXPLORANDO"
            v = 0.0
            # Rotar hacia el heading con MÁXIMO claro barrido (mismo criterio
            # que el disparo/salida de la emergencia: sin contradicciones).
            K, claros = self._claro_por_heading(s)
            claros = np.minimum(claros, 2.0)   # más de 2 m no aporta
            rel_k = np.where(K <= 180, K, K - 360)
            # leve preferencia por girar poco (desempate estable)
            score = claros - 0.002 * np.abs(rel_k)
            k = int(np.argmax(score))
            ang = math.radians(float(rel_k[k]))
            w = max(-p["w_max"], min(p["w_max"], 2.5 * ang)) if abs(ang) > 1e-6 else 0.0
            if abs(w) < 0.4:
                w = 0.4 if ang >= 0 else -0.4  # no morir a medio girar
            if abs(ang) < math.radians(20) and dist_camino > p["emerg_exit"]:
                self.estado, self._cooldown = self._estado_previo, 0.0
                # OJO: al retomar un giro interrumpido NO se resetea su
                # cronómetro — si cada evasión lo reiniciara, un giro imposible
                # (hacia una pared) ciclaría giro<->evasión para siempre en
                # vez de abortar por timeout.
                w = 0.0

        # ---------- salida ----------
        w = max(-p["w_max"], min(p["w_max"], w))
        v = max(0.0, min(p["v_max"], v))
        self._yaw_int += w * dt
        self._pose_int = (self._pose_int[0] + v * math.cos(yaw) * dt,
                          self._pose_int[1] + v * math.sin(yaw) * dt)
        self._last_cmd = (v, w)
        return v, w, self.estado
