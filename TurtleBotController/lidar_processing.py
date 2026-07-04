"""
Procesamiento puro del LaserScan real (sin rclpy) — testeable en cualquier máquina.

Convención de salida (la misma que asume todo el código del proyecto):
array de `resolution` distancias, índice 0 = frente del robot, antihorario.

Tres responsabilidades (T2 del ROADMAP):
1. Máscara de validez: todo rayo NaN / inf / < min_valid / >= max_range se
   reemplaza por max_range ANTES de que cualquier consumidor haga min().
   "Sin información" se trata como "sin obstáculo detectado en ese rayo",
   igual que hace el mock del simulador cuando un raycast no golpea nada.
2. Remuestreo por ángulo usando los metadatos del mensaje (angle_min /
   angle_increment), en vez de remuestrear por posición del arreglo y rotar
   90 índices fijos. Funciona con cualquier número de rayos, angle_min != 0,
   escáneres horarios (increment < 0) y barridos parciales.
3. Watchdog de datos viejos (scan_is_stale), consumido por el loop real.

El único dato que NO viene en el mensaje es el montaje físico del sensor:
su ángulo cero apunta a la DERECHA del robot, así que el frente del robot
está en el ángulo láser +90° (front_offset_rad = pi/2, configurable en
config.json como lidar_front_angle_deg).
"""
import math
import numpy as np

_TWO_PI = 2.0 * math.pi


def process_scan(ranges, angle_min, angle_increment, resolution=360,
                 max_range=12.0, min_valid=0.18,
                 front_offset_rad=math.pi / 2.0):
    """
    Convierte los rangos crudos de un LaserScan al formato canónico del
    proyecto. Devuelve una lista de `resolution` floats, índice 0 = frente,
    antihorario, sin ningún valor fuera de [min_valid, max_range].
    """
    ranges = np.asarray(ranges, dtype=float)
    n = len(ranges)
    out = np.full(resolution, max_range, dtype=float)
    if n == 0 or angle_increment == 0.0:
        return out.tolist()

    # 1. Máscara de validez sobre los rangos crudos
    valid = np.isfinite(ranges) & (ranges >= min_valid) & (ranges < max_range)
    clean = np.where(valid, ranges, max_range)

    # 2. Remuestreo por ángulo: para cada rumbo de salida (marco del robot),
    #    buscar el rayo crudo cuyo ángulo láser le corresponde.
    bearings = np.arange(resolution) * (_TWO_PI / resolution)   # 0=frente, CCW
    laser_angles = bearings + front_offset_rad
    idx = np.rint((laser_angles - angle_min) / angle_increment).astype(int)

    coverage = n * abs(angle_increment)
    if coverage >= _TWO_PI * 0.99:
        # Barrido de círculo completo: los rumbos fuera de [0, n) dan la vuelta
        out = clean[idx % n]
    else:
        # Barrido parcial: los rumbos no cubiertos quedan en max_range
        in_range = (idx >= 0) & (idx < n)
        out[in_range] = clean[idx[in_range]]

    return out.tolist()


def scan_is_stale(last_scan_time, now, max_age=0.3):
    """
    True si el último scan es demasiado viejo para manejar con él (o si
    nunca llegó ninguno). `last_scan_time` y `now` en el mismo reloj
    monotónico; `last_scan_time=None` significa "sin datos aún".
    """
    if last_scan_time is None:
        return True
    return (now - last_scan_time) > max_age
