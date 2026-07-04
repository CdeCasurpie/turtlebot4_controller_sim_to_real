"""
Tests de T2 — higiene del LiDAR real (TurtleBotController/lidar_processing.py).
Mensajes LaserScan sintéticos, sin ROS, sin robot, sin pygame.

Verificación exigida por el ROADMAP:
- un rayo 0.0 NO dispara EVASION_EMERGENCIA;
- scan viejo → el watchdog manda detener;
- obstáculo sintético en el ángulo láser +90° (el frente físico) termina en el índice 0.
"""
import math
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from TurtleBotController.lidar_processing import process_scan, scan_is_stale
from controller.navigation import NavigationController

MAX_RANGE = 12.0
PI = math.pi

def raw_scan(n=360, angle_min=0.0, value=MAX_RANGE - 0.5):
    """Escaneo crudo uniforme (todo lejos pero válido)."""
    inc = 2 * PI / n
    return [value] * n, angle_min, inc

ok = 0

# ---------------------------------------------------------------- validez ---

# 1. Un rayo 0.0 (inválido típico del sensor) se reemplaza por max_range
ranges, amin, inc = raw_scan()
ranges[95] = 0.0
out = process_scan(ranges, amin, inc, max_range=MAX_RANGE)
assert all(v >= 0.18 for v in out), min(out)
assert MAX_RANGE in out
ok += 1

# 2. NaN e inf también se sanean a max_range
ranges, amin, inc = raw_scan()
ranges[10] = float('nan'); ranges[20] = float('inf'); ranges[30] = float('-inf')
out = process_scan(ranges, amin, inc, max_range=MAX_RANGE)
assert all(np.isfinite(out)) and all(0.18 <= v <= MAX_RANGE for v in out)
ok += 1

# 3. Reflexión del chasis (0.15 < min_valid=0.18) se sanea
ranges, amin, inc = raw_scan()
ranges[200] = 0.15
out = process_scan(ranges, amin, inc, max_range=MAX_RANGE, min_valid=0.18)
assert all(v >= 0.18 for v in out), min(out)
ok += 1

# 4. INTEGRACIÓN: un 0.0 en pleno cono frontal NO dispara emergencia en el
#    controlador (antes de T2, min() sobre el scan crudo lo hacía permanente)
ranges, amin, inc = raw_scan()
ranges[90] = 0.0  # ángulo láser 90° = justo el frente del robot
scan = process_scan(ranges, amin, inc, max_range=MAX_RANGE)
ctrl = NavigationController(robot_radius=0.17, lidar_max_range=MAX_RANGE,
                            v_max=0.3, lidar_min_valid=0.18)
v, w = ctrl.step(scan, [], 0.05)
assert ctrl.estado == "EXPLORANDO" and v > 0, (ctrl.estado, v)
ok += 1

# 5. Un obstáculo REAL al frente sí sobrevive al saneo y sí frena
ranges, amin, inc = raw_scan()
for j in range(60, 121):   # bloque alrededor del ángulo láser 90° (frente)
    ranges[j] = 0.25
scan = process_scan(ranges, amin, inc, max_range=MAX_RANGE)
ctrl = NavigationController(robot_radius=0.17, lidar_max_range=MAX_RANGE,
                            v_max=0.3, lidar_min_valid=0.18)
v, w = ctrl.step(scan, [], 0.05)
assert ctrl.estado == "EVASION_EMERGENCIA" and v == 0.0, (ctrl.estado, v)
ok += 1

# ------------------------------------------------- rotación por metadatos ---

# 6. Obstáculo en el ángulo láser angle_min + 90° → índice 0 (angle_min = 0)
ranges, amin, inc = raw_scan()
ranges[90] = 1.0
out = process_scan(ranges, amin, inc, max_range=MAX_RANGE)
assert abs(out[0] - 1.0) < 1e-9, out[0]
ok += 1

# 7. Mismo invariante con angle_min = -pi (el 90 fijo de antes fallaría):
#    el frente físico sigue siendo el ángulo láser ABSOLUTO +90°
n = 360
amin = -PI
inc = 2 * PI / n
ranges = [MAX_RANGE - 0.5] * n
j_front = round((PI / 2 - amin) / inc) % n   # rayo que apunta a láser +90°
ranges[j_front] = 1.0
out = process_scan(ranges, amin, inc, max_range=MAX_RANGE)
assert abs(out[0] - 1.0) < 1e-9, (j_front, out[0])
ok += 1

# 8. Resolución cruda distinta de 360 (RPLIDAR-like, 1147 rayos)
n = 1147
amin = 0.0
inc = 2 * PI / n
ranges = [MAX_RANGE - 0.5] * n
j_front = round((PI / 2) / inc)
ranges[j_front] = 0.8
out = process_scan(ranges, amin, inc, max_range=MAX_RANGE)
assert abs(out[0] - 0.8) < 1e-9, out[0]
assert len(out) == 360
ok += 1

# 9. Sentido antihorario preservado: obstáculo a la IZQUIERDA del robot
#    (láser 180° cuando angle_min=0) → índice ~90 de la salida
ranges, amin, inc = raw_scan()
ranges[180] = 2.0
out = process_scan(ranges, amin, inc, max_range=MAX_RANGE)
assert abs(out[90] - 2.0) < 1e-9, out[90]
ok += 1

# 10. Escáner horario (angle_increment negativo) también mapea bien
n = 360
inc = -2 * PI / n
ranges = [MAX_RANGE - 0.5] * n
# rayo i apunta a -i grados; láser +90° ≡ -270° → i = 270
ranges[270] = 1.5
out = process_scan(ranges, 0.0, inc, max_range=MAX_RANGE)
assert abs(out[0] - 1.5) < 1e-9, out[0]
ok += 1

# 11. Barrido parcial (180°): los rumbos no cubiertos quedan en max_range
n = 180
inc = PI / n           # cubre solo [0°, 180°) del marco láser
ranges = [1.0] * n
out = process_scan(ranges, 0.0, inc, max_range=MAX_RANGE)
covered = sum(1 for v in out if v < MAX_RANGE)
assert 0 < covered < 360, covered
ok += 1

# 12. Sin rayos / increment 0 → todo max_range (sin crash)
assert process_scan([], 0.0, 0.0175) == [MAX_RANGE] * 360
assert process_scan([1.0] * 10, 0.0, 0.0) == [MAX_RANGE] * 360
ok += 1

# -------------------------------------------------------------- watchdog ---

# 13. Sin scan aún → viejo; recién llegado → fresco; >0.3s → viejo
assert scan_is_stale(None, 100.0) is True
assert scan_is_stale(100.0, 100.1, max_age=0.3) is False
assert scan_is_stale(100.0, 100.5, max_age=0.3) is True
ok += 1

print(f"LIDAR HYGIENE OK: {ok}/13 casos pasan (sin ROS)")
