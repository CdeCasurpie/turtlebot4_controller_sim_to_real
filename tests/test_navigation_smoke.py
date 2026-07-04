"""
Smoke test del NavigationController con los parámetros del robot real
(v_max=0.3, lidar_min_valid=0.18) y entradas tipo lista (como TurtleBotReal).
Sin ROS, sin pygame.
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from controller.navigation import NavigationController

def make_ctrl():
    return NavigationController(robot_radius=0.17, lidar_max_range=12.0,
                                v_max=0.3, lidar_min_valid=0.18)

def open_scan():
    return [12.0] * 360  # lista, como el robot real

def wall_ahead_scan(d):
    scan = [12.0] * 360
    for i in list(range(0, 45)) + list(range(315, 360)):
        scan[i] = d
    return scan

ok = 0

# 1. Espacio abierto → EXPLORANDO, v en el tope real 0.3
ctrl = make_ctrl()
v, w = ctrl.step(open_scan(), [], 0.05)
assert ctrl.estado == "EXPLORANDO" and abs(v - 0.3) < 1e-9, (ctrl.estado, v)
ok += 1

# 2. Señal left → BUSCANDO_IZQ; con espacio libre a la izquierda → GIRANDO_IZQ
ctrl = make_ctrl()
det = [{'class': 'left', 'distance': 1.2, 'relative_angle': 0.05}]
ctrl.step(open_scan(), det, 0.05)
assert ctrl.estado in ("BUSCANDO_IZQ", "GIRANDO_IZQ"), ctrl.estado
ok += 1

# 3. GIRANDO dura turn_duration y vuelve a EXPLORANDO con cooldown
ctrl = make_ctrl()
ctrl.step(open_scan(), det, 0.05)
steps = 0
while ctrl.estado.startswith(("BUSCANDO", "GIRANDO")) and steps < 100:
    v, w = ctrl.step(open_scan(), [], 0.05)
    steps += 1
assert ctrl.estado == "EXPLORANDO" and ctrl.cooldown_senal > 0, (ctrl.estado, steps)
ok += 1

# 4. stop a 1.0m → DETENIDO con v=0; a 2.0m no dispara
ctrl = make_ctrl()
v, w = ctrl.step(open_scan(), [{'class': 'stop', 'distance': 1.0, 'relative_angle': 0.0}], 0.05)
assert ctrl.estado == "DETENIDO" and v == 0.0, (ctrl.estado, v)
ctrl2 = make_ctrl()
ctrl2.step(open_scan(), [{'class': 'stop', 'distance': 2.0, 'relative_angle': 0.0}], 0.05)
assert ctrl2.estado == "EXPLORANDO", ctrl2.estado
ok += 1

# 5. DETENIDO expira a los 3s y vuelve a EXPLORANDO con cooldown de 3s
for _ in range(61):  # 61 * 0.05 = 3.05s
    ctrl.step(open_scan(), [], 0.05)
assert ctrl.estado == "EXPLORANDO" and ctrl.cooldown_senal > 2.5, (ctrl.estado, ctrl.cooldown_senal)
ok += 1

# 6. finish a 1.0m → FINALIZADO permanente con v=w=0
ctrl = make_ctrl()
ctrl.step(open_scan(), [{'class': 'finish', 'distance': 1.0, 'relative_angle': 0.0}], 0.05)
assert ctrl.estado == "FINALIZADO", ctrl.estado
v, w = ctrl.step(open_scan(), [], 0.05)
assert v == 0.0 and w == 0.0 and ctrl.estado == "FINALIZADO"
ok += 1

# 7. Pared muy cerca al frente → EVASION_EMERGENCIA con v=0
ctrl = make_ctrl()
v, w = ctrl.step(wall_ahead_scan(0.25), [], 0.05)
assert ctrl.estado == "EVASION_EMERGENCIA" and v == 0.0, (ctrl.estado, v)
ok += 1

# 8. Reflexión del chasis (0.15m) NO entra a lidar_points (filtro 0.18)
ctrl = make_ctrl()
scan = open_scan()
scan[180] = 0.15
pts = ctrl._lidar_points(__import__('numpy').asarray(scan, dtype=float))
assert len(pts) == 0, pts
ok += 1

# 9. snapshot/restore reproduce el estado
ctrl = make_ctrl()
ctrl.step(open_scan(), [{'class': 'stop', 'distance': 1.0, 'relative_angle': 0.0}], 0.05)
snap = ctrl.snapshot()
ctrl.step(open_scan(), [], 0.05)
ctrl.restore(snap)
assert ctrl.snapshot() == snap
ok += 1

print(f"SMOKE OK: {ok}/9 casos pasan (parámetros del robot real, entrada tipo lista)")
