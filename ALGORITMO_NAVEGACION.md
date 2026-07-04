# Lógica del algoritmo de navegación autónoma (`test_autonomous_controller.py`)

Este documento explica en detalle cómo funciona el controlador autónomo que corre en el simulador (`test_autonomous_controller.py`), incluyendo la matemática detrás de cada decisión, cómo se conectan sus piezas, y cómo se compara con lo que ya corre en el robot real (`run_real_autonomous.py`) respecto a la detección de señales con YOLO. La máquina de estados y `buscar_camino_libre` están duplicadas casi verbatim entre ambos archivos, así que casi todo lo descrito en las secciones 1–7 aplica igual al robot real; la sección 8 se enfoca justo en dónde **sí** difieren: la fuente de las detecciones de señales.

---

## 1. Estructura general del bucle

El programa corre a `dt = 1/60` s (60 Hz) dentro de un bucle de pygame. En cada iteración, si no está en pausa, ocurre esta secuencia (líneas 162–307):

1. Se actualiza el cooldown de señales.
2. Se leen los sensores (`lidar_scan`, `vision_dets`) y se precomputan los puntos cartesianos del LiDAR (`lidar_points`).
3. **Paso 1** — la visión (YOLO simulado) puede disparar una transición de estado.
4. **Paso 2** — la lógica propia del estado actual calcula velocidades candidatas `v_target`, `w_target`.
5. **Paso 3** — una capa de anti-choques/evasión de emergencia revisa el LiDAR crudo y, si hay riesgo, **sobrescribe** todo lo anterior.
6. **Paso 4** — se aplica el movimiento físico (`robot.move`) y se guarda un snapshot en el historial para poder pausar/rebobinar.

Es importante notar el orden: la visión solo puede *proponer* un cambio de estado cuando el estado actual es `EXPLORANDO`; la lógica de cada estado calcula velocidades "deseadas"; y la capa de emergencia (paso 3) tiene la última palabra y puede anular cualquier estado, incluyendo forzar la transición a `EVASION_EMERGENCIA`.

---

## 2. Sensores y su representación matemática

### 2.1 `lidar_scan`
Es un arreglo de 360 distancias (metros), un valor por grado. La convención (ver `CLAUDE.md`) es: **índice 0 = justo al frente del robot, el índice crece en sentido antihorario (CCW)**. Un valor igual a `robot.lidar_max_range` significa "no se detectó nada".

### 2.2 `lidar_points` (línea 152–156)
Para cada índice `i` con distancia `dist_p < lidar_max_range`, se convierte la lectura polar a cartesiana **en el marco de referencia local del robot**:

```
x = dist_p * cos(radians(i))
y = dist_p * sin(radians(i))
```

Eje +X = adelante del robot, eje +Y = izquierda del robot (el ángulo crece CCW). Esta lista se recalcula cada frame y es la entrada geométrica de `buscar_camino_libre`.

### 2.3 `dist_frente_estricto` (línea 169)
```python
dist_frente_estricto = min(lidar_scan[0:15] + lidar_scan[345:360])
```
Mínimo de un cono frontal **estricto** de ±15° (30° totales). Se usa exclusivamente para modular la velocidad lineal.

### 2.4 `vision_dets`
Lista de detecciones, cada una con `class` (`left`/`right`/`stop`), `distance` y `relative_angle` (radianes). En el simulador esta lista viene de un detector geométrico simulado; en el robot real viene de correr el modelo YOLO de verdad. Ver la sección 8 para el detalle de esa diferencia.

---

## 3. `buscar_camino_libre`: el corazón geométrico compartido

```python
def buscar_camino_libre(lidar_points, radio_robot, direccion='front', margen_extra=0.10):
```

Responde: **"¿existe un rumbo (ángulo) por el que el robot podría avanzar en línea recta sin chocar con nada que ya ve el LiDAR?"**

### 3.1 Selección del abanico de ángulos a probar
Según `direccion`, se define un conjunto discreto de ángulos candidatos (0°=frente, CCW positivo) y `M` = número de distancias de muestreo a lo largo de cada ángulo:

- `'left'`: `[50, 63, 76, 90, 103, 116, 130]`, `M=5` — abanico de 80° centrado en 90°.
- `'right'`: `[230, 243, 256, 270, 283, 296, 310]`, `M=5` — simétrico, centrado en 270° (-90°).
- `'front'`: `[-20, -13, -6, 0, 6, 13, 20]`, `M=5` — abanico angosto de ±20°.
- otro valor (`'any'`): 12 ángulos de `range(0, 360, 30)`, `M=3` — barrido de 360° para "escapar rápido".

### 3.2 Distancias de muestreo
```python
margen = radio_robot + margen_extra
paso_inicial = 0.3
distancia_paso = (2 * radio_robot) / M
distancias_prueba = [paso_inicial + i * distancia_paso for i in range(M)]
```
Para cada ángulo se prueban `M` puntos equiespaciados, empezando en 0.3 m (para no chocar con el propio cuerpo del robot al evaluar tan cerca). `margen` es el radio de seguridad efectivo: radio físico (0.17) + margen extra, que varía según qué tan arriesgada debe ser la búsqueda (0.10 normal, 0.02 escape de emergencia, 0.15 barrido `'any'`).

### 3.3 Evaluación de colisión (inflado de obstáculos)
```python
for d_c in distancias_prueba:
    cx, cy = d_c * cos(ang_eval), d_c * sin(ang_eval)
    choca = any(hypot(px - cx, py - cy) < margen for (px, py) in lidar_points)
```
Se genera un punto candidato a distancia `d_c` en la dirección `ang_c`, y se comprueba si algún punto del LiDAR cae dentro de un círculo de radio `margen` alrededor de ese punto (equivalente a inflar cada obstáculo por el radio de seguridad — suma de Minkowski). Si cualquiera de los `M` puntos de un ángulo choca, esa ruta se invalida y se corta el chequeo. El **primer** ángulo (en orden de la lista) completamente libre se retorna de inmediato — no se busca el "mejor entre todos", solo el primero según la prioridad ya codificada en el orden de la lista.

### 3.4 Salida
`(hay_espacio, mejor_ang, intentos, distancias_prueba, margen)`. `intentos` y los otros dos solo alimentan el overlay visual (círculos cian=válido, rojo=bloqueado).

---

## 4. La máquina de estados

### `EXPLORANDO` (crucero + evasión suave reactiva)

**Velocidad lineal** (reutilizada en casi todos los estados salvo `DETENIDO`/`EVASION_EMERGENCIA`):
```python
v_target = clamp(0.1, 0.8, (dist_frente_estricto - 0.4) * 0.8)
```
Controlador proporcional: crece linealmente con la distancia libre al frente (ganancia 0.8, punto muerto en 0.4 m), saturado entre 0.1 y 0.8 m/s.

**Evasión suave por campo de repulsión angular**:
```python
min_dist = min(lidar_scan); min_angle = argmin(lidar_scan)  # a rango (-180, 180]
if min_dist < 0.7:
    factor_giro = 1.5 if min_dist < 0.4 else 1.0
    margen = 0.7
    if min_angle >= 0:   target = 90 + (margen - min_dist) * 80.0
    else:                target = -90 - (margen - min_dist) * 80.0
    w_target -= radians(target - min_angle) * factor_giro
```
Toma el punto de obstáculo más cercano de **todo** el LiDAR (no solo al frente) y calcula un ángulo objetivo que busca llevarlo a ±90° (el robot termina tangente al obstáculo). Cuanto más cerca del margen deseado (0.7 m), más se exagera el objetivo, empujando el giro con más fuerza. Es un controlador proporcional con error = `target - min_angle`, amplificado por `factor_giro` si está muy cerca (<0.4 m).

### `BUSCANDO_IZQ` / `BUSCANDO_DER` (se detectó una señal, se busca espacio para girar)
- `v_target`: misma fórmula proporcional.
- Centrado visual: `w_target = relative_angle * 2.5` (control proporcional para mantener la señal centrada en el FOV).
- Búsqueda de hueco lateral vía `buscar_camino_libre(..., 'left'|'right', 0.10)`. Si hay espacio → transiciona a `GIRANDO_IZQ`/`GIRANDO_DER`.

### `GIRANDO_IZQ` / `GIRANDO_DER` (giro de ~80°, en lazo abierto)
```python
w_target = 2.0 if GIRANDO_IZQ else -2.0   # rad/s constante
tiempo_estado += dt
if tiempo_estado >= 0.7: estado_actual = "EXPLORANDO"; cooldown_senal = 0.2
```
Sin realimentación angular: `w · t = 2.0 × 0.7 = 1.4 rad ≈ 80°`. Giro por temporización (lazo abierto), no depende de odometría durante el giro.

### `DETENIDO` (parado ante señal `stop`)
Cuenta regresiva desde 3.0 s. Al salir, `cooldown_senal = 3.0` s (más largo que el de los giros, 0.2 s) porque la señal probablemente sigue visible justo después de reanudar.

### Transición inicial por visión (Paso 1)
Solo se evalúa desde `EXPLORANDO` y con `cooldown_senal <= 0`:
```python
if clase == 'left':  -> BUSCANDO_IZQ
elif clase == 'right': -> BUSCANDO_DER
elif clase == 'stop' and dist <= 1.6: -> DETENIDO, tiempo_estado = 3.0
```
`stop` exige además estar a ≤1.6 m; `left`/`right` disparan en cuanto se detectan (ese estado solo empieza a *buscar* espacio, no actúa de inmediato).

---

## 5. Capa de anti-choques / evasión de emergencia (prioridad máxima)

Corre **después** de la lógica de estados y puede sobrescribir cualquier decisión, incluyendo forzar `EVASION_EMERGENCIA`.

```python
min_dist_frontal = min(lidar_scan[i] for i in range(0,45)+range(315,360))  # cono ±45°
if min_dist_frontal < 0.32 and v_target > 0.05: riesgo_inminente = True
if min(lidar_scan) < 0.19: riesgo_inminente = True   # 0.17 radio + 0.02 ruido
```
Dos disparadores: (1) cono frontal ancho por debajo de 0.32 m **y** el robot queriendo avanzar; (2) umbral absoluto de 0.19 m en cualquier dirección, sin condición de velocidad — roce físico inminente.

En `EVASION_EMERGENCIA`: se detiene la traslación (`v_target=0`), se busca un hueco al frente con el margen más agresivo de todos (`margen_extra=0.02`). Si hay escape: `w_target = ángulo_relativo_rad × 4.0` (ganancia más alta del sistema); si ya está alineado (<15°) y el frente está despejado (>0.4 m), sale de inmediato a `EXPLORANDO`. Si no hay ningún escape frontal: `w_target = 3.0` — gira sobre su eje barriendo hasta que aparezca un hueco.

---

## 6. Actuación física y sistema de historial

```python
hubo_choque = robot.move(v_target, w_target, dt)
```
Se envían las velocidades finales al mock (tracción diferencial + retraso de 3 frames + ruido gaussiano + resolución de colisiones contra paredes). Cada 0.5 s se guarda un snapshot completo (pose + variables de estado) para el sistema de pausa/rebobinado (`Espacio`, `←`/`→`), que restaura tanto la pose física (vía atributos con *name mangling*, `robot._TurtleBotMock__x`, etc.) como el estado de la máquina (`nonlocal`).

---

## 7. Renderizado (breve, no afecta la lógica)

Dos vistas (`L` alterna): global (marco del mundo) y local/robot (egocéntrica). Se dibujan los rayos LiDAR coloreados por zona de riesgo, el cono del FOV de cámara, y los círculos cian/rojo de `intentos_render` — la ventana directa a qué ángulos evaluó `buscar_camino_libre` y por qué se tomó cada decisión de giro.

---

## 8. Detección de señales: simulador (falsa) vs. robot real (YOLO real)

Esta es la parte que preguntaste directamente: **¿ya corre el modelo YOLO real alimentando este mismo algoritmo? Sí, en el robot real ya está implementado end-to-end.** Aquí el detalle de qué existe y qué falta.

### 8.1 En el simulador — no hay modelo, es geometría simulada
`TurtleBotMock.get_vision_detections()` (en `Simulator/TurtleBotSim/turtlebot.py`) **no corre ningún modelo**. Simplemente recorre `world.signals` (las señales que dibujaste a mano en el editor de mapas) y para cada una comprueba geométricamente si cae dentro del cono del FOV de la cámara, dentro de `[camera_min_range, camera_max_range]`, y si no está ocluida (usando el mismo raycaster que el LiDAR). Es un "YOLO perfecto": nunca falla, nunca da falsos positivos, no tiene noción de confianza. Sirve para probar la máquina de estados de forma aislada, no para validar el modelo de visión en sí.

### 8.2 En el robot real — YOLO corriendo dentro de la VPU de la OAK-D, no en la CPU
`TurtleBotReal.get_vision_detections()` en `TurtleBotController/turtlebot.py` ya no corre nada en la CPU
de la Raspberry Pi ni depende de un tópico ROS de imagen: delega en
`TurtleBotController/vpu_vision.py: VpuYoloDetector`, que:

1. Construye un pipeline DepthAI (`ColorCamera` → `YoloDetectionNetwork` → `XLinkOut "nn"`) que corre
   **enteramente dentro del chip Myriad X de la OAK-D Lite** — la Pi nunca recibe ni decodifica frames,
   solo resultados ya resueltos.
2. El modelo cargado es el `.blob` de `yolonanov2` (4 clases `left/right/stop/finish`), configurado vía
   `config.json → vision.vpu_blob_path`/`classes_path`/`num_classes`/`iou_threshold`/`fps`, con
   `confidence_threshold = 0.85` aplicado **dentro de la VPU** (`detection_nn.setConfidenceThreshold`).
3. Un hilo de fondo lee la cola `nn` con `.get()` bloqueante (cero busy-wait en la Pi) y convierte cada
   caja detectada a `relative_angle`/`distance` con la misma matemática que antes (proyección al FOV de
   la cámara y aproximación pinhole asumiendo ~20 cm de ancho real de señal).
4. Expone `get_detections()` en la misma estructura `[{'class', 'distance', 'relative_angle'}, ...]` que
   espera la máquina de estados — `get_vision_detections()` solo reenvía esa lista.

Y `run_real_autonomous.py` es una copia casi exacta de `test_autonomous_controller.py` (mismas 5 fases, mismo `buscar_camino_libre`, mismos umbrales) que consume exactamente este `get_vision_detections()` real. Es decir: **correr `python run_real_autonomous.py` en el robot ya te da LiDAR + YOLO real (corriendo en la VPU) alimentando el mismo algoritmo que ves en el simulador** — no es una tarea pendiente, ya está cableado (ver `DEPLOY_ROBOT_VPU.md` para el paso a paso). Las únicas diferencias deliberadas frente al simulador son de seguridad de hardware: `v_target` tope 0.3 m/s en vez de 0.8 (línea 121 de `run_real_autonomous.py`), y se descartan lecturas LiDAR `< 0.18 m` por ser reflexiones del propio chasis del robot (línea 87).

**Requisito de versión crítico:** esto necesita `depthai` de la rama **2.x** (`depthai<3`). La 3.x
reestructuró la API por completo (sin `YoloDetectionNetwork`/`XLinkIn`/`XLinkOut`, `Pipeline` atado a un
`Device`) y rompe tanto `vpu_vision.py` como el sandbox `vpu_deployment/test_depthai_yolo.py`. Ya está
fijado en `vpu_deployment/requirements.txt`; si instalas `depthai` a mano en el entorno del robot,
usa `pip install "depthai<3"`, no `pip install depthai` a secas.

### 8.3 `finish` → nuevo estado terminal `FINALIZADO`

Ambos archivos ya manejan la 4ª clase del modelo v2:

```python
elif clase == 'finish' and dist <= 1.6:
    estado_actual = "FINALIZADO"
...
elif estado_actual == "FINALIZADO":
    # Meta alcanzada: se queda detenido, sin volver a EXPLORANDO.
    v_target = 0.0
    w_target = 0.0
```

`FINALIZADO` es terminal: a diferencia de `DETENIDO` (cuenta regresiva de 3 s y vuelve a `EXPLORANDO`), aquí no hay temporizador ni retorno — el robot se queda parado indefinidamente una vez que ve la señal `finish` a ≤1.6 m. Igual que con `DETENIDO`, la capa de anti-choques (sección 5) corre después y puede seguir sobreescribiendo hacia `EVASION_EMERGENCIA` si algo se acerca demasiado incluso ya "finalizado" — la seguridad física tiene prioridad sobre el estado terminal.

En `run_real_autonomous.py` hay además una diferencia de lifecycle: justo después de `robot.move(...)`, si `estado_actual == "FINALIZADO"` se imprime un mensaje y se hace `break` para salir del bucle principal (el bloque `finally` llama `robot.stop()` de todos modos) — el script termina solo al llegar a la meta. En el simulador **no** se hace `break`: el estado se queda congelado en `FINALIZADO` pero el bucle de pygame sigue corriendo, para poder seguir usando pausa/rebobinado y las vistas de depuración.

**Limitación conocida:** el editor de mapas (`test_manual_simulation.py`) solo tiene atajos de teclado para colocar señales `left`/`right`/`stop` (teclas `1`/`2`/`3`); no hay un atajo para `finish` todavía, así que hoy no se puede probar la transición a `FINALIZADO` dentro del simulador dibujando un mapa — solo llegará a producirse en el robot real, donde la detección viene del modelo v2 y no de `world.signals`.

### 8.4 Lo que sigue pendiente

- **Nunca se ha corrido contra hardware real.** El código (`vpu_vision.py`) se validó por construcción del pipeline (con `depthai<3` instalado localmente, sin OAK-D conectada), pero falta la corrida real en el robot con la cámara conectada — ver `DEPLOY_ROBOT_VPU.md`.
- **Dependencia silenciosa si la VPU no arranca.** Si la OAK-D no se detecta o el `.blob`/`classes.txt` no coinciden, `self.vpu_detector` queda `None` (solo se imprime `[VISION-VPU] Error al iniciar la VPU: ...` una vez al arrancar) y `get_vision_detections()` retorna `[]` en cada frame sin volver a avisar — el LiDAR seguiría funcionando normal, pero el robot nunca reaccionaría a señales.
- **El editor de mapas no tiene atajo para `finish`** (ver 8.3) — si se quiere probar `FINALIZADO` en el simulador, habría que añadir una tecla (`4`) en `test_manual_simulation.py` que haga `signal_mode = 'finish'`.

---

## 9. Resumen en un párrafo

El controlador es una máquina de estados reactiva (`EXPLORANDO`, `BUSCANDO_IZQ/DER`, `GIRANDO_IZQ/DER`, `DETENIDO`, `FINALIZADO`, `EVASION_EMERGENCIA`) que en cada frame combina tres fuentes de decisión con distinta prioridad —visión (inicia maniobras solo desde `EXPLORANDO`, con cooldown anti-rebote), lógica proporcional por estado (velocidad lineal según distancia frontal, velocidad angular vía campo de repulsión del punto de obstáculo más cercano o giro de lazo abierto calibrado por tiempo, y una búsqueda geométrica compartida `buscar_camino_libre` que infla los obstáculos del LiDAR por un radio de seguridad para hallar el primer rumbo libre), y una capa de anti-choques de máxima prioridad que fuerza detención y evasión agresiva ante proximidad peligrosa—; esta misma lógica, matemática y umbrales están duplicados casi exactamente en `run_real_autonomous.py`, con la única diferencia relevante en la fuente de las detecciones de señales: el simulador las simula geométricamente contra `world.signals` sin ningún modelo real, mientras que el robot real (`TurtleBotReal.get_vision_detections()`, vía `TurtleBotController/vpu_vision.py: VpuYoloDetector`) **ejecuta el modelo v2 de 4 clases `left/right/stop/finish` corriendo enteramente dentro de la VPU Myriad X de la OAK-D** (no en la CPU de la Pi, no vía ROS), usando `depthai` de la rama 2.x; la señal `finish` dispara el estado terminal `FINALIZADO` (detención permanente, con salida limpia del programa en el robot real vía `break`, y congelado sin salir del bucle en el simulador para poder seguir depurando) — lo que queda pendiente es la corrida real en el robot con la OAK-D físicamente conectada (ver `DEPLOY_ROBOT_VPU.md`), y añadir un atajo de teclado para `finish` en el editor de mapas si se quiere probar ese estado también en el simulador.
