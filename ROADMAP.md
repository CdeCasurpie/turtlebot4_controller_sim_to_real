# ROADMAP — Plan estratégico de mejora

Cada tarea está diseñada para ejecutarse en **una sesión de agente con contexto fresco**.
Protocolo: abrir sesión nueva → "Lee ROADMAP.md y ejecuta la tarea T<n>" → al terminar,
marcar `[x]`, anotar hallazgos en la sección de la tarea, y commitear.
**No mezclar tareas en una misma sesión.** El orden importa: T1 desbloquea casi todo lo demás.

Regla de oro: ninguna tarea se marca como hecha sin su **verificación** ejecutada y pasando.

---

## FASE 0 — Fundación (prerrequisito de todo)

### [x] T1 — Extraer el controlador a un módulo compartido — HECHO 2026-07-04
**Problema:** la máquina de estados + `buscar_camino_libre` están duplicadas verbatim en
`test_autonomous_controller.py` y `run_real_autonomous.py`. Cada ajuste exige un port manual.
**Qué hacer:**
- Crear `controller/navigation.py` con una clase `NavigationController`:
  - `step(lidar_scan, vision_dets, dt) -> (v, w)` — pura, sin I/O, sin pygame, sin ROS.
  - Todo el estado interno (estado actual, timers, cooldowns) vive en la clase.
  - Todos los números mágicos (0.7, 0.32, 0.19, 0.7s de giro, ganancias, umbrales) como
    parámetros del constructor con defaults = valores actuales, para poder optimizarlos después (T8).
- `run_real_autonomous.py` y `test_autonomous_controller.py` quedan como shells finos:
  instancian robot + controlador, hacen el loop, renderizan/loguean.
- NO cambiar comportamiento en esta tarea: refactor puro, misma lógica.
**Verificación:** correr `test_autonomous_controller.py` con semilla fija antes y después del
refactor (fijar `random.seed`/`np.random.seed`) y comparar trayectorias — deben ser idénticas.
**Archivos:** nuevo `controller/navigation.py`; editar los dos scripts.

**RESULTADO (2026-07-04):** Hecho y verificado. `controller/navigation.py` creado; ambos
scripts son ahora shells finos. Verificación ejecutada con harness headless (3600 pasos,
semilla 42, mapa `world_map.json`): (a) determinismo confirmado (2 corridas idénticas);
(b) el controlador nuevo con una subclase que reproduce el bug histórico del cono frontal
es IDÉNTICO BIT A BIT a la lógica original — refactor fiel; (c) smoke test
`tests/test_navigation_smoke.py` 9/9 con parámetros del robot real.
**Hallazgos:** las dos copias NO eran idénticas — 3 diferencias:
1. `v_max` 0.8 (sim) vs 0.3 (real) → ahora parámetro.
2. Filtro de reflexiones <0.18m solo en el real → ahora parámetro `lidar_min_valid`.
3. **BUG en el sim**: `min(lidar_scan[0:15] + lidar_scan[345:360])` con ndarray SUMA
   elemento a elemento (en el real, con lista, concatena). El sim calculaba la distancia
   frontal como ~2× la real. Corregido en el módulo compartido (semántica del real).
   Impacto medido: velocidad media del sim 0.713 → 0.496 m/s (−30%); el comportamiento
   del sim ANTES de T1 era sistemáticamente más optimista que el real. Cualquier tuning
   histórico hecho en el sim heredó ese sesgo — refuerza la necesidad de T8.
**Nota de entorno:** el Python local no tiene pygame (los scripts de pygame corren en otro
entorno); la verificación del sim se hizo con harness headless sin pygame + py_compile.

---

## FASE 1 — Bugs latentes (cada uno puede perder una corrida por sí solo)

### [ ] T2 — Higiene del LiDAR real: máscara de validez, watchdog, rotación calibrada
**Problema (3 bugs):**
1. `dist_frente_estricto`, `min_dist_frontal` y `min(lidar_scan)` consumen el scan crudo;
   un rayo inválido `0.0` o una reflexión del chasis (<0.18m) dispara `EVASION_EMERGENCIA`
   permanente. El filtro <0.18m solo se aplica a `lidar_points`.
2. Si `/scan` deja de llegar, `latest_scan` se congela y el robot maneja sobre datos viejos.
3. La rotación de 90 índices en `TurtleBotController/turtlebot.py` está hardcodeada; debe
   derivarse de `msg.angle_min` / `msg.angle_increment`.
**Qué hacer:**
- En `get_lidar_scan()` (o en el controlador): reemplazar lecturas fuera de
  `[0.18, lidar_max_range)` por `lidar_max_range` ANTES de cualquier `min()`.
- Guardar el timestamp del último scan; si es más viejo que 0.3s, `get_lidar_scan()` señala
  staleness y el controlador/loop detiene el robot.
- Calcular la rotación del arreglo desde los metadatos del mensaje, no con el 90 fijo.
**Verificación:** tests unitarios con mensajes `LaserScan` sintéticos (sin robot): scan con un
0.0 no dispara emergencia; scan viejo → robot se detiene; obstáculo sintético en
`angle_min + 90°` termina en el índice 0.

### [ ] T3 — Calibración de visión + auditoría del entrenamiento (fliplr)
**Problema (2 bugs):**
1. `vpu_vision.py` usa `focal_length = 640` px pero el config asume FOV 60° (focal correcta
   ≈554 px) → todas las distancias infladas ~15%, desplazando los umbrales de stop/finish (1.6m).
   Además el preview 640×640 es un recorte del sensor: el FOV efectivo real es desconocido.
2. Si el modelo v2 se entrenó con `fliplr=0.5` (default de ultralytics), las señales left/right
   se voltearon en espejo SIN intercambiar la etiqueta → confusión left↔right latente.
**Qué hacer:**
- Revisar los args de entrenamiento de `yolonanov2` (buscar `args.yaml` o el script de
  entrenamiento). Si `fliplr != 0.0` → reentrenar con `fliplr=0.0` y regenerar el .blob
  (usar `vpu_deployment/MODEL_CONVERSION.md` como guía). Si no hay registro, asumir lo peor.
- Calibración empírica de la focal: con la OAK-D, colocar una señal impresa a distancias
  medidas (0.5 / 1.0 / 2.0 m), registrar el ancho del bbox, despejar
  `focal = bbox_px * dist / ancho_real` y promediar. Escribir el valor en `config.json`
  (nuevo campo `focal_length_px`) y usarlo en `vpu_vision.py`.
- De paso: mapeo de ángulo correcto `atan(normalized_x * tan(FOV/2))` en vez de lineal.
- Nota: requiere hardware (OAK-D + señal impresa). La parte de auditoría fliplr no.
**Verificación:** señal a 1.00m medido → distancia reportada 1.00 ± 0.05m. Para fliplr:
inferencia sobre imágenes de left y right espejadas manualmente — la clase NO debe voltearse
con confianza alta.

### [ ] T4 — Filtro temporal k-de-n sobre las detecciones
**Problema:** un solo frame ≥0.85 dispara transiciones; un falso positivo de `finish`
termina la carrera permanentemente.
**Qué hacer:**
- En `controller/navigation.py` (requiere T1): buffer de las últimas n detecciones por clase.
  Transición solo si k-de-n frames consecutivos coinciden en clase (empezar con 3-de-5 para
  left/right/stop, 5-de-7 para finish). Distancia = mediana de las distancias del buffer.
- Bajar `confidence_threshold` en `config.json` a ~0.5 (el filtro temporal reemplaza al
  umbral alto como control de falsos positivos, y gana recall a distancia).
- Aplicar el mismo filtro en sim y real (gratis si T1 está hecho).
**Verificación:** test unitario: secuencia con 1 frame espurio de `finish` entre frames vacíos
→ no hay transición; 5 frames consecutivos → sí. En el simulador (tras T6, el mock tendrá
falsos positivos): 0 transiciones espurias en 100 episodios.

---

## FASE 2 — Precisión sim-to-real

### [ ] T5 — Giros a lazo cerrado con /odom
**Problema:** el giro de 80° es a lazo abierto (ω=2.0 × 0.7s) y además el loop real corre más
lento que dt nominal (el sleep no descuenta el tiempo de cómputo) → ángulo real mayor y variable.
**Qué hacer:**
- Suscribirse a `/odom` en `_TurtleBotRosNode`; exponer `get_heading()` en `TurtleBotReal`
  y en `TurtleBotMock` (el mock ya tiene θ — devolverlo con ruido leve).
- `GIRANDO_*`: rotar hasta que `|heading - heading_inicial| >= 80°` (con wrap-around),
  con timeout de seguridad (~2s) como fallback.
- Arreglar el timing del loop real: medir tiempo transcurrido real y dormir el remanente
  (`sleep(max(0, dt - elapsed))`), en vez del sleep fijo dentro de `move()`.
**Verificación:** en sim, el ángulo final del giro debe quedar en 80° ± 5° bajo ruido de
actuadores ×3. En real: marcar orientación en el piso, comandar 10 giros, medir dispersión.

### [ ] T6 — Simulador pesimista + aleatorización de dominio
**Problema:** el mock de visión es perfecto (clase exacta, distancia exacta, determinista) y
el ruido físico es gaussiano i.i.d. con parámetros fijos → el controlador está sobreajustado
a un robot imaginario. El sim ni siquiera reproduce las reflexiones <0.18m que el filtro
real existe para eliminar.
**Qué hacer, en `TurtleBotMock`:**
- Visión: probabilidad de detección decreciente con distancia/ángulo; ruido multiplicativo
  de distancia (±25%); matriz de confusión de clases (incluir left↔right ~2%); tasa de
  falsos positivos (~0.5% de frames); flicker (dropout de frames).
- LiDAR: inyectar rayos inválidos (0.0) y reflexiones espurias <0.18m aleatorias;
  offset angular de montaje por episodio (±3°).
- Física: por episodio, sortear: frames de delay (2–6), escalas de ruido (×0.5–×3),
  radio (±1cm), tope de velocidad (±10%). Sesgo constante por episodio además del ruido i.i.d.
- Todo detrás de un `DomainRandomizationConfig` con semilla, para reproducibilidad.
**Verificación:** el controlador actual (post T1–T5) debe seguir completando el mapa actual
en ≥90% de 100 episodios aleatorizados. Si no llega — eso es información, no fracaso: anotar
los modos de falla para T8.

---

## FASE 3 — La máquina de evaluación (la ventaja competitiva real)

### [ ] T7 — Harness Monte Carlo headless + biblioteca de mapas adversariales
**Problema:** hoy "probar" = mirar pygame, n=1. No hay métrica, no hay regresión.
**Qué hacer:**
- `evaluate.py`: corre N episodios sin pygame, más rápido que tiempo real
  (controlador de T1 + mock de T6), multiproceso. Reporta por episodio: éxito/fracaso,
  tiempo hasta finish, # colisiones, # giros equivocados, # transiciones espurias.
  Salida: JSON + resumen en consola. Semillas reproducibles.
- Biblioteca `maps/` de mapas adversariales (usar el editor o escribir el JSON a mano):
  hueco angosto (diámetro+5cm), callejón sin salida, trampa en U, señal a 45°, dos señales
  visibles a la vez, señal justo tras una esquina, recta larga, campo de obstáculos pequeños,
  y el mapa actual `world_map.json` como caso base.
- Opcional: generador procedural de mapas tipo circuito con semilla.
**Verificación:** `python evaluate.py --episodes 100` corre sin display y produce el reporte.
Guardar el reporte baseline en `reports/` — es la línea base contra la que se mide todo lo demás.

### [ ] T8 — Optimización automática de parámetros
**Problema:** ~15 números mágicos ajustados a ojo mirando un solo mapa.
**Qué hacer:**
- `optimize.py` con Optuna (o CMA-ES): busca sobre los parámetros expuestos en T1
  (rangos de repulsión, umbrales de emergencia, ganancias, k-de-n, velocidades, márgenes).
- Objetivo lexicográfico: maximizar tasa de éxito sobre TODOS los mapas de T7 con
  aleatorización de T6; desempatar por tiempo medio. ~50 episodios por trial.
- Guardar el mejor set en `controller/tuned_params.json`; el controlador lo carga si existe.
**Verificación:** el set optimizado supera al baseline de T7 en tasa de éxito Y tiempo,
sobre semillas NO usadas durante la optimización (holdout).

---

## FASE 4 — Rendimiento (ganar por tiempo)

### [ ] T9 — Mejoras de velocidad y navegación
Solo después de que T7/T8 den una base medible. Candidatos, cada uno evaluado contra el harness:
- **Follow-the-Gap** para crucero, reemplazando la repulsión proporcional (elimina
  oscilación y mínimos locales; ~30 líneas).
- **Timeouts en todos los estados** (BUSCANDO_* hoy puede quedarse para siempre).
- **Subir el tope de velocidad**: el Create 3 llega a 0.46 m/s; velocidad escalada por
  espacio libre en vez del cap fijo de 0.3.
- **Distancia por estéreo**: migrar `vpu_vision.py` a `YoloSpatialDetectionNetwork`
  (la OAK-D tiene estéreo; da (x,y,z) por detección en el dispositivo, invariante al ángulo).
- **Chequeo de corredor exacto** en `buscar_camino_libre` (distancia perpendicular
  punto-rayo) en vez de muestreo discreto.
**Verificación:** cada cambio se acepta solo si NO baja la tasa de éxito en el harness y
mejora el tiempo medio. Un cambio a la vez, con reporte guardado en `reports/`.

---

## FASE 5 — Validación en el mundo real

### [ ] T10 — Regresión con datos reales
- Grabar cada corrida real con `rosbag record /scan /odom /cmd_vel` + log de detecciones VPU.
- Script de replay: alimentar scans/detecciones grabados al controlador offline y comparar
  comandos → test de regresión con datos reales.
- Recolectar frames reales de la cámara en el circuito para un set de validación del modelo
  que no venga de la distribución de entrenamiento.

---

## Registro de decisiones y hallazgos
(Anotar aquí al cerrar cada tarea: qué se encontró, qué se decidió, qué quedó pendiente.)

- 2026-07-04 (T1): Controlador extraído a `controller/navigation.py`, verificado bit a bit
  contra la lógica original. Bug nuevo encontrado y corregido: el sim sumaba (no concatenaba)
  los slices del cono frontal por ser ndarray → distancia frontal ~2× → velocidad media
  inflada 30%. El sim ahora es más fiel al real. `tests/test_navigation_smoke.py` añadido.
  CLAUDE.md actualizado a la nueva arquitectura.
- 2026-07-04: Roadmap creado a partir de la revisión crítica del proyecto. Bugs latentes
  identificados: (a) min() sobre scan crudo con rayos inválidos → emergencia permanente;
  (b) focal 640px vs FOV 60° → distancias infladas ~15%; (c) posible fliplr=0.5 en el
  entrenamiento → confusión left/right; (d) giro a lazo abierto + timing del loop real →
  sobregiro variable; (e) finish se dispara con 1 solo frame → fin de carrera irreversible.
