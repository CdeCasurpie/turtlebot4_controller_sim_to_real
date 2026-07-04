# Guía definitiva: correr la navegación autónoma en el robot, con YOLO en la VPU

Esta es la guía end-to-end para subir el repo al TurtleBot4 y dejar corriendo `run_real_autonomous.py`
(el mismo algoritmo de `test_autonomous_controller.py` — LiDAR + máquina de estados) alimentado por
detecciones de señales que corren **dentro de la VPU (Myriad X) de la OAK-D Lite**, no en la CPU de la
Raspberry Pi. Reemplaza al `ultralytics` sobre CPU que usaba antes `TurtleBotReal`.

## 0. Qué cambió (resumen del código)

- **`TurtleBotController/vpu_vision.py`** (nuevo): `VpuYoloDetector`, construye un pipeline DepthAI
  (`ColorCamera` → `YoloDetectionNetwork` → `XLinkOut "nn"`) idéntico en configuración al validado en
  `vpu_deployment/test_depthai_yolo.py`, y lo corre en un hilo de fondo con `.get()` bloqueante sobre la
  cola `nn`, exponiendo `get_detections()` en el mismo formato que ya consumía la máquina de estados:
  `[{'class', 'distance', 'relative_angle'}, ...]`.
- **`TurtleBotController/turtlebot.py`**: `TurtleBotReal` ya no carga `ultralytics.YOLO` ni se suscribe a
  un tópico ROS de imagen (`_TurtleBotRosNode` perdió `image_sub`/`latest_image`/`cv_bridge`).
  `get_vision_detections()` ahora delega en `self.vpu_detector.get_detections()`.
- **`TurtleBotController/config.json`**: la sección `vision` cambió de `yolo_model_path` (`.pt`, CPU) a
  `vpu_blob_path` + `classes_path` + `num_classes` + `iou_threshold` + `fps` (VPU). `ros.image_topic` se
  eliminó (ya no se usa). Ver el archivo para los valores actuales.
- **`run_real_autonomous.py` / `test_autonomous_controller.py`**: sin cambios funcionales por esto (ya
  tenían el manejo de la clase `finish` → estado `FINALIZADO` de un paso anterior); siguen consumiendo
  `get_vision_detections()` exactamente igual, sin saber ni importarles si la inferencia corre en CPU o VPU.

Nada de esto toca `Simulator/` — el simulador sigue usando su detector geométrico falso sobre `world.signals`.

## 1. Requisitos en el robot antes de empezar

- Raspberry Pi (o companion computer) con ROS 2 ya funcionando contra el Create 3 (LiDAR en `/scan`,
  `/cmd_vel` moviendo la base) — esto es lo que ya validaba `test_controller.py` sin visión.
- OAK-D Lite conectada por **USB3** (cable/puerto azul; por USB2 la VPU no tiene ancho de banda suficiente).
- El repo debe incluir `vpu_deployment/models/turtlebot_signals_v2.blob` y `yolonanov2/classes.txt` — ya
  están versionados, no hace falta regenerarlos (eso está documentado, si algún día hace falta, en
  `vpu_deployment/MODEL_CONVERSION.md`).

## 2. Copiar el repo actualizado al robot

```bash
scp -r . pi@<ip-del-turtlebot4>:~/turtlebot4_controller_sim_to_real/
```

Si ya tenías una copia previa (por ejemplo, de cuando solo existía el sandbox `vpu_deployment/`), sobreescribe
completa — los archivos clave que cambiaron son `TurtleBotController/turtlebot.py`, `TurtleBotController/config.json`
y el nuevo `TurtleBotController/vpu_vision.py`.

## 3. Instalar `depthai` en el mismo entorno de Python que usa ROS 2

```bash
source /opt/ros/<tu-distro>/setup.bash   # el mismo que ya usas para correr run_real_autonomous.py
python3 -m pip install "depthai<3"
```

**Importante: tiene que ser la rama 2.x, no la última versión de PyPI.** `pip install depthai` a secas
instala hoy `depthai` 3.x, que reestructuró la API por completo — no existen `dai.node.YoloDetectionNetwork`,
`dai.node.XLinkIn` ni `dai.node.XLinkOut` (los que usa tanto `vpu_deployment/test_depthai_yolo.py` como
`TurtleBotController/vpu_vision.py`), y `dai.Pipeline()` cambió de firma. Si ya instalaste la 3.x por error,
bájala explícitamente: `python3 -m pip install "depthai<3"` (esto la reemplaza). Puedes confirmar la versión
activa con `python3 -c "import depthai; print(depthai.__version__)"` — debe imprimir algo `2.x.x.x`.

`ultralytics` y `cv_bridge` ya **no** son necesarios para este flujo (siguen instalados si los usas para
otra cosa, pero `TurtleBotReal` no los importa más).

Reglas udev para la VPU (una sola vez; sáltate esto si ya las instalaste al validar el sandbox):

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## 4. Verificar que la OAK-D se ve por USB3 antes de correr nada

```bash
python3 -c "import depthai as dai; print(dai.Device.getAllAvailableDevices())"
```

Si no aparece nada o hay error de permisos, revisa el cable/puerto USB3 y que las reglas udev del paso
anterior se aplicaron (reconecta el dispositivo tras instalarlas).

## 5. Verificar que el LiDAR y el `cmd_vel` están vivos (independiente de la visión)

```bash
ros2 topic hz /scan
ros2 topic info /cmd_vel
```

Si esto no funciona, el problema es de la capa ROS/Create 3, no de la VPU — resuélvelo antes de seguir
(es exactamente lo mismo que necesitaba `test_controller.py`, que no usa visión en absoluto).

## 6. Correr el controlador completo

Desde la **raíz del repo** (necesario para que `from TurtleBotController.turtlebot import TurtleBotReal`
resuelva el paquete):

```bash
cd ~/turtlebot4_controller_sim_to_real
python3 run_real_autonomous.py
```

Salida esperada al arrancar:

```
==================================================
 NAVEGACIÓN AUTÓNOMA FINAL (IDÉNTICA AL SIMULADOR)
==================================================
[VISION-VPU] Pipeline DepthAI iniciado: /home/pi/turtlebot4_controller_sim_to_real/vpu_deployment/models/turtlebot_signals_v2.blob
[TurtleBotController] Iniciando ROS 2 en el DOMAIN_ID: 77
[TurtleBotController] Esperando sensores (1 segundo)...
[TurtleBotController] ¡Robot Real Listo para actuar!

Robot listo. Comenzando...
[EXPLORANDO       ]              Frente: 1.85m | v: 0.80 w:  0.00
```

Cuando el modelo detecte una señal verás algo como `[BUSCANDO_IZQ      ] [LEFT:1.2m]  ...` en la misma
línea (se sobrescribe con `\r`, como ya hacía el script). Al ver `finish` a ≤1.6 m entra a `FINALIZADO`,
se detiene, imprime `¡Señal FINISH alcanzada! Deteniendo el robot y terminando el programa.` y el proceso
termina solo (no hace falta Ctrl+C).

## 7. Confirmar que la inferencia corre en la VPU, no en la Pi

- `htop` en otra sesión SSH mientras corre `run_real_autonomous.py`: el proceso Python debe mantenerse
  bajo y estable (el trabajo de red neuronal no debería aparecer como uso de CPU de la Pi).
- Si quieres números concretos del chip (uso de Leon CSS/MSS, temperatura) **antes** de arrancar el
  controlador completo, usa el sandbox aislado como diagnóstico puntual (no corras ambos a la vez —
  compiten por el mismo dispositivo USB):
  ```bash
  python3 vpu_deployment/test_depthai_yolo.py --stats
  ```
  Esto confirma que el mismo `.blob`/pipeline corre bien en tu OAK-D antes de depender de él dentro del
  controlador de producción.

## 8. Diferencias de comportamiento a tener en cuenta

- `v_target` tiene tope de **0.3 m/s** en `run_real_autonomous.py` (vs. 0.8 en el simulador) — límite de
  seguridad de hardware, no relacionado con la VPU.
- Se ignoran lecturas de LiDAR `< 0.18 m` por ser reflexiones del propio chasis del robot.
- El umbral de confianza (`confidence_threshold` en `config.json`, 0.85) ahora se aplica **dentro de la
  VPU** (`detection_nn.setConfidenceThreshold`), no como parámetro de una llamada Python — el efecto para
  la máquina de estados es idéntico: detecciones por debajo del umbral simplemente no llegan.

## 9. Troubleshooting

| Síntoma | Causa probable | Qué hacer |
|---|---|---|
| `[VISION-VPU] Error al iniciar la VPU: ...` al arrancar, el robot se mueve solo por LiDAR | La OAK-D no se detectó (USB2/cable, permisos) o falta `depthai` | Repetir pasos 3–4; el robot sigue funcionando sin visión, no se cae el proceso — pero tampoco reaccionará a señales |
| `FileNotFoundError` sobre el `.blob` | Ruta de `vpu_blob_path` en `config.json` no coincide con la copia del repo en el robot | Confirmar que `vpu_deployment/models/turtlebot_signals_v2.blob` existe en esa ruta relativa a `TurtleBotController/` |
| `ValueError: ... tiene N clases pero se esperaban 4` | `yolonanov2/classes.txt` fue editado o no coincide con `num_classes` en `config.json` | Revisar que `classes.txt` tenga exactamente `left`, `right`, `stop`, `finish` en ese orden |
| El robot nunca cambia de `EXPLORANDO` aunque haya señales físicas visibles | FOV/orientación de la OAK-D, o umbral de confianza demasiado alto para las condiciones de luz | Bajar `confidence_threshold` en `config.json` para probar, o validar detecciones con `vpu_deployment/test_depthai_yolo.py --display` en un banco de pruebas |
| `RuntimeError` de DepthAI al conectar (dispositivo ocupado) | Otro proceso (por ejemplo el sandbox `test_depthai_yolo.py`) sigue con el dispositivo abierto | Asegurarse de que solo un proceso a la vez use la OAK-D |

## 10. Rollback (si hiciera falta volver al pipeline por CPU)

El pipeline anterior (`ultralytics` + tópico ROS de imagen) queda en el historial de git antes de este
cambio — no se borró ningún archivo del modelo viejo (`yolonano/best.pt` sigue en el repo). Si se necesita
revertir, es cuestión de restaurar la versión previa de `TurtleBotController/turtlebot.py` y
`TurtleBotController/config.json` (`git log` sobre esos dos archivos) y reinstalar `ultralytics`/`cv_bridge`.

## 11. Resumen

Con estos cambios, `run_real_autonomous.py` corre en el robot real exactamente el mismo algoritmo de
`test_autonomous_controller.py` (misma máquina de estados, mismo `buscar_camino_libre`, mismos umbrales),
pero alimentado por `TurtleBotController/vpu_vision.py: VpuYoloDetector`, que ejecuta el modelo `yolonanov2`
(4 clases, incluyendo `finish`) enteramente dentro de la VPU Myriad X de la OAK-D Lite vía DepthAI — la
Raspberry Pi ya no decodifica frames ni corre la red neuronal, solo recibe detecciones resueltas; para
dejarlo funcionando en el robot solo hace falta copiar el repo actualizado, instalar `depthai<3`, confirmar
que la OAK-D se ve por USB3 y que `/scan`/`cmd_vel` ya funcionaban de antes, y correr
`python3 run_real_autonomous.py` desde la raíz del repo — el `.blob` y las clases ya están versionados,
no hay que reconvertir ni reentrenar nada para este paso.
