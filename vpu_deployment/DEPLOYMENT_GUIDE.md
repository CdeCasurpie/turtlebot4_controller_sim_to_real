# Guía de Despliegue: Inferencia YOLOv8 en la VPU de la OAK-D Lite

Meta: mover la inferencia de señales (`left`/`right`/`stop`/`finish`) de la CPU de la
Raspberry Pi 4 a la VPU (Myriad X) de la OAK-D Lite, vía `depthai`. Todo esto vive en
`vpu_deployment/`, un sandbox aislado que no toca `TurtleBotController/` (control real)
ni la lógica de navegación de producción.

El modelo ya está convertido y listo (`vpu_deployment/models/turtlebot_signals_v2.blob`,
4 clases: `left, right, stop, finish`) — el cómo se generó está en `MODEL_CONVERSION.md`,
no hace falta leerlo para lo que sigue. **Lo que falta es todo lo de abajo: desplegar en
la Raspberry Pi y verificar que la inferencia corre en la VPU, no en su CPU.**

## Pendiente 1: Copiar al robot

```bash
scp -r vpu_deployment/ pi@<ip-del-turtlebot4>:~/turtlebot4_controller_sim_to_real/
```

## Pendiente 2: Dependencias e instalar el dispositivo en la Pi

```bash
python3 -m pip install -r vpu_deployment/requirements.txt

# La OAK-D debe verse por USB3 ANTES de correr el pipeline completo
python3 -c "import depthai as dai; print(dai.__version__); print(dai.Device.getAllAvailableDevices())"
```

**Cuidado con la versión de `depthai`:** tiene que quedar en la rama 2.x (`requirements.txt` ya lo fija
con `depthai<3`). Si por lo que sea terminas con la 3.x instalada (por ejemplo, si alguien corrió
`pip install depthai` a secas en algún momento en ese mismo entorno), este script y
`TurtleBotController/vpu_vision.py` van a fallar — la 3.x eliminó `YoloDetectionNetwork`/`XLinkIn`/`XLinkOut`
y cambió cómo se crea el `Pipeline`. El primer `print` de arriba debe mostrar algo `2.x.x.x`; si muestra
`3.x.x`, corre `python3 -m pip install "depthai<3"` para bajarla.

Si no aparece o da error de permisos, reglas udev (una sola vez, luego reconectar la OAK-D):

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Pendiente 3: Correr en modo headless (el modo real de despliegue)

```bash
python3 vpu_deployment/test_depthai_yolo.py --conf 0.5 --iou 0.5 --fps 15
```

Por qué este es el comando correcto para el robot, y no un `--display`: `--source cam`
(default) usa la `ColorCamera` de la propia OAK-D — la imagen nunca sale del chip. El
nodo `YoloDetectionNetwork` hace preprocesamiento, inferencia y NMS **dentro de la VPU**;
lo único que cruza el USB hacia la Pi es el resultado ya resuelto (clase + confianza +
bbox por objeto, unos pocos floats). El script en este modo (`run_headless()`) ni siquiera
crea el stream de frame ni abre ventanas: usa `q_nn.get()` **bloqueante**, así el proceso
en la Pi queda dormido esperando a la VPU en vez de gastar CPU en un loop activo. Vas a
ver una línea que se actualiza con las detecciones (`left:0.93`, `stop:0.87`, ...) y nada
más. Ctrl+C para salir.

`--display` (streamear el frame de vuelta y abrir `cv2.imshow`) existe solo para depurar
en un banco de pruebas con monitor — a propósito gasta CPU/USB extra en el host para poder
dibujar las cajas, así que no es el modo a usar en el robot.

## Pendiente 4: Verificar que la carga vive en la VPU, no en la Pi

- [ ] Con `htop` en otra sesión SSH mientras corre el script en modo headless: el proceso Python se mantiene bajo y estable, ningún hilo satura un core.
- [ ] `python3 vpu_deployment/test_depthai_yolo.py --stats` imprime periódicamente uso de CPU (Leon CSS/MSS) y temperatura *del chip OAK-D* (nodo `SystemLogger`, corre dentro del dispositivo) — números concretos de que el trabajo no es de la Pi.
- [ ] Detecciones correctas sobre señales reales frente a la cámara (revisar con `--display` en un banco de pruebas si hace falta ver las cajas dibujadas).

## Después de esto: ya integrado a producción

Este sandbox valida el modelo aislado. La integración al controlador de producción ya se
hizo: `TurtleBotController/turtlebot.py` ya no corre `ultralytics` sobre la CPU, sino que usa
`TurtleBotController/vpu_vision.py: VpuYoloDetector`, que reproduce este mismo pipeline
DepthAI (mismo `.blob`, mismas clases) dentro de `TurtleBotReal.get_vision_detections()`.
Ver `DEPLOY_ROBOT_VPU.md` (raíz del repo) para la guía end-to-end de cómo desplegar y correr
`run_real_autonomous.py` con esto ya activado. Este sandbox (`test_depthai_yolo.py`) sigue
siendo útil como herramienta de diagnóstico standalone (validar el `.blob`/dispositivo sin
depender de ROS 2), pero ya no es el único lugar donde corre este modelo.
