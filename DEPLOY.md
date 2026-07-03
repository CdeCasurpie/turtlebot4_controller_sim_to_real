# Guía completa: instalar y correr el controlador autónomo en el TurtleBot4

Esta guía asume que **nunca has usado el TurtleBot4**. Si la sigues de arriba a abajo,
terminas con el robot recorriendo el circuito solo. Cada paso dice **qué hacer**,
**qué deberías ver** y **por qué se hace**.

La guía tiene 3 partes según la frecuencia:

| Parte | Cuándo | Tiempo |
|---|---|---|
| [A. Entender el sistema](#parte-a--cómo-funciona-el-sistema-léelo-una-vez) | Léela una vez | 5 min |
| [B. Instalación inicial](#parte-b--instalación-inicial-se-hace-una-sola-vez) | **Una sola vez** | ~30-40 min |
| [C. Rutina de cada sesión](#parte-c--rutina-de-cada-sesión-cada-vez-que-enciendes-el-robot) | Cada vez que enciendes el robot | ~5 min |
| [D. Correr la navegación autónoma](#parte-d--correr-la-navegación-autónoma) | Cuando quieras que corra solo | 1 min |
| [E. Checklist del día de competencia](#parte-e--checklist-del-día-de-competencia) | El día de la carrera | — |
| [F. Opcional: YOLO en la VPU](#parte-f--opcional-correr-yolo-dentro-de-la-cámara-vpu) | Si quieres más rendimiento | ~20 min |
| [G. Solución de problemas](#parte-g--solución-de-problemas) | Cuando algo falle | — |

---

# PARTE A — Cómo funciona el sistema (léelo una vez)

## Qué es el robot físicamente

El **TurtleBot4 Lite** son 3 aparatos apilados:

1. **Base iRobot Create 3** (el disco de abajo): tiene las ruedas y los motores.
   Solo obedece órdenes de velocidad ("avanza a X m/s, gira a Y rad/s").
2. **Raspberry Pi 4** (la computadora de en medio): una mini-PC con Linux (Ubuntu).
   Aquí es donde se instala y corre **nuestro código**.
3. **Sensores** (arriba): un **lidar** RPLIDAR (láser giratorio que mide la distancia
   a los obstáculos en 360°) y una **cámara OAK-D-Lite** (con la que detectamos las
   señales de tránsito del circuito).

## Qué es ROS 2 (lo mínimo que necesitas saber)

Todos los componentes del robot se comunican con **ROS 2**, un sistema de mensajería.
Cada flujo de datos se llama **tópico** y tiene un nombre:

- `/scan` → el lidar publica aquí las 360 distancias, varias veces por segundo.
- `/oakd/rgb/preview/image_raw` → la cámara publica aquí sus imágenes.
- `/cmd_vel` → quien quiera mover el robot escribe aquí velocidades; la base Create 3 las obedece.

Dos detalles de ROS que causan el 90% de los problemas:

- **`ROS_DOMAIN_ID`**: un número de "canal". Dos programas solo se ven entre sí
  si usan el mismo número. Nuestros scripts lo toman automáticamente de la
  configuración del propio robot, así que normalmente no hay que tocarlo.
- **Bringup**: al encender el robot, los sensores NO publican solos. Hay que lanzar
  un programa (el "bringup") que enciende lidar, cámara y la conexión con la base.
  Sin bringup no hay `/scan` ni imágenes y nuestro controlador se queda esperando.

## Qué hace nuestro código

Todo corre **dentro de la Raspberry Pi del robot** (la laptop solo se usa para
copiar archivos y dar la orden de arranque). Así el WiFi puede fallar a mitad de
carrera y el robot sigue solo.

```
        EN EL ROBOT (Raspberry Pi 4)
┌─────────────────────────────────────────────┐
│  bringup ─→ publica /scan (lidar)           │
│             publica imágenes de la cámara   │
│                                             │
│  run_real_autonomous.py (nuestro programa): │
│    • lee el lidar 20 veces por segundo      │
│    • YOLO busca señales left/right/stop     │
│      en las imágenes (en un hilo aparte)    │
│    • una máquina de estados decide:         │
│      explorar / girar / detenerse / evadir  │
│    • publica velocidades en /cmd_vel        │
└─────────────────────────────────────────────┘

        EN TU LAPTOP (solo para operar)
   copiar código (scp) · dar la orden por SSH
```

**SSH** es la forma de "entrar" a la Raspberry Pi desde tu laptop: abres una terminal
que en realidad está corriendo dentro del robot.

---

# PARTE B — Instalación inicial (se hace UNA sola vez)

> Necesitas: el robot encendido, tu laptop con Windows, y ambos en la **misma red WiFi**.
> El robot necesita internet durante este proceso (para descargar librerías).

## Paso B1. Conecta tu laptop al WiFi del laboratorio

- Red: `Lab_Computech_5G` — contraseña: `Computech2025!`

**Por qué:** el robot ya está configurado para conectarse a esa red. Para hablarle
por SSH, tu laptop tiene que estar en la misma red.

## Paso B2. Averigua la IP del robot

La IP es la "dirección" del robot en la red, algo como `192.168.0.102`.
**Cambia cada vez que el robot se enciende** (el router se la asigna), así que este
paso se repite en cada sesión.

Opciones (de más fácil a más segura):

1. En una terminal de Windows (PowerShell): `ping turtlebot4.local` — si responde,
   esa es la IP.
2. Entrar al panel del router: navegador → `tplinkwifi.net` (contraseña `turtlebot4`)
   → lista de dispositivos conectados → buscar "turtlebot4" o "ubuntu".

## Paso B3. Verifica que puedes entrar al robot por SSH

En PowerShell:

```powershell
ssh ubuntu@<IP_ROBOT>
```

(reemplaza `<IP_ROBOT>` por la IP del paso anterior; contraseña: `turtlebot4`)

**Qué deberías ver:** la primera vez pregunta `Are you sure you want to continue
connecting?` → escribe `yes`. Luego pide la contraseña y el prompt cambia a algo como
`ubuntu@turtlebot4:~$` — **ya estás "dentro" del robot**.

Escribe `exit` para salir y volver a tu laptop.

**Por qué:** si SSH no funciona, nada de lo que sigue funcionará. Mejor detectarlo ahora.

## Paso B4. Copia el código del proyecto al robot

En PowerShell, **en tu laptop**:

```powershell
cd c:\Users\juanp\OneDrive\Escritorio\turtlebot4_controller_sim_to_real
.\deploy\deploy_to_robot.ps1 -RobotIp <IP_ROBOT>
```

Te pedirá la contraseña (`turtlebot4`) una o dos veces.

**Qué deberías ver:** una lista de archivos copiándose y al final
`Despliegue OK en ~/turtlebot4_controller`.

**Por qué:** el código vive en tu laptop, pero tiene que ejecutarse en la Raspberry Pi.
Este script copia **solo lo necesario** (el controlador, el modelo YOLO `best.pt` y los
scripts de instalación) a la carpeta `~/turtlebot4_controller` dentro del robot. No copia
el simulador ni los tests de pygame porque esos solo sirven en tu PC.

## Paso B5. Instala las dependencias dentro del robot

Entra al robot y lanza el instalador:

```powershell
ssh ubuntu@<IP_ROBOT>
```

y ya dentro del robot:

```bash
cd ~/turtlebot4_controller
bash deploy/install_on_robot.sh
```

**Qué deberías ver:** 4 etapas numeradas. La etapa 3 (instalación de librerías de
Python) puede tardar **10-20 minutos** porque descarga PyTorch para ARM — es normal.
Al final: `Instalación completa`.

**Por qué cada cosa que hace:**

- Crea un **entorno virtual de Python** (`~/tb4_controller_venv`): una instalación de
  librerías aislada, para no romper las librerías del sistema del robot (que ROS necesita).
- Instala `numpy`, `ultralytics` (la librería que ejecuta YOLO) y `opencv`.
- **Exporta el modelo a NCNN**: convierte `best.pt` a un formato optimizado para el
  procesador ARM de la Raspberry. Con esto YOLO pasa de ~1 imagen/segundo a ~5-10.
  Si esta parte falla no pasa nada: el controlador usa el modelo original (más lento
  pero funcional).

✅ **Fin de la instalación.** Esto no se vuelve a hacer, salvo que cambies el código
(en ese caso solo repites el paso B4).

---

# PARTE C — Rutina de cada sesión (cada vez que enciendes el robot)

## Paso C1. Enciende el robot y espera

Presiona el botón de encendido y espera ~1-2 minutos hasta que haga su sonido/luces de listo.

**Por qué:** la Raspberry tarda en arrancar Linux y en conectarse al WiFi. Si intentas
lo siguiente muy pronto, el SSH fallará.

## Paso C2. Averigua la IP y entra por SSH

Igual que en B2/B3 (la IP puede haber cambiado):

```powershell
ssh ubuntu@<IP_ROBOT>
```

## Paso C3. Lanza el bringup (enciende los sensores)

Dentro del robot:

```bash
ros2 launch turtlebot4_bringup lite.launch.py
```

**Qué deberías ver:** muchas líneas de log y ningún error rojo repetitivo. Esta
terminal se queda "ocupada" mostrando logs — **déjala abierta**, el bringup debe
seguir corriendo todo el tiempo.

**Por qué:** este es el programa que enciende el lidar, la cámara y la comunicación
con la base. Sin él, nuestro controlador no recibe ningún dato.

> ⚠️ **Es normal que falle.** Si se llena de errores o no arranca: apaga el robot,
> abre la compuerta trasera, desconecta y vuelve a conectar el cable (quedó flojo),
> enciende de nuevo y reintenta. Es el problema más común de este robot.

## Paso C4. Verifica que los sensores publican

Abre una **segunda terminal** SSH (`ssh ubuntu@<IP_ROBOT>` de nuevo en otra ventana de PowerShell) y ejecuta:

```bash
ros2 topic hz /scan
```

**Qué deberías ver:** `average rate: 7.5` (o similar) apareciendo cada segundo.
Significa que el lidar publica datos. Corta con `Ctrl+C`. Luego:

```bash
ros2 topic hz /oakd/rgb/preview/image_raw
```

**Qué deberías ver:** un rate de ~15-30. Significa que la cámara publica imágenes.

**Por qué:** es la forma más rápida de saber si el bringup realmente funcionó. Si
alguno de los dos no muestra nada, ve a la [Parte G](#parte-g--solución-de-problemas)
antes de continuar — el controlador no puede funcionar sin sensores.

## Paso C5 (opcional pero recomendado). Pruebas de sanidad

Estas dos pruebas mueven el robot: hazlas con **espacio libre alrededor**.
En la segunda terminal SSH:

```bash
cd ~/turtlebot4_controller
source ~/tb4_controller_venv/bin/activate
```

(`source ...` activa el entorno virtual con las librerías que instalamos; hay que
hacerlo en cada terminal nueva donde vayas a correr Python a mano.)

**Prueba 1 — lidar** (el robot avanza despacio y esquiva):

```bash
python3 test_controller.py
```

**Qué deberías ver:** una línea que se actualiza con las distancias en 4 direcciones:
`0°` (frente), `90°` (izquierda), `180°` (atrás), `270°` (derecha). **Verifica con tu
mano**: acércala por delante del robot y debe bajar el número de `0°`; por la izquierda,
el de `90°`. Si los lados están intercambiados o rotados, ver Parte G ("esquiva hacia
el lado equivocado"). Corta con `Ctrl+C`.

**Prueba 2 — cámara + YOLO** (el robot gira para centrar la señal):

```bash
python3 test_vision.py
```

**Qué deberías ver:** al mostrarle una señal impresa (left/right/stop) a 1-2 metros,
la reporta en pantalla y gira hacia ella. Corta con `Ctrl+C`.

**Por qué:** estas pruebas aíslan cada sensor. Si algo está mal (lidar rotado, cámara
muerta, modelo que no carga), lo descubres aquí en 2 minutos y no a mitad de la carrera.

---

# PARTE D — Correr la navegación autónoma

Con el bringup corriendo (Paso C3), en la otra terminal SSH:

```bash
cd ~/turtlebot4_controller
bash deploy/run_competition.sh
```

**Qué deberías ver:**

```
==================================================
 TURTLEBOT4 - NAVEGACIÓN AUTÓNOMA DE COMPETENCIA
==================================================
 ROS_DOMAIN_ID = ...
 ...
Arrancando en 3...
[VISION] YOLO cargado exitosamente: ...
[VISION] Inferencia YOLO: XXX ms/frame
[EXPLORANDO        ]  Frente: 2.31m | v: 0.30 w: 0.00
```

y el robot empieza a moverse solo. La última línea se actualiza en vivo con el estado
(EXPLORANDO, GIRANDO_IZQ, DETENIDO, EVASION_EMERGENCIA...), la distancia al frente y
las velocidades.

- **Para DETENERLO**: `Ctrl+C` en esa terminal — frena el robot antes de salir.
- La cuenta regresiva de 3 s es para que te dé tiempo de soltar el robot; con
  `bash deploy/run_competition.sh --now` arranca al instante.

**Qué hace el script por ti** (por si te lo preguntas): carga el entorno de ROS,
lee el `ROS_DOMAIN_ID` correcto de la configuración del robot, activa el entorno
virtual de Python y lanza `run_real_autonomous.py`. Es exactamente lo que harías a
mano, pero sin poder equivocarte.

### Dato: el número `[VISION] Inferencia YOLO: XXX ms/frame`

Es cuánto tarda cada análisis de imagen:

- **~100-300 ms** → estás usando el modelo NCNN optimizado. Perfecto.
- **~1000 ms** → está usando el modelo original (el export NCNN falló). El robot
  funciona igual (la visión corre en un hilo aparte y no frena el control), pero
  detecta las señales con más retraso. Si quieres arreglarlo: Parte G.
- ¿Quieres aún más velocidad de visión? Mira la [Parte F (VPU)](#parte-f--opcional-correr-yolo-dentro-de-la-cámara-vpu).

### Modo carrera: 0.46 m/s con `safety_override=full`

La base Create 3 limita la velocidad a **0.306 m/s** por defecto ("safe mode").
Las reglas de la competencia permiten desactivarlo, lo que sube el tope físico a
**0.46 m/s** (+50%). Son dos pasos:

**1. Desactivar el límite en la base** (cada vez que se enciende el robot, después
del bringup):

```bash
ros2 param set /motion_control safety_override full
# debe responder: Set parameter successful
ros2 param get /motion_control safety_override   # verificar: full
```

**2. Subir el tope del controlador** en `TurtleBotController/config.json`:

```json
"controller": {
    "v_max": 0.46,
    "w_max": 1.8
}
```

Consejos:
- Prueba PRIMERO todo el circuito a 0.30 (config por defecto). Cambia a 0.46 solo
  cuando el robot complete el recorrido de forma consistente.
- El controlador ya escala el frenado con la velocidad (frena según el "claro"
  que ve por delante), pero a 0.46 la distancia de reacción real crece: si ves
  frenadas de emergencia frecuentes en las curvas, vuelve a 0.30 para la
  clasificación y usa 0.46 solo si necesitas el tiempo.
- `safety_override=full` también desactiva la protección de acantilados
  (escaleras). En un circuito plano no importa.

### Si el WiFi del lab es inestable: usa tmux

Si se corta tu SSH mientras el programa corre, el programa **muere** (y el robot se
detiene). `tmux` evita eso: crea una sesión de terminal que vive dentro del robot,
independiente de tu conexión.

```bash
tmux new -s carrera            # crea la sesión (el prompt cambia, barra verde abajo)
bash deploy/run_competition.sh # corre el controlador DENTRO de tmux
```

- Salir de tmux sin matar el programa: `Ctrl+B`, soltar, luego `D`.
- Volver a ver el programa: `tmux attach -t carrera`.
- Aunque se caiga el WiFi o cierres la laptop, el robot sigue corriendo.

---

# PARTE E — Checklist del día de competencia

Imprime esto o tenlo a mano:

- [ ] **1.** Encender el robot, esperar el sonido de listo (~2 min).
- [ ] **2.** Conectar laptop al WiFi del lab. Averiguar IP del robot (paso B2).
- [ ] **3.** SSH terminal 1: `ros2 launch turtlebot4_bringup lite.launch.py` → dejarla corriendo.
      Si falla: reasentar el cable trasero y reintentar (¡llegar con tiempo!).
- [ ] **4.** SSH terminal 2: `ros2 topic hz /scan` y `ros2 topic hz /oakd/rgb/preview/image_raw` → ambos publican.
- [ ] **5.** Prueba rápida de visión con una señal (`test_vision.py`, 1 min).
- [ ] **6.** Colocar el robot en la salida.
- [ ] **7.** En terminal 2: `cd ~/turtlebot4_controller && tmux new -s carrera` y
      `bash deploy/run_competition.sh` → durante el "Arrancando en 3..." soltar el robot.
- [ ] **8.** Para detenerlo al final: `Ctrl+C` (o `tmux attach -t carrera` primero, si te desconectaste).
- [ ] Entre intentos: `Ctrl+C`, reposicionar el robot, volver a lanzar (el bringup **no**
      se reinicia, sigue corriendo en la terminal 1).

**Plan B sin laptop** (opcional): se puede dejar instalado un servicio que arranca la
navegación solo con encender el robot — instrucciones dentro de
[deploy/turtlebot4-autonomous.service](deploy/turtlebot4-autonomous.service). Útil como
respaldo si el WiFi del local es un desastre, pero pierdes la posibilidad de ver logs
y de frenarlo cómodamente, así que úsalo solo si el SSH es inviable.

---

# PARTE F — Opcional: correr YOLO dentro de la cámara (VPU)

**Qué es:** la cámara OAK-D-Lite trae su propio chip de inteligencia artificial
(una "VPU" Intel Myriad X). Podemos hacer que YOLO corra **dentro de la cámara** en
lugar del procesador de la Raspberry: ~15-25 detecciones por segundo y el CPU del Pi
queda libre para el control.

**¿Lo necesito?** No necesariamente. Si en la Parte D viste `~100-300 ms/frame`,
el sistema ya funciona bien. La VPU es un plus de rendimiento que requiere una
calibración extra. Recomendación: ten primero todo funcionando sin VPU, y prueba
esto solo si te sobra un día de laboratorio.

## F1. Convertir el modelo (una vez, en tu laptop)

La VPU no entiende el formato `.pt`; necesita un formato propio llamado "blob".
La conversión se hace en una página web del fabricante:

1. Abre **https://tools.luxonis.com** en el navegador.
2. Sube el archivo `yolonano/best.pt`. Opciones: versión **YOLOv8 (detection)**,
   input shape **416**, plataforma **RVC2**.
3. Descarga el ZIP resultante. Contiene un `.blob` y un `.json`.
4. Cópialos en tu repo con estos nombres exactos:
   - `yolonano/vpu/best.blob`
   - `yolonano/vpu/best.json`
5. Vuelve a correr `.\deploy\deploy_to_robot.ps1 -RobotIp <IP>` — detecta la carpeta
   `vpu/` y la copia al robot.

## F2. Activarlo en el robot

La cámara solo puede tener **un dueño a la vez**. Normalmente el dueño es el nodo
`oakd` de ROS (el que publica las imágenes). Para que nuestra VPU pueda usarla,
hay que apagar ese nodo:

```bash
sudo systemctl stop oakd     # libera la cámara (el lidar no se ve afectado)
cd ~/turtlebot4_controller
source ~/tb4_controller_venv/bin/activate
python3 deploy/check_vpu.py  # prueba en vivo
```

**Qué deberías ver:** `[VISION-VPU] Pipeline corriendo en la Myriad X...` y, al
mostrarle una señal, una línea tipo `left: 1.05m @ +2.3°` actualizándose. Al final
reporta los FPS.

Después de eso, `bash deploy/run_competition.sh` usará la VPU automáticamente
(el controlador detecta que el blob existe; en el arranque dirá
`[VISION] Backend activo: VPU`). Para volver al modo normal:
`sudo systemctl start oakd`.

## F3. Calibrar (importante — 5 minutos)

La imagen que ve la VPU está recortada distinto a la del modo normal, así que las
distancias estimadas salen con otra escala. Como el robot decide **detenerse ante un
STOP a 1.6 m**, hay que calibrar:

1. **Distancia**: pon una señal exactamente a **1.0 m** de la cámara y corre
   `check_vpu.py`. Si reporta por ejemplo `1.40m`, edita
   `TurtleBotController/config.json` → `vision.vpu.distance_scale` = `1.0/1.4` ≈ `0.71`.
   Repite hasta que reporte ~1.0 m.
2. **Detección débil**: la conversión al blob pierde algo de precisión; si detecta
   la señal de forma intermitente, baja `vision.confidence_threshold` de `0.85` a
   `0.75` y verifica que no aparezcan detecciones falsas.
3. Tras editar `config.json` en la laptop, vuelve a correr el deploy (paso B4), o
   edítalo directamente en el robot con `nano TurtleBotController/config.json`.

---

# PARTE G — Solución de problemas

| Qué pasa | Por qué pasa | Qué hacer |
|---|---|---|
| No puedo hacer SSH al robot | Robot aún arrancando, IP equivocada, o red distinta | Espera 2 min tras encender; verifica la IP en el router; confirma que laptop y robot están en el mismo WiFi |
| El bringup se llena de errores o no arranca | Cable interno flojo (clásico de este robot) | Apagar → reasentar el cable de la compuerta trasera → encender → reintentar |
| `ros2 topic hz /scan` no muestra nada | El bringup no está corriendo o murió | Revisa la terminal del bringup (paso C3); relánzalo |
| La cámara no publica imágenes | El nodo `oakd` se cayó o algo más usa la cámara | `sudo systemctl restart oakd` y volver a verificar con `ros2 topic hz` |
| El programa corre, dice EXPLORANDO, pero el robot **no se mueve** | La base espera otro tipo de mensaje en `/cmd_vel` (`Twist` vs `TwistStamped`, depende del firmware) | Edita `TurtleBotController/config.json`: cambia `use_twist_stamped` de `true` a `false` (o al revés) y relanza. Puedes confirmar el tipo con `ros2 topic info /cmd_vel -v` |
| El programa se queda en "Esperando sensores..." para siempre | El controlador está en otro "canal" ROS (`ROS_DOMAIN_ID`) que el robot | Usa siempre `bash deploy/run_competition.sh` (toma el domain correcto solo). Diagnóstico: `echo $ROS_DOMAIN_ID` y `ros2 topic list` en el robot |
| El robot esquiva hacia el lado equivocado | La orientación del lidar no coincide con la calibración (rotación de 90°) | Corre `python3 test_controller.py`, acerca la mano por el frente/izquierda y mira qué índice baja; ajusta la línea `scan = scan[90:] + scan[:90]` en `TurtleBotController/turtlebot.py` |
| Nunca detecta señales | YOLO no cargó, o la cámara no publica | En el log de arranque debe aparecer `[VISION] YOLO cargado exitosamente` y luego `[VISION] Inferencia YOLO: ...`. Si no: revisa cámara (arriba) y que `yolonano/best.pt` exista en el robot |
| `[VISION] Inferencia YOLO: ~1000 ms` (lento) | El export NCNN falló durante la instalación | En el robot: `source ~/tb4_controller_venv/bin/activate && python3 deploy/export_ncnn.py` y relanza |
| `Illegal instruction (core dumped)` al usar YOLO o exportar | pip instaló el torch para servidores ARM con GPU NVIDIA (versión `+cu130`), que usa instrucciones que el procesador del Pi no tiene | En el robot, dentro del venv: `pip uninstall -y torch torchvision triton` y luego `pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu`. El instalador ya lo hace solo desde la versión actual |
| `pip install` falla durante la instalación | La red del lab no tiene internet | Comparte internet desde tu celular (hotspot) al robot solo durante la instalación |
| VPU: error `X_LINK_DEVICE_ALREADY_IN_USE` | El nodo `oakd` de ROS todavía tiene la cámara | `sudo systemctl stop oakd` y reintenta |
| VPU: `X_LINK_DEVICE_NOT_FOUND` o error de permisos | Cable USB de la cámara o permisos del sistema | Reconecta el USB de la OAK-D. Si persiste: `echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \| sudo tee /etc/udev/rules.d/80-movidius.rules && sudo udevadm control --reload-rules && sudo udevadm trigger` |
| VPU corre pero no ve señales | El umbral de confianza (0.85) es alto para el modelo convertido | Baja `confidence_threshold` a 0.75 en `config.json` y calibra con `deploy/check_vpu.py` (Parte F3) |

**Sobre la velocidad máxima:** la base Create 3 en modo seguro no pasa de ~0.306 m/s.
Nuestro controlador ya limita a 0.3 m/s, así que no hay que configurar nada.

---

# Apéndice: qué es cada archivo de `deploy/`

| Archivo | Se ejecuta en | Qué hace |
|---|---|---|
| [deploy/deploy_to_robot.ps1](deploy/deploy_to_robot.ps1) | Tu laptop (PowerShell) | Copia el código y el modelo al robot por SSH |
| [deploy/install_on_robot.sh](deploy/install_on_robot.sh) | El robot | Instala Python venv + librerías + optimiza el modelo (una vez) |
| [deploy/run_competition.sh](deploy/run_competition.sh) | El robot | Prepara el entorno ROS y lanza la navegación autónoma |
| [deploy/export_ncnn.py](deploy/export_ncnn.py) | El robot | Convierte `best.pt` al formato rápido NCNN (lo llama el instalador) |
| [deploy/check_vpu.py](deploy/check_vpu.py) | El robot | Prueba y calibra el modo VPU (Parte F) |
| [deploy/turtlebot4-autonomous.service](deploy/turtlebot4-autonomous.service) | El robot | (Opcional) autoarranque de la navegación al encender |
| [deploy/requirements-robot.txt](deploy/requirements-robot.txt) | El robot | Lista de librerías Python que instala el instalador |
