# ============================================================
#  Makefile para TurtleBot4
# ============================================================

DOMAIN_ID ?= 1
ROBOT_NAME ?= turtlebot4
PAIRING_CODE ?= SALVAME_CORTIJO
PORT ?= 6000

.PHONY: run ros bridge clean stop info verify status

# ---- Default: mostrar como usarlo ----
run:
	@echo ""
	@echo "  TurtleBot4 — Comandos disponibles"
	@echo "  =================================="
	@echo ""
	@echo "  Paso 0:  make verify             Verificar hardware (LIDAR/Camara)"
	@echo "  Paso 1:  make ros                Lanzar ros2 bringup"
	@echo "  Paso 2:  make bridge             Lanzar turtle_bridge.py"
	@echo ""
	@echo "  make status                      Ver WiFi, IP, Bateria y Recursos"
	@echo "  make stop                        Matar ambos procesos"
	@echo "  make clean                       Borrar scripts antiguos"
	@echo "  make info                        Ver configuracion actual"
	@echo ""
	@echo "  Para cambiar el domain_id:       make ros DOMAIN_ID=3"
	@echo ""

# ---- Estado General (WiFi, Bateria, Recursos) ----
status:
	@echo ""
	@echo "[ ESTADO DEL TURTLEBOT4 ]"
	@echo "========================================"
	@echo -n "  WiFi Actual:   "
	@iwgetid -r || echo "Desconectado/No disponible"
	@echo -n "  Direccion IP:  "
	@hostname -I | awk '{print $$1}'
	@echo "----------------------------------------"
	@echo "  BATERIA DEL ROBOT:"
	@echo "  Consultando a ROS 2... (espera maxima de 5s)"
	@export ROS_DOMAIN_ID=$(DOMAIN_ID) && \
	timeout 5 bash -c 'ros2 topic echo /battery_state --once 2>/dev/null | grep -E "voltage|percentage"' || echo "  [!] No se pudo leer la bateria. ¿Esta corriendo 'make ros'?"
	@echo "----------------------------------------"
	@echo "  RECURSOS DE LA RASPBERRY PI:"
	@echo -n "    Uptime:   " && uptime -p
	@echo -n "    Memoria:  " && free -h | awk '/^Mem:/ {print $$3 " usados de " $$2}'
	@echo -n "    Temp CPU: " && cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null | awk '{print $$1/1000 " C"}' || echo "N/A"
	@echo "========================================"
	@echo ""

# ---- Paso 0: Verificar Hardware ----
verify:
	@echo ""
	@echo "[ Verificando Hardware del TurtleBot4 ]"
	@echo "----------------------------------------"
	@echo -n "  LIDAR (RPLIDAR): "
	@if ls /dev/ttyUSB* > /dev/null 2>&1; then \
		echo "CONECTADO ($$(ls /dev/ttyUSB* | head -n 1))"; \
	else \
		echo "NO DETECTADO (Revisar cable USB)"; \
	fi
	@echo -n "  Camara (OAK-D):  "
	@if lsusb | grep -i "luxonis\|movidius" > /dev/null 2>&1; then \
		echo "CONECTADA"; \
	else \
		echo "NO DETECTADA (Revisar cable USB interno)"; \
	fi
	@echo "----------------------------------------"
	@echo "Nota: Si la camara esta conectada pero no funciona en ROS, "
	@echo "es probable que un proceso la tenga ocupada. Liberar con:"
	@echo "  sudo fuser -k /dev/video*  o  pkill -f oakd"
	@echo ""

# ---- Paso 1: ros2 launch ----
ros:
	export ROS_DOMAIN_ID=$(DOMAIN_ID) && \
	ros2 launch turtlebot4_bringup lite.launch.py

# ---- Paso 1.5: Liberar Cámara para Python ----
stop_cam:
	@echo "Liberando cámara OAK-D de ROS 2..."
	@-pkill -f oakd_container
	@-pkill -f component_container
	@echo "¡Cámara liberada con éxito para usar con YOLO!"

# ---- Paso 2: turtle_bridge ----
bridge:
	export ROS_DOMAIN_ID=$(DOMAIN_ID) && \
	python3 ~/turtle_bridge.py \
		--ros-args \
		-p robot_name:=$(ROBOT_NAME) \
		-p pairing_code:=$(PAIRING_CODE) \
		-p port:=$(PORT)

# ---- Limpiar scripts viejos ----
clean:
	rm -f ~/controller_template.py ~/enviador.py ~/recibidor.py ~/recibidor_datos.py
	@echo "Scripts antiguos eliminados."

# ---- Matar procesos ----
stop:
	-pkill -f "turtle_bridge.py" 2>/dev/null
	-pkill -f "lite.launch.py" 2>/dev/null
	@echo "Procesos detenidos."

# ---- Ver configuracion ----
info:
	@echo "ROS_DOMAIN_ID:  $(DOMAIN_ID)"
	@echo "ROBOT_NAME:     $(ROBOT_NAME)"
	@echo "PAIRING_CODE:   $(PAIRING_CODE)"
	@echo "PORT:           $(PORT)"
