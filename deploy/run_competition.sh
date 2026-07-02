#!/usr/bin/env bash
# Lanza la navegación autónoma de competencia EN el TurtleBot4.
# Uso (en el robot):
#   bash deploy/run_competition.sh            # arranca con cuenta regresiva de 3 s
#   bash deploy/run_competition.sh --now      # arranca inmediatamente
#
# Requisitos previos: bringup corriendo (ros2 launch turtlebot4_bringup lite.launch.py)
# y dependencias instaladas (bash deploy/install_on_robot.sh).

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="$HOME/tb4_controller_venv"

# Entorno ROS 2 (Jazzy o Humble, el que exista) + config del robot (ROS_DOMAIN_ID, RMW).
for distro in jazzy humble; do
    if [ -f "/opt/ros/$distro/setup.bash" ]; then
        source "/opt/ros/$distro/setup.bash"
        break
    fi
done
[ -f /etc/turtlebot4/setup.bash ] && source /etc/turtlebot4/setup.bash

# Venv con ultralytics/numpy (creado por install_on_robot.sh).
[ -f "$VENV_DIR/bin/activate" ] && source "$VENV_DIR/bin/activate"

echo "=================================================="
echo " TURTLEBOT4 - NAVEGACIÓN AUTÓNOMA DE COMPETENCIA"
echo "=================================================="
echo " ROS_DOMAIN_ID = ${ROS_DOMAIN_ID:-<no definido>}"
echo " Directorio    = $APP_DIR"
echo " Python        = $(which python3)"
echo "=================================================="

if [ "$1" != "--now" ]; then
    for i in 3 2 1; do
        echo "Arrancando en $i..."
        sleep 1
    done
fi

cd "$APP_DIR"
exec python3 run_real_autonomous.py
