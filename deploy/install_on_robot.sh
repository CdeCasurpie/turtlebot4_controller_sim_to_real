#!/usr/bin/env bash
# Instala las dependencias del controlador autónomo EN el TurtleBot4 (Raspberry Pi 4).
# Uso (en el robot, dentro del directorio del proyecto):
#   bash deploy/install_on_robot.sh
#
# Crea un venv con acceso a los paquetes del sistema (rclpy/cv_bridge vienen de ROS 2)
# e instala numpy/ultralytics/opencv. Al final intenta exportar el modelo a NCNN
# (mucho más rápido que torch en ARM); si falla, el controlador usa best.pt igual.

set -e

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="$HOME/tb4_controller_venv"

echo "=== [1/4] Paquetes del sistema ==="
sudo apt-get update
sudo apt-get install -y python3-venv python3-pip

echo "=== [2/4] Entorno virtual en $VENV_DIR ==="
if [ ! -f "$VENV_DIR/bin/activate" ]; then
    python3 -m venv --system-site-packages "$VENV_DIR"
fi
source "$VENV_DIR/bin/activate"
pip install --upgrade pip

echo "=== [3/4] Dependencias Python (torch para ARM puede tardar varios minutos) ==="
pip install -r "$APP_DIR/deploy/requirements-robot.txt"

echo "=== [4/4] Export del modelo YOLO a NCNN (opcional, acelera la inferencia en el Pi) ==="
if python3 "$APP_DIR/deploy/export_ncnn.py"; then
    echo "Modelo NCNN listo: el controlador lo usará automáticamente."
else
    echo "AVISO: el export NCNN falló. No pasa nada: se usará yolonano/best.pt con torch (más lento)."
fi

echo ""
echo "Instalación completa. Para correr la navegación autónoma:"
echo "  bash $APP_DIR/deploy/run_competition.sh"
