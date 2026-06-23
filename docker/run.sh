#!/usr/bin/env bash
# Run del contenedor SIN docker-compose (util en Jetson JetPack 4.x).
#
# Uso:
#   HOST_IP=192.168.1.10 ./docker/run.sh
#   HOST_IP=192.168.1.10 ./docker/run.sh ros2 launch test_bot real_robot.launch.py enable_nav:=false
#   ./docker/run.sh bash                  # shell de depuracion
#
# Variables:
#   HOST_IP        IP del PC (destino del video H264/RTP). Vacio => sin video a la GUI.
#   SERIAL_DEV     puerto del ESP32 (def /dev/ttyTHS1). Si no existe, se omite (Fase 1 no lo necesita).
#   ROS_DOMAIN_ID  def 0
#   IMAGE          def capbot-ros-foxy:foxy-l4t
set -e

cd "$(dirname "$0")/.."

HOST_IP="${HOST_IP:-}"
SERIAL_DEV="${SERIAL_DEV:-/dev/ttyTHS1}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
IMAGE="${IMAGE:-capbot-ros-foxy:foxy-l4t}"
NAME="${NAME:-capbot}"

# El serie es opcional en Fase 1 (solo camara). Solo se mapea si existe.
DEV_ARGS=()
if [ -e "${SERIAL_DEV}" ]; then
    DEV_ARGS+=(--device "${SERIAL_DEV}")
else
    echo "AVISO: ${SERIAL_DEV} no existe; corriendo sin serie (ok para Fase 1)."
fi

exec docker run -it --rm \
    --name "${NAME}" \
    --runtime nvidia \
    --network host \
    --ipc host \
    -e HOST_IP="${HOST_IP}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
    "${DEV_ARGS[@]}" \
    -v /tmp/argus_socket:/tmp/argus_socket \
    -v "$PWD/src/test_bot/config:/ros_ws/install/test_bot/share/test_bot/config:ro" \
    -v "$PWD/src/test_bot/maps:/ros_ws/install/test_bot/share/test_bot/maps:ro" \
    "${IMAGE}" "$@"
