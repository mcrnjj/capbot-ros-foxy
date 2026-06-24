#!/usr/bin/env bash
# build_nav2.sh - compila nav2 (Foxy) DESDE FUENTE dentro del contenedor.
#
# Pensado para ITERAR: el workspace vive en /nav2_ws, que conviene montar en un
# volumen del host para que colcon cachee el progreso entre intentos (cada
# reintento solo recompila lo que falto/cambio).
#
# Uso (en el host, desde la raiz del repo):
#   mkdir -p ~/nav2_ws
#   sudo docker run -it --rm \
#       -v "$PWD/nav2:/nav2_src:ro" \
#       -v "$HOME/nav2_ws:/nav2_ws" \
#       capbot-ros-foxy:foxy-l4t \
#       bash /nav2_src/build_nav2.sh
#
# Cuando termine OK, el resultado queda en ~/nav2_ws/install (host).
# Recomendado: SWAP de 6 GB+ antes (ver docker/README.md).

set -e
SRC_DIR="$(cd "$(dirname "$0")" && pwd)"
NAV2_WS=/nav2_ws

# ROS base
if [ -f /opt/ros/${ROS_DISTRO}/install/setup.bash ]; then
    source /opt/ros/${ROS_DISTRO}/install/setup.bash
else
    source /opt/ros/${ROS_DISTRO}/setup.bash
fi
[ -f /opt/deps_ws/install/setup.bash ] && source /opt/deps_ws/install/setup.bash

# Herramientas y libs de sistema (Ubuntu, no-ROS)
pip3 install -q vcstool rosdep || true
apt-get update && apt-get install -y --no-install-recommends \
    libasio-dev libtinyxml2-dev libzmq3-dev libgraphicsmagick++1-dev \
    libyaml-cpp-dev libeigen3-dev wget || true

mkdir -p "${NAV2_WS}/src"
cd "${NAV2_WS}"

# Importar fuentes. --force => si cambiamos la version/rama de un repo en el
# .repos, al re-correr hace checkout de la nueva (sin esto, vcs deja la vieja).
vcs import --force src < "${SRC_DIR}/nav2.repos" || true

# Paquetes que NO compilamos en este robot:
#   smac_planner -> necesita ompl (build pesado); usamos navfn (Dijkstra/A*).
#   rviz_plugins -> necesita rviz.   system_tests -> tests + gazebo.
#   amcl -> localizacion por lidar; aca localizamos con EKF + ArUco.
for p in nav2_smac_planner nav2_rviz_plugins nav2_system_tests nav2_amcl; do
    d="$(find src -maxdepth 3 -type d -name "$p" 2>/dev/null | head -n1)"
    [ -n "$d" ] && touch "$d/COLCON_IGNORE"
done

# rosdep: instala SOLO deps de sistema (Ubuntu). Las ROS que no estan en la base
# se compilan desde fuente (BT.CPP, angles, bond) y se skipean aca. Las de
# middleware/DDS ya vienen en la base (skip_keys oficiales).
rosdep init 2>/dev/null || true
rosdep update || true
rosdep install --from-paths src --ignore-src -r -y \
    --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 \
                 urdfdom_headers libopensplice67 libopensplice69 \
                 behaviortree_cpp_v3 ompl \
                 gazebo_ros_pkgs gazebo_ros gazebo_dev gazebo_plugins \
                 slam_toolbox nav2_rviz_plugins nav2_smac_planner \
                 nav2_amcl nav2_system_tests" || true

# Build (incremental; -j2 + swap para no agotar RAM en la Nano).
MAKEFLAGS="-j2" colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release --no-warn-unused-cli

echo ""
echo "==== nav2 build terminado. Resultado en ${NAV2_WS}/install ===="
