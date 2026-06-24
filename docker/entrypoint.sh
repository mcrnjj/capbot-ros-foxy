#!/usr/bin/env bash
# Entrypoint del contenedor test_bot: sourcea ROS2 Foxy + (opcional) nav2 +
# el workspace compilado, y luego ejecuta el comando (por defecto el launch).
set -e

# ROS2 Foxy (dusty-nv lo deja en /opt/ros/foxy; soporta install/ o setup.bash)
if [ -f "/opt/ros/${ROS_DISTRO}/install/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO}/install/setup.bash"
else
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

# Underlay de deps desde fuente (xacro...)
if [ -f "/opt/deps_ws/install/setup.bash" ]; then
    source "/opt/deps_ws/install/setup.bash"
fi

# robot_localization (EKF, Fase 2)
if [ -f "/opt/rl_ws/install/setup.bash" ]; then
    source "/opt/rl_ws/install/setup.bash"
fi

# nav2 (si se compilo con BUILD_NAV2=1)
if [ -f "/opt/nav2_ws/install/setup.bash" ]; then
    source "/opt/nav2_ws/install/setup.bash"
fi

# Workspace del paquete
if [ -f "/ros_ws/install/setup.bash" ]; then
    source "/ros_ws/install/setup.bash"
fi

exec "$@"
