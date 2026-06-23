# capbot-ros-foxy

Stack autónomo ROS2 Foxy + nav2 para la Jetson Nano, en Docker (base L4T). Robot
real del Capstone, sin simulación. El contenedor es el servicio único de la
Jetson: posee la cámara CSI y el serie del ESP32, y reexpone a la GUI
`capbot-host` (no se toca la GUI).

## Quickstart (en la Jetson)

```bash
cd capbot-ros-foxy
L4T_TAG=r32.7.1 docker compose -f docker/docker-compose.yml build
HOST_IP=<IP_del_PC> docker compose -f docker/docker-compose.yml up
```

Preparación de la Jetson, nav2 y troubleshooting: ver [docker/README.md](docker/README.md).

## Estructura

```
docker/         Dockerfile + compose + entrypoint
src/test_bot/   paquete ROS2 (scripts, description, config, launch, maps)
```

## Fases

- 0-1: Docker + cámara CSI (tee: ArUco + video a la GUI) + aruco_localizer.
- 2: puente serie ESP32 (/odom) + EKF + teleop gateway.
- 3: nav2 + gui_bridge_node.
- 4: firmware VEL_CMD (repo capbot-ESP32).
