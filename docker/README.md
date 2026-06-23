# capbot-ros-foxy en Docker (ROS2 Foxy + nav2) — Jetson Nano

Contenedor **servicio único** de la Jetson: posee el serie del ESP32 y la cámara
CSI, corre la pila ROS2 (localización ArUco + EKF + nav2) y **reexpone a la GUI
`capbot-host` los mismos protocolos** (video H264/RTP, telemetría WS, comandos
UDP, navegación WS). No hay que tocar `capbot-host` ni `capbot-jetson-bridge`.

> La imagen es **arm64 / L4T** → se **construye y corre en la Jetson** (o en x86
> con `docker buildx` + qemu, mucho más lento). Foxy va compilado desde fuente
> sobre la base dusty-nv L4T; Python del contenedor = 3.6.

---

## 1. Preparación de la Jetson (una vez)

```bash
# a) Runtime nvidia por defecto (CUDA + cámara CSI dentro del contenedor)
sudo tee /etc/docker/daemon.json >/dev/null <<'EOF'
{ "default-runtime": "nvidia",
  "runtimes": { "nvidia": { "path": "nvidia-container-runtime", "runtimeArgs": [] } } }
EOF
sudo systemctl restart docker

# b) Liberar el serie /dev/ttyTHS1 (la consola serie lo ocupa por defecto)
sudo systemctl stop nvgetty && sudo systemctl disable nvgetty
sudo usermod -aG dialout $USER     # re-login despues

# c) Confirmar JetPack/L4T (define el tag de la imagen base)
cat /etc/nv_tegra_release          # ej: "R32 (release), REVISION: 7.1" -> r32.7.1
```

---

## 2. Build

```bash
cd capbot-ros-foxy

# Fases 1-2 (cámara + ArUco + EKF + serie + teleop): SIN nav2, build rápido.
L4T_TAG=r32.7.1 docker compose -f docker/docker-compose.yml build

# Fase 3 (con nav2 desde fuente): build LARGO -> activa swap antes.
BUILD_NAV2=1 docker compose -f docker/docker-compose.yml build
```

> **Swap para el build de nav2** (Nano 4 GB):
> ```bash
> sudo fallocate -l 6G /swapfile && sudo chmod 600 /swapfile
> sudo mkswap /swapfile && sudo swapon /swapfile
> ```

---

## 3. Run

```bash
# HOST_IP = IP del PC donde corre capbot-host (destino del video).
HOST_IP=192.168.1.10 docker compose -f docker/docker-compose.yml up

# Solo localización (sin nav2), aunque la imagen traiga nav2:
HOST_IP=192.168.1.10 docker compose -f docker/docker-compose.yml run --rm test_bot \
    ros2 launch test_bot real_robot.launch.py enable_nav:=false

# Shell de depuracion (ROS2 + workspace ya sourceados):
docker compose -f docker/docker-compose.yml run --rm test_bot bash
```

---

## 4. Qué funciona en cada fase

| Fase | Qué corre | Verificación |
|------|-----------|--------------|
| 1 | cámara CSI (`tee`: ArUco + video a GUI) + `aruco_localizer` | GUI muestra video; `ros2 topic hz /camera/image_raw`; `ros2 topic echo /aruco_pose` |
| 2 | + `esp32_serial_bridge` (/odom) + EKF + `teleop_gateway` + `cmd_mux` | joystick de la GUI mueve el robot (manual); `/odom`; TF `map→odom→base_link` |
| 3 | + nav2 + `gui_bridge_node` | goal desde la GUI/RViz → plan + `/cmd_vel` |
| 4 | (firmware `VEL_CMD`, repo capbot-ESP32) | nav2 cierra el lazo: el robot navega solo |

---

## 5. Notas / problemas comunes

- **`capbot-jetson-bridge` y este contenedor NO se corren a la vez**: pelean por
  el mismo serie y la misma cámara CSI. Autónomo = este contenedor; teleop puro
  desde PC = `main.py` del bridge. La red Jetson↔PC sí coexiste siempre.
- **Cámara CSI no abre**: revisar `ls /tmp/argus_socket` (montado), `runtime: nvidia`
  activo, y `gst-inspect-1.0 nvarguscamerasrc` dentro del contenedor.
- **`cv2.aruco` ausente**: el `pip install opencv-contrib-python` del Dockerfile
  debe traer wheel aarch64/cp36; si falla, instalar una OpenCV-contrib del sistema.
- **Permiso del serie**: el usuario del contenedor debe poder abrir `/dev/ttyTHS1`
  (grupo `dialout`).
- **nav2 pesado**: si el build OOM-killea, baja a `MAKEFLAGS=-j1` en el Dockerfile.
