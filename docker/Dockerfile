# =============================================================================
# capbot-ros-foxy — ROS2 Foxy + nav2 en Jetson Nano (servicio unico)
# =============================================================================
# La Jetson Nano (JetPack 4.x) es Ubuntu 18.04 / L4T r32 y NO tiene debs Foxy.
# Usamos las imagenes dusty-nv que traen ROS2 Foxy COMPILADO DESDE FUENTE sobre
# la base L4T (asi la camara CSI/argus y CUDA funcionan nativo). Python = 3.6.
#
# OJO: como Foxy va desde fuente, NO hay debs `ros-foxy-*` por apt. Lo que no
# venga en la variante `ros-base` (xacro, joint_state_publisher, robot_localization,
# nav2...) se compila desde fuente en un underlay.
#
# IMPORTANTE: imagen arm64/L4T -> se CONSTRUYE Y CORRE EN LA JETSON
# (o en x86 via `docker buildx` + qemu, mucho mas lento).
# Contexto de build = RAIZ del repo (capbot-ros-foxy). Ajusta L4T_TAG al JetPack:
#   cat /etc/nv_tegra_release
#   r32.7.1 -> JetPack 4.6.1  |  r32.6.1 -> JetPack 4.6  |  r32.5.0 -> JetPack 4.5
# =============================================================================
ARG L4T_TAG=r32.7.1
FROM dustynv/ros:foxy-ros-base-l4t-${L4T_TAG}

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=foxy

# Helper para sourcear ROS (dusty-nv usa install/ o setup.bash segun la imagen).
RUN echo 'if [ -f /opt/ros/${ROS_DISTRO}/install/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/install/setup.bash; else source /opt/ros/${ROS_DISTRO}/setup.bash; fi' > /etc/profile.d/ros_source.sh

# El repo apt de ROS (foxy/bionic, EOL) viene con la llave de firma VENCIDA y su
# InRelease ya no valida con la llave renovada -> rompe `apt-get update`. NO
# usamos debs de ROS (Foxy ya esta desde fuente en la base), asi que DESACTIVAMOS
# ese repo. colcon (lo unico de ROS que necesitamos) se toma desde pip mas abajo.
RUN rm -f /etc/apt/sources.list.d/ros2.list \
          /etc/apt/sources.list.d/ros2-latest.list \
          /etc/apt/sources.list.d/ros-latest.list 2>/dev/null; \
    for f in $(grep -rl 'packages.ros.org' /etc/apt/ 2>/dev/null); do \
        sed -i '/packages.ros.org/d' "$f"; done; \
    echo "repo apt de ROS desactivado"

# -----------------------------------------------------------------------------
# 1) Dependencias de sistema (NO-ROS)
#    - GStreamer + PyGObject: la captura CSI va por gi.repository.Gst (NO cv2),
#      evitando el conflicto aruco-vs-gstreamer. nvarguscamerasrc / nvv4l2h264enc
#      vienen del stack L4T del host (montar /tmp/argus_socket + runtime nvidia).
#    - colcon/git/vcstool para compilar los paquetes ROS que faltan desde fuente.
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-gi \
        python3-gi-cairo \
        gir1.2-gstreamer-1.0 \
        gir1.2-gst-plugins-base-1.0 \
        gstreamer1.0-tools \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        libgstreamer1.0-0 \
        libgstreamer-plugins-base1.0-0 \
        python3-numpy \
        python3-yaml \
        git \
        curl ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# colcon (lo necesita el `colcon build`). Suele venir en la base dusty-nv; si no,
# se instala por pip (incluye colcon-ros para paquetes ament_cmake).
RUN command -v colcon >/dev/null 2>&1 || \
    pip3 install --no-cache-dir colcon-common-extensions

# -----------------------------------------------------------------------------
# 2) OpenCV-contrib (cv2.aruco) por pip — TOLERANTE A FALLO
#    NO necesita GStreamer (la captura va por PyGObject), asi que la wheel de
#    PyPI sirve. 4.6.0.66 es la ultima con soporte cp36. Si ya hay aruco en la
#    base, se omite; si la wheel no existe, NO se rompe el build (la camara/video
#    igual funcionan; aruco quedaria pendiente -> ver docker/README.md).
# -----------------------------------------------------------------------------
RUN python3 -c "import cv2, cv2.aruco" 2>/dev/null && echo "cv2.aruco ya presente en la base" || \
    pip3 install --no-cache-dir --only-binary=:all: "opencv-contrib-python==4.6.0.66" || \
    echo "WARN: no se pudo instalar opencv-contrib-python; instalar OpenCV-contrib manualmente (ver docker/README.md)"
RUN pip3 install --no-cache-dir "scipy==1.5.4" "websockets<10" || \
    echo "WARN: pip scipy/websockets fallo; revisar wheels cp36/aarch64"

# -----------------------------------------------------------------------------
# 3) Underlay: xacro (lo usa el launch via `import xacro`). Solo se compila si
#    NO viene ya en la base. Se fija el TAG 2.0.3 (release de Foxy): el HEAD de la
#    rama `ros2` es para distros nuevas y su CMake usa `SCRIPTS_DESTINATION`, que
#    el ament de Foxy no soporta.
#    joint_state_publisher NO hace falta: los frames que usa ArUco
#    (base_link -> camera_link_optical) son joints FIJOS y robot_state_publisher
#    los publica en /tf_static sin /joint_states.
# -----------------------------------------------------------------------------
ENV DEPS_WS=/opt/deps_ws
RUN if python3 -c "import xacro" 2>/dev/null; then \
        echo "xacro ya presente en la base"; \
    else \
        mkdir -p ${DEPS_WS}/src && cd ${DEPS_WS}/src && \
        git clone --depth 1 -b 2.0.3 https://github.com/ros/xacro.git && \
        cd ${DEPS_WS} && source /etc/profile.d/ros_source.sh && \
        colcon build --packages-select xacro \
            --cmake-args -DCMAKE_BUILD_TYPE=Release && \
        rm -rf ${DEPS_WS}/build ${DEPS_WS}/log ${DEPS_WS}/src/*/.git; \
    fi

# -----------------------------------------------------------------------------
# 4) (OPCIONAL, PESADO) nav2 Foxy desde fuente — Fase 3
#    Desactivado por defecto para iterar Fases 1-2 rapido. Activar con:
#       --build-arg BUILD_NAV2=1     (usa swap en la Nano; build largo)
# -----------------------------------------------------------------------------
ARG BUILD_NAV2=0
ENV NAV2_WS=/opt/nav2_ws
COPY nav2 /tmp/nav2_recipe
RUN if [ "$BUILD_NAV2" = "1" ]; then set -e; \
        apt-get update && apt-get install -y --no-install-recommends \
            libasio-dev libtinyxml2-dev libzmq3-dev libgraphicsmagick++1-dev \
            libyaml-cpp-dev libeigen3-dev wget; \
        pip3 install --no-cache-dir vcstool rosdep; \
        source /etc/profile.d/ros_source.sh; \
        [ -f "${DEPS_WS}/install/setup.bash" ] && source ${DEPS_WS}/install/setup.bash; \
        mkdir -p ${NAV2_WS}/src && cd ${NAV2_WS}; \
        vcs import src < /tmp/nav2_recipe/nav2.repos; \
        for p in smac_planner nav2_smac_planner nav2_rviz_plugins nav2_system_tests nav2_amcl; do \
            d="$(find src -maxdepth 3 -type d -name "$p" | head -n1)"; [ -n "$d" ] && touch "$d/COLCON_IGNORE"; \
        done; \
        (rosdep init || true) && rosdep update || true; \
        rosdep install --from-paths src --ignore-src -r -y \
            --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers libopensplice67 libopensplice69 behaviortree_cpp_v3 ompl gazebo_ros_pkgs gazebo_ros gazebo_dev gazebo_plugins slam_toolbox nav2_rviz_plugins nav2_smac_planner nav2_amcl nav2_system_tests" || true; \
        MAKEFLAGS="-j2" colcon build \
            --cmake-args -DCMAKE_BUILD_TYPE=Release --no-warn-unused-cli; \
        rm -rf ${NAV2_WS}/build ${NAV2_WS}/log ${NAV2_WS}/src; \
        rm -rf /var/lib/apt/lists/*; \
    else echo "nav2 NO compilado (BUILD_NAV2=0). Fase 3: reconstruir con --build-arg BUILD_NAV2=1"; fi

# pyserial para esp32_serial_bridge (Fase 2). Va DESPUES de nav2 para no
# invalidar su cache (solo runtime; colcon no lo necesita en build).
RUN pip3 install --no-cache-dir pyserial || true

# robot_localization (EKF, Fase 2) desde fuente — Foxy no tiene debs.
# Va DESPUES de nav2 para no invalidar su cache. Solo se compilan
# robot_localization + sus deps que faltan en ros-base (diagnostic_updater,
# geographic_msgs); el resto (eigen, geographiclib, boost) es apt.
ENV RL_WS=/opt/rl_ws
# Libs de sistema explicitas (sin rosdep, que depende de red y falla por DNS):
# robot_localization -> eigen, geographiclib, boost; el resto de deps son ROS
# (ros-base + diagnostic_updater/geographic_msgs clonados). colcon compila offline.
RUN set -e; \
    apt-get update && apt-get install -y --no-install-recommends \
        libgeographic-dev libeigen3-dev libyaml-cpp-dev libboost-dev; \
    mkdir -p ${RL_WS}/src && cd ${RL_WS}/src && \
    git clone --depth 1 -b foxy-devel https://github.com/cra-ros-pkg/robot_localization.git && \
    git clone --depth 1 -b ros2 https://github.com/ros/diagnostics.git && \
    git clone --depth 1 -b 1.0.4 https://github.com/ros-geographic-info/geographic_info.git && \
    cd ${RL_WS} && source /etc/profile.d/ros_source.sh && \
    MAKEFLAGS="-j2" colcon build --packages-up-to robot_localization \
        --cmake-args -DCMAKE_BUILD_TYPE=Release --no-warn-unused-cli; \
    rm -rf ${RL_WS}/build ${RL_WS}/log ${RL_WS}/src && \
    rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# 4b) jetson-inference (detectNet / TensorRT) — deteccion de obstaculos en GPU
#     Requiere "default-runtime": "nvidia" en /etc/docker/daemon.json del host
#     (CUDA visible en build). BUILD_INTERACTIVE=NO descarga los modelos por
#     defecto (incluye SSD-Mobilenet-v2) sin el menu interactivo.
# -----------------------------------------------------------------------------
ENV JI_DIR=/opt/jetson-inference
RUN set -e; \
    apt-get update && apt-get install -y --no-install-recommends \
        cmake libpython3-dev python3-numpy; \
    git clone --recursive https://github.com/dusty-nv/jetson-inference ${JI_DIR}; \
    cd ${JI_DIR} && mkdir build && cd build; \
    cmake -DBUILD_INTERACTIVE=NO ../; \
    make -j2 && make install && ldconfig; \
    rm -rf ${JI_DIR}/build /var/lib/apt/lists/*

# Pre-construir el engine de TensorRT EN LA IMAGEN (evita los ~3-5 min de la
# primera corrida y que el .engine se pierda al recrear el contenedor).
# Tolerante a fallo: si no hay GPU en build, se genera en el primer arranque.
RUN python3 -c "from jetson_inference import detectNet; detectNet('ssd-mobilenet-v2', threshold=0.5)" \
    || python3 -c "from jetson.inference import detectNet; detectNet('ssd-mobilenet-v2', threshold=0.5)" \
    || echo "WARN: engine TensorRT no pre-generado; se construira en el primer arranque"

# -----------------------------------------------------------------------------
# 5) Workspace del paquete test_bot. Contexto de build = RAIZ del repo.
# -----------------------------------------------------------------------------
ENV WS=/ros_ws
WORKDIR ${WS}
COPY src/test_bot ${WS}/src/test_bot
RUN source /etc/profile.d/ros_source.sh && \
    if [ -f "${DEPS_WS}/install/setup.bash" ]; then source ${DEPS_WS}/install/setup.bash; fi && \
    if [ -d "${NAV2_WS}/install" ]; then source ${NAV2_WS}/install/setup.bash; fi && \
    MAKEFLAGS="-j2" colcon build --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Shells interactivas (p.ej. `docker exec -it capbot bash`) sourcean ROS+workspace.
RUN printf '%s\n' \
    'source /etc/profile.d/ros_source.sh' \
    '[ -f /opt/deps_ws/install/setup.bash ] && source /opt/deps_ws/install/setup.bash' \
    '[ -f /opt/rl_ws/install/setup.bash ] && source /opt/rl_ws/install/setup.bash' \
    '[ -f /opt/nav2_ws/install/setup.bash ] && source /opt/nav2_ws/install/setup.bash' \
    '[ -f /ros_ws/install/setup.bash ] && source /ros_ws/install/setup.bash' \
    >> /root/.bashrc

# -----------------------------------------------------------------------------
# 6) Entrypoint
# -----------------------------------------------------------------------------
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "test_bot", "real_robot.launch.py"]
