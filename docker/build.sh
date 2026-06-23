#!/usr/bin/env bash
# Build de la imagen SIN docker-compose (util en Jetson JetPack 4.x, que no lo trae).
#
# Uso:
#   ./docker/build.sh                 # Fases 1-2 (sin nav2)
#   BUILD_NAV2=1 ./docker/build.sh    # Fase 3 (con nav2, build largo)
#   L4T_TAG=r32.6.1 ./docker/build.sh # otra version de JetPack
set -e

# Raiz del repo (un nivel arriba de docker/)
cd "$(dirname "$0")/.."

L4T_TAG="${L4T_TAG:-r32.7.1}"
BUILD_NAV2="${BUILD_NAV2:-0}"
IMAGE="${IMAGE:-capbot-ros-foxy:foxy-l4t}"

echo "Building ${IMAGE}  (L4T_TAG=${L4T_TAG}, BUILD_NAV2=${BUILD_NAV2})"
docker build -t "${IMAGE}" \
    --build-arg L4T_TAG="${L4T_TAG}" \
    --build-arg BUILD_NAV2="${BUILD_NAV2}" \
    -f docker/Dockerfile .
