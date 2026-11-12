#!/usr/bin/env bash
# Prepara el HOST. Con el modelo de contenedores, el host solo necesita:
#   driver NVIDIA (ya instalado) + Docker + NVIDIA Container Toolkit + xhost (para RViz desde el contenedor).
# ROS 2, MoveIt 2 y los paquetes UR van dentro de la imagen Docker, no aquí.
# Uso: sudo ./scripts/install_host.sh
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "Ejecuta con sudo: sudo $0" >&2
  exit 1
fi
TARGET_USER="${SUDO_USER:-$USER}"

echo "==> Docker (paquetes de Ubuntu) + xhost"
apt-get update
apt-get install -y docker.io docker-compose-v2 docker-buildx x11-xserver-utils python3-venv curl git
systemctl enable --now docker
usermod -aG docker "$TARGET_USER"

echo "==> NVIDIA Container Toolkit (GPU dentro de los contenedores: RViz acelerado, visión con CUDA)"
if ! command -v nvidia-ctk >/dev/null; then
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
    | gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
  curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
    | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
    > /etc/apt/sources.list.d/nvidia-container-toolkit.list
  apt-get update
  apt-get install -y nvidia-container-toolkit
fi
nvidia-ctk runtime configure --runtime=docker
systemctl restart docker

echo
echo "Listo. Cierra sesión y vuelve a entrar (o 'newgrp docker') para usar docker sin sudo."
echo "Después: ./scripts/dev.sh   (construye la imagen de desarrollo y abre una terminal dentro)"
