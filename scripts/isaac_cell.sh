#!/usr/bin/env bash
# Arranca la celda de Isaac Sim (isaac/ur_inspection_cell.py) en el HOST.
# Uso: ./scripts/isaac_cell.sh [--robot ur5e] [--headless] [--camera-hz 15]
#
# Por qué este envoltorio:
#  - Isaac Sim trae sus propias librerías ROS 2 Jazzy (rclcpp, ros2_control, controladores...).
#    setup_ros_env.sh las añade a LD_LIBRARY_PATH; sin eso el Controller Manager falla al cargar plugins
#    (p. ej. "libtinyxml2.so.9: cannot open shared object file").
#  - Esas librerías solo se usan si NO hay otro ROS cargado: por eso se limpia el entorno ROS del host.
#  - ROS_DOMAIN_ID se toma de .env para coincidir con los contenedores.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
ISAACSIM_ROOT="${ISAACSIM_ROOT:-$HOME/isaacsim/isaacsim/_build/linux-x86_64/release}"
[[ -x "$ISAACSIM_ROOT/python.sh" ]] || { echo "No encuentro $ISAACSIM_ROOT/python.sh; ajusta ISAACSIM_ROOT" >&2; exit 1; }

DOMAIN="$(grep -E '^ROS_DOMAIN_ID=' "$REPO/.env" 2>/dev/null | cut -d= -f2 | awk '{print $1}')"

exec env -u ROS_DISTRO -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u PYTHONPATH -u LD_LIBRARY_PATH \
  ROS_DOMAIN_ID="${DOMAIN:-0}" \
  bash -c 'source "$0/setup_ros_env.sh" && cd "$0" && exec ./python.sh "$1/isaac/ur_inspection_cell.py" "${@:2}"' \
  "$ISAACSIM_ROOT" "$REPO" "$@"
