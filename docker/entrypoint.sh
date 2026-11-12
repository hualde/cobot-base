#!/usr/bin/env bash
# Carga ROS 2 y, si existe, el workspace compilado; después ejecuta el comando del contenedor.
set -e
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f /ws/install/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /ws/install/setup.bash
fi
exec "$@"
