#!/usr/bin/env bash
# Comprueba la instalación LOCAL de Isaac Sim (no instala nada): smoke test de ros2_control headless.
# Isaac Sim corre en el host y usa sus librerías ROS 2 Jazzy internas: no hace falta hacer source de ROS.
# Uso: ISAACSIM_ROOT=<carpeta con isaac-sim.sh y python.sh> ./scripts/setup_isaacsim.sh
set -euo pipefail

ISAACSIM_ROOT="${ISAACSIM_ROOT:-$HOME/isaacsim/isaacsim/_build/linux-x86_64/release}"
[[ -x "$ISAACSIM_ROOT/python.sh" ]] || { echo "No encuentro $ISAACSIM_ROOT/python.sh; ajusta ISAACSIM_ROOT" >&2; exit 1; }
echo "Isaac Sim: $ISAACSIM_ROOT ($(cat "$ISAACSIM_ROOT/../../../VERSION" 2>/dev/null || echo 'versión desconocida'))"

cd "$ISAACSIM_ROOT"
./python.sh standalone_examples/api/isaacsim.ros2.control/ros2_control_smoke_test.py --headless
