#!/usr/bin/env bash
# Comprueba host + imagen de desarrollo. No modifica nada.
set -uo pipefail
cd "$(dirname "$0")/.." || exit 1
ok=0; fail=0
check() { if eval "$2" >/dev/null 2>&1; then echo "  [OK]    $1"; ok=$((ok+1)); else echo "  [FALTA] $1"; fail=$((fail+1)); fi; }

echo "Host"
check "Ubuntu 24.04"                  "grep -q 'VERSION_ID=\"24.04\"' /etc/os-release"
check "GPU NVIDIA"                    "nvidia-smi"
check "docker sin sudo"               "docker info"
check "docker compose"                "docker compose version"
check "GPU en contenedores"           "docker info --format '{{json .Runtimes}}' | grep -q nvidia"
check "xhost"                         "command -v xhost"

echo "Isaac Sim (${ISAACSIM_ROOT:=$HOME/isaacsim/isaacsim/_build/linux-x86_64/release})"
check "isaac-sim.sh / python.sh"      "test -x $ISAACSIM_ROOT/isaac-sim.sh -a -x $ISAACSIM_ROOT/python.sh"
check "RAM >= 32 GB (mínimo NVIDIA)"  "test \$(awk '/MemTotal/{print int(\$2/1048576+0.5)}' /proc/meminfo) -ge 32"
check "VRAM >= 16 GB (mínimo NVIDIA)" "test \$(nvidia-smi --query-gpu=memory.total --format=csv,noheader,nounits | head -1) -ge 16000"

echo "Imagen de desarrollo (cobot-base:dev)"
check "imagen construida"             "docker image inspect cobot-base:dev"
for p in ur_robot_driver ur_moveit_config ur_description moveit_ros_move_group controller_manager joint_trajectory_controller rviz2; do
  check "paquete $p"                  "docker run --rm cobot-base:dev ros2 pkg prefix $p"
done
check "GPU visible en el contenedor"  "docker run --rm --gpus all cobot-base:dev nvidia-smi"

echo "Visión (venv ${VISION_VENV:=$HOME/venvs/vision}, opcional hasta el proyecto 4)"
check "torch + CUDA"                  "$VISION_VENV/bin/python -c 'import torch; assert torch.cuda.is_available()'"
check "anomalib"                      "$VISION_VENV/bin/python -c 'import anomalib'"

echo
echo "$ok OK, $fail pendientes"
exit $(( fail > 0 ))
