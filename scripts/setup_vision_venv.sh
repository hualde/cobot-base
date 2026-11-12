#!/usr/bin/env bash
# Entorno Python aislado para visión (OpenCV, PyTorch con CUDA, anomalib).
# Se mantiene fuera del Python de ROS para no romper numpy/cv_bridge del sistema.
# Uso: ./scripts/setup_vision_venv.sh [ruta_venv]   (por defecto ~/venvs/vision)
set -euo pipefail
VENV="${1:-$HOME/venvs/vision}"
REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"

python3 -m venv "$VENV"
"$VENV/bin/pip" install --upgrade pip wheel
# Las ruedas de PyTorch en PyPI para Linux x86_64 ya incluyen CUDA.
"$VENV/bin/pip" install -r "$REPO_DIR/requirements-vision.txt"

"$VENV/bin/python" - <<'EOF'
import torch, cv2, anomalib
print(f"torch {torch.__version__} | CUDA disponible: {torch.cuda.is_available()}"
      + (f" ({torch.cuda.get_device_name(0)})" if torch.cuda.is_available() else ""))
print(f"opencv {cv2.__version__} | anomalib {anomalib.__version__}")
EOF
echo "Activa con: source $VENV/bin/activate"
