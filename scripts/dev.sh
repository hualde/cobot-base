#!/usr/bin/env bash
# Abre una terminal dentro del contenedor de desarrollo (lo construye y arranca si hace falta).
# Uso: ./scripts/dev.sh            -> bash dentro del contenedor
#      ./scripts/dev.sh <comando>  -> ejecuta el comando dentro (p. ej. ./scripts/dev.sh colcon build)
set -euo pipefail
cd "$(dirname "$0")/.."

# .env con tu UID/GID, para que los ficheros generados dentro sean tuyos.
if [[ ! -f .env ]]; then
  sed -e "s/^USER_UID=.*/USER_UID=$(id -u)/" -e "s/^USER_GID=.*/USER_GID=$(id -g)/" .env.example > .env
  echo "Creado .env (revísalo si trabajas con el robot real)"
fi

# Permite que tu usuario (el mismo UID que el del contenedor) dibuje en la pantalla: RViz, rqt.
xhost "+SI:localuser:$(id -un)" >/dev/null 2>&1 || echo "Aviso: xhost no disponible; RViz no podrá abrir ventana"

docker compose up -d --build dev
if [[ $# -eq 0 ]]; then
  exec docker compose exec dev bash
else
  exec docker compose exec dev bash -ic "$*"
fi
