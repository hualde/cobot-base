#!/usr/bin/env bash
# Crea una versión entregable: imagen runtime probada + paquete para el cliente.
# Uso: ./scripts/release.sh 0.1.0
#
# Resultado en dist/<versión>/:
#   cobot-base_<versión>.tar.gz   imagen Docker (para instalar sin internet: docker load)
#   compose.yaml, .env.example    lo que va en el PC de la célula
set -euo pipefail
VERSION="${1:?Uso: $0 <versión>, p. ej. 0.1.0}"
cd "$(dirname "$0")/.."
OUT="dist/$VERSION"

echo "==> Tests (etapa test del Dockerfile). Si fallan, no se genera la versión."
docker build -f docker/Dockerfile --target test .

echo "==> Imagen runtime cobot-base:$VERSION"
docker build -f docker/Dockerfile --target runtime \
  --label "org.opencontainers.image.version=$VERSION" \
  --label "org.opencontainers.image.revision=$(git rev-parse --short HEAD 2>/dev/null || echo sin-git)" \
  -t "cobot-base:$VERSION" .

echo "==> Paquete en $OUT"
mkdir -p "$OUT"
docker save "cobot-base:$VERSION" | gzip > "$OUT/cobot-base_$VERSION.tar.gz"
cp deploy/compose.yaml "$OUT/"
sed "s/^VERSION=.*/VERSION=$VERSION           # versión entregada/" deploy/.env.example > "$OUT/.env.example"
(cd "$OUT" && sha256sum ./*.tar.gz > SHA256SUMS)
ls -lh "$OUT"
