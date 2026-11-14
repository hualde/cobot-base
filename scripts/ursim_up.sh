#!/usr/bin/env bash
# Arranca URSim (contenedor "ursim" de compose.yaml), lo enciende y suelta frenos vía Dashboard Server (puerto 29999).
# No necesita ROS en el host. Polyscope (VNC web): http://192.168.56.101:6080/vnc.html
# Uso: ./scripts/ursim_up.sh
set -euo pipefail
cd "$(dirname "$0")/.."
IP=192.168.56.101

docker compose up -d ursim

dash() {  # envía un comando al Dashboard Server y devuelve la respuesta
  exec 3<>"/dev/tcp/$IP/29999"
  read -r -t 5 _banner <&3 || true
  printf '%s\n' "$1" >&3
  local reply=""
  read -r -t 10 reply <&3 || true
  exec 3>&-
  echo "$reply"
}

echo -n "Esperando al Dashboard Server (el primer arranque tarda ~1 min)"
for _ in $(seq 1 180); do
  if (exec 3<>"/dev/tcp/$IP/29999") 2>/dev/null; then echo " OK"; break; fi
  echo -n "."; sleep 1
done

dash "close popup" >/dev/null
echo "power on:      $(dash 'power on')"
for _ in $(seq 1 30); do
  mode="$(dash 'robotmode')"
  [[ "$mode" == *IDLE* || "$mode" == *RUNNING* ]] && break
  sleep 1
done
echo "brake release: $(dash 'brake release')"
for _ in $(seq 1 30); do
  [[ "$(dash 'robotmode')" == *RUNNING* ]] && break
  sleep 1
done
echo "estado:        $(dash 'robotmode')"
echo
echo "URSim listo en $IP. Siguiente, dentro del contenedor dev:"
echo "  ros2 launch cobot_bringup ur_moveit.launch.py"
