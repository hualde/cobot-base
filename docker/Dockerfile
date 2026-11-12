# syntax=docker/dockerfile:1
#
# Un único Dockerfile con varias etapas (multi-stage). Cada etapa es una imagen distinta para un uso distinto:
#
#   manifests  -> solo los package.xml (truco de caché, no se usa directamente)
#   deps       -> ROS 2 + dependencias de EJECUCIÓN del workspace (UR, MoveIt 2, ros2_control...)
#   dev        -> deps + herramientas de desarrollo + usuario con tu UID      (docker compose / Dev Container)
#   build      -> compila el workspace                                          (intermedia)
#   test       -> ejecuta los tests; si fallan, falla el build                  (CI)
#   runtime    -> deps + workspace compilado, sin compiladores ni código fuente (entrega al cliente / PC de la célula)
#
# Construir una etapa concreta:  docker build -f docker/Dockerfile --target <etapa> .

ARG ROS_DISTRO=jazzy

# ---------------------------------------------------------------------------------------------------------------
# manifests: copia src/ y borra todo menos los package.xml.
# Así la capa de dependencias (lenta) solo se reconstruye cuando cambia un package.xml, no con cada cambio de código.
FROM ros:${ROS_DISTRO}-ros-base AS manifests
COPY src /tmp/src
RUN find /tmp/src -type f ! -name package.xml -delete

# ---------------------------------------------------------------------------------------------------------------
FROM ros:${ROS_DISTRO}-ros-base AS deps
ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

COPY --from=manifests /tmp/src /tmp/manifests/src
RUN apt-get update \
 && rosdep update --rosdistro ${ROS_DISTRO} \
 && rosdep install --from-paths /tmp/manifests/src --ignore-src -y --rosdistro ${ROS_DISTRO} \
      --dependency-types exec \
 && rm -rf /var/lib/apt/lists/*

# Cualquier shell interactiva (también `docker compose exec`, que se salta el ENTRYPOINT) tendrá ROS cargado.
RUN printf '%s\n' \
      "source /opt/ros/${ROS_DISTRO}/setup.bash" \
      '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' \
      >> /etc/bash.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
WORKDIR /ws

# ---------------------------------------------------------------------------------------------------------------
FROM deps AS dev
ARG ROS_DISTRO
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

COPY --from=manifests /tmp/src /tmp/manifests/src
RUN apt-get update \
 && rosdep install --from-paths /tmp/manifests/src --ignore-src -y --rosdistro ${ROS_DISTRO} \
      --dependency-types build --dependency-types buildtool --dependency-types test \
 && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions python3-pytest \
      ros-${ROS_DISTRO}-rviz2 ros-${ROS_DISTRO}-rqt-graph ros-${ROS_DISTRO}-rqt-image-view \
      ros-${ROS_DISTRO}-ros2controlcli \
      git gdb less nano bash-completion sudo x11-apps \
 && rm -rf /var/lib/apt/lists/*

# Usuario con el MISMO UID/GID que en el host: los build/ install/ log/ que se crean en la carpeta montada
# son tuyos y no de root. La imagen base de Ubuntu 24.04 trae un usuario "ubuntu" con UID 1000: se elimina.
RUN if id -u ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi \
 && (getent group ${USER_GID} >/dev/null || groupadd --gid ${USER_GID} ${USERNAME}) \
 && useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USERNAME} \
 && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME}
USER ${USERNAME}
CMD ["sleep", "infinity"]

# ---------------------------------------------------------------------------------------------------------------
FROM deps AS build
ARG ROS_DISTRO
COPY --from=manifests /tmp/src /tmp/manifests/src
RUN apt-get update \
 && rosdep install --from-paths /tmp/manifests/src --ignore-src -y --rosdistro ${ROS_DISTRO} \
      --dependency-types build --dependency-types buildtool --dependency-types test \
 && apt-get install -y --no-install-recommends python3-colcon-common-extensions python3-pytest \
 && rm -rf /var/lib/apt/lists/*
COPY src /ws/src
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && colcon build --event-handlers console_cohesion+

# ---------------------------------------------------------------------------------------------------------------
FROM build AS test
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && colcon test --event-handlers console_cohesion+ \
 && colcon test-result --verbose

# ---------------------------------------------------------------------------------------------------------------
FROM deps AS runtime
COPY --from=build /ws/install /ws/install
CMD ["ros2", "launch", "cobot_bringup", "ursim_moveit.launch.py", "rviz:=false"]
