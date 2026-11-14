# cobot-base

## 1. Descripción del proyecto

### ¿Qué es?

**cobot-base** es la base sobre la que se construyen nuestros proyectos de robótica para clientes.
Contiene un **brazo robótico Universal Robots (UR5e) con una cámara en la muñeca**, que se puede:

- **mover desde el ordenador**: le dices a qué posición ir y el software calcula el movimiento sin chocar;
- **probar en simulación**, sin tener el robot delante;
- **instalar en la fábrica del cliente** para controlar el robot real, sin cambiar el programa.

Hoy el proyecto hace lo básico: mover el brazo y ver lo que ve la cámara. Cada proyecto nuevo (inspección de
piezas, coger y dejar objetos...) parte de una copia de este y le añade su propia lógica.

### ¿Qué problema resuelve?

Programar un robot industrial tiene tres dificultades:

1. **El robot no siempre está disponible** (está en la fábrica, es caro, es peligroso probar con él).
   → Usamos **simuladores**: uno que imita el controlador del robot y otro que imita el mundo en 3D.
2. **El software depende de muchas piezas instaladas** (ROS 2, MoveIt, driver del robot...) y en cada ordenador
   pueden estar en versiones distintas.
   → Usamos **Docker**, que mete todo el software en un paquete cerrado que funciona igual en cualquier PC.
3. **Instalar en el cliente suele ser largo y propenso a errores.**
   → Al cliente le entregamos ese paquete ya probado: se copia y se arranca.

### Vocabulario básico

Si es tu primera vez con este software, estos son los términos que vas a ver:

| Término | Qué es, en una frase |
|---|---|
| **Cobot** | Robot colaborativo: brazo industrial pensado para trabajar cerca de personas. Usamos los de **Universal Robots (UR)**. |
| **ROS 2** | El "sistema operativo" de los robots: un conjunto de programas que se comunican enviándose mensajes (posición del brazo, imágenes de la cámara...). Usamos la versión **Jazzy**. |
| **Nodo** | Un programa de ROS 2. Por ejemplo: el que habla con el robot, el que planifica movimientos, el que muestra la cámara. |
| **Tópico** | Un "canal" por el que los nodos se envían mensajes. Ejemplo: `/wrist_camera/image` lleva las imágenes de la cámara. |
| **Paquete** | Una carpeta con código ROS 2 que se compila junta. Nuestro paquete se llama `cobot_bringup`. |
| **Launch** | Un fichero que arranca varios nodos a la vez con un solo comando. |
| **MoveIt 2** | El planificador de movimientos: calcula cómo llevar el brazo de un punto a otro sin chocar. |
| **ros2_control** | La pieza que envía las órdenes de movimiento a los motores (reales o simulados). |
| **RViz** | Ventana de visualización de ROS 2: ves el robot en 3D y le indicas a dónde moverse. |
| **URDF** | Fichero que describe la forma del robot: piezas, articulaciones, medidas y dónde está la cámara. |
| **Driver UR** | El programa que conecta ROS 2 con el controlador real del robot UR. |
| **URSim** | Simulador oficial de Universal Robots: el mismo software que lleva el armario del robot, pero en tu PC. |
| **Isaac Sim** | Simulador 3D de NVIDIA: dibuja el robot, el almacén, las piezas y simula la cámara y la física. |
| **Docker** | Herramienta para empaquetar software con todo lo que necesita y ejecutarlo aislado. |
| **Imagen (Docker)** | El paquete cerrado: como un "instalador" que ya trae todo dentro. |
| **Contenedor** | Una imagen en marcha. Puedes tener varios contenedores a la vez. |
| **Host** | Tu ordenador "de verdad", el que ejecuta los contenedores. |

### ¿Cómo encajan las piezas?

```
     TÚ  (le indicas a dónde mover el brazo, en RViz)
      │
      ▼
   MoveIt 2  ── calcula el movimiento ──►  ros2_control  ── envía la orden ──►  ROBOT
                                                                              │
                                                     ┌────────────────────────┼─────────────────────┐
                                                     ▼                        ▼                     ▼
                                              Isaac Sim (3D)          URSim (controlador     Robot UR real
                                              en tu PC                simulado) en tu PC     en la fábrica
```

El programa es **el mismo** en los tres casos. Solo cambia a dónde se envían las órdenes.

---

## 2. Estructura del proyecto

```
cobot-base/
│
├── README.md                     Este documento.
│
├── src/                          ★ EL CÓDIGO DEL ROBOT (paquetes ROS 2)
│   └── cobot_bringup/            Nuestro paquete. "Bringup" = arrancar el robot.
│       ├── package.xml           Ficha del paquete: nombre y LISTA DE DEPENDENCIAS (qué otros paquetes necesita).
│       ├── CMakeLists.txt        Instrucciones de compilación (qué carpetas se instalan).
│       ├── launch/               Ficheros para arrancar todo con un comando:
│       │   ├── isaac_moveit.launch.py     → mover el robot simulado en Isaac Sim
│       │   └── ur_moveit.launch.py        → mover el robot de URSim o el robot real
│       ├── urdf/
│       │   └── ur_wrist_camera.urdf.xacro → forma del robot + cámara montada en la muñeca
│       ├── config/
│       │   └── isaac_controllers.yaml     → qué controladores de movimiento usa el robot en Isaac Sim
│       └── test/
│           └── test_config.py             → pruebas automáticas (el modelo es correcto, la configuración cuadra)
│
├── isaac/                        ★ LA SIMULACIÓN 3D
│   └── ur_inspection_cell.py     Crea la escena en Isaac Sim: almacén, robot, mesa con piezas y cámara.
│
├── docker/                       ★ EL EMPAQUETADO
│   ├── Dockerfile                Receta para construir las imágenes (qué se instala dentro).
│   └── entrypoint.sh             Lo primero que se ejecuta al arrancar un contenedor (prepara ROS 2).
│
├── compose.yaml                  Describe los contenedores que usas EN TU PC: "dev" (para programar) y "ursim".
├── .env.example                  Plantilla de configuración de tu PC (se copia a .env automáticamente).
│
├── deploy/                       ★ LO QUE SE INSTALA EN EL CLIENTE
│   ├── compose.yaml              Describe el contenedor que funciona EN LA FÁBRICA.
│   └── .env.example              Plantilla de configuración de la célula: IP del robot, modelo, versión.
│
├── scripts/                      ★ COMANDOS LISTOS PARA USAR (se ejecutan en tu PC)
│   ├── install_host.sh           Instala Docker en tu PC. Solo la primera vez.
│   ├── dev.sh                    Abre una terminal DENTRO del contenedor de desarrollo.
│   ├── isaac_cell.sh             Abre Isaac Sim con la escena.
│   ├── ursim_up.sh               Arranca URSim y enciende el robot simulado.
│   ├── release.sh                Prepara una versión para entregar al cliente.
│   ├── verify_env.sh             Comprueba que tienes todo instalado.
│   ├── setup_isaacsim.sh         Prueba rápida de que Isaac Sim funciona.
│   └── setup_vision_venv.sh      Prepara herramientas de visión artificial (para proyectos futuros).
│
├── .devcontainer/                Para trabajar desde VS Code dentro del contenedor (opcional).
├── .github/workflows/ci.yml      Pruebas automáticas cada vez que se sube un cambio a GitHub.
├── .gitignore / .dockerignore    Ficheros que no se guardan en git / no se meten en la imagen.
└── requirements-vision.txt       Librerías de visión artificial (para proyectos futuros).
```

Carpetas que aparecen al compilar (no se guardan en git): `build/`, `install/`, `log/`. Y al crear una versión:
`dist/`.

**Resumen**: escribes código en `src/`, lo pruebas con `isaac/` o URSim, `docker/` lo empaqueta y `deploy/` es lo
que llevas al cliente. `scripts/` te ahorra recordar comandos.

---

## 3. Dónde se ejecuta cada cosa

Hay dos ordenadores con papeles muy distintos:

- **Tu ordenador (desarrollo):** donde escribes el código, lo pruebas en simulación y **fabricas la
  imagen** que se entrega. Solo se usa en la oficina.
- **El PC de la fábrica (producción):** solo **ejecuta** esa imagen. No tiene simuladores, ni RViz, ni
  código fuente, y no necesita tu ordenador para nada. Puedes irte y la célula sigue funcionando.

| Tarea | Dónde |
|---|---|
| Escribir y compilar el código | Tu ordenador (contenedor `dev`) |
| Ver el robot y probar (Isaac Sim, URSim, RViz) | Tu ordenador |
| Crear la versión para el cliente (`release.sh`) | Tu ordenador |
| Mover el robot real en producción | PC de la fábrica |

```
┌─────────────────────────── TU ORDENADOR (desarrollo) ─────────────────────────┐
│                                                                                │
│   Isaac Sim                 Contenedor "dev"                Contenedor "ursim" │
│   (instalado en tu PC)      (ROS 2, MoveIt, RViz,           (controlador UR    │
│   dibuja el robot     ◄───►  driver UR y tu código)  ◄────►  simulado)         │
│   y el almacén                                                                 │
└────────────────────────────────────────────────────────────────────────────────┘

┌────── PC DE LA FÁBRICA (producción) ─────┐   cable de red   ┌──────────────────┐
│   Contenedor "cobot"                     │ ◄──────────────► │  Robot UR real   │
│   (tu código ya compilado + ROS 2)       │                  │                  │
└──────────────────────────────────────────┘                  └──────────────────┘
```

- **Isaac Sim** se instala directamente en tu PC porque necesita la tarjeta gráfica y su propia ventana.
- **Todo lo de ROS 2** está dentro de contenedores. Así tu PC, el de un compañero y el del cliente tienen
  exactamente las mismas versiones.
- **En la fábrica** no hay simuladores: solo el contenedor con tu programa y el robot real.

---

## 4. Instalación en tu ordenador (solo la primera vez)

Necesitas: Ubuntu 24.04, una tarjeta gráfica NVIDIA e Isaac Sim 6.2 instalado.

```bash
cd cobot-base
sudo ./scripts/install_host.sh     # 1. Instala Docker
newgrp docker                      # 2. Te da permiso para usar Docker (o cierra sesión y vuelve a entrar)
./scripts/dev.sh                   # 3. Construye el contenedor de desarrollo (~10 min) y te deja dentro
```

Cuando estés dentro del contenedor verás algo como `dev@tu-pc:/ws$`. Ahí compila el proyecto:

```bash
colcon build --symlink-install     # compilar
colcon test                        # ejecutar las pruebas
colcon test-result --verbose       # ver el resultado
```

Para salir del contenedor: `exit`. Para volver a entrar: `./scripts/dev.sh`.

> **Isaac Sim y la tarjeta gráfica**: NVIDIA recomienda como mínimo una RTX 4080 (16 GB) y 32 GB de RAM.
> Con una RTX 4060 (8 GB) esta escena funciona, pero escenas más grandes pueden no caber.

---

## 5. Primeros pasos: mover el robot en Isaac Sim

Necesitas **dos terminales**.

**Terminal 1** (en tu PC) — abre Isaac Sim con la escena:

```bash
./scripts/isaac_cell.sh
```

Espera a que aparezca el almacén con el robot (la primera vez tarda varios minutos en descargar los modelos).

**Terminal 2** — entra al contenedor y arranca MoveIt:

```bash
./scripts/dev.sh
ros2 launch cobot_bringup isaac_moveit.launch.py ur_type:=ur5e
```

Se abre **RViz** con el robot. Para moverlo:

1. Arrastra la bola/flechas que aparecen en el extremo del brazo hasta otra posición.
2. En el panel *MotionPlanning*, pulsa **Plan & Execute**.
3. El robot se mueve en RViz **y** en Isaac Sim.

Para ver lo que ve la cámara de la muñeca (en otra terminal dentro del contenedor):

```bash
ros2 run rqt_image_view rqt_image_view /wrist_camera/image
```

Comandos útiles para explorar (dentro del contenedor):

```bash
ros2 topic list                    # ver todos los canales de mensajes
ros2 topic echo /joint_states      # ver la posición de las articulaciones en directo
ros2 topic hz /wrist_camera/image  # ver cuántas imágenes por segundo envía la cámara
```

Opciones de la escena: `./scripts/isaac_cell.sh --robot ur10e` (otro modelo de robot),
`--env warehouse_shelves` (almacén con estanterías), `--env none` (sin almacén).

Si cierras Isaac Sim y lo vuelves a abrir, vuelve a lanzar también el comando de la terminal 2.

---

## 6. Probar contra el controlador UR (URSim)

URSim es el software real del armario de control de Universal Robots. Probar contra él es lo más parecido a
tener el robot físico.

```bash
./scripts/ursim_up.sh                                  # en tu PC: arranca URSim, lo enciende y suelta frenos
./scripts/dev.sh                                       # entrar al contenedor
ros2 launch cobot_bringup ur_moveit.launch.py          # conectar ROS 2 al controlador + MoveIt + RViz
```

Puedes ver la pantalla del robot (la del teach pendant) en el navegador:
`http://192.168.56.101:6080/vnc.html`. Cuando muevas el robot desde RViz, también se moverá ahí.

---

## 7. Cómo trabajar en el día a día

| Quiero... | Cómo |
|---|---|
| Programar | Edita los ficheros de `src/` con tu editor normal, en tu PC. |
| Compilar | Dentro del contenedor: `colcon build --symlink-install` |
| Añadir una librería o paquete | Añádela a `src/cobot_bringup/package.xml` y reconstruye: `docker compose build dev` |
| Trabajar desde VS Code | Abre la carpeta y elige *Reopen in Container* |
| Parar los contenedores | `docker compose down` |

**Dos reglas importantes:**

1. **No instales nada a mano dentro del contenedor** (`apt install`...). Se pierde al reiniciarlo y el cliente no lo
   tendría. Todo lo que el proyecto necesita se apunta en `package.xml`.
2. **El código está en tu PC**, el contenedor solo lo "ve". Puedes editar fuera y compilar dentro: es la misma
   carpeta.

---

## 8. Despliegue en el cliente

El objetivo es que instalar en la fábrica sea **copiar una carpeta y ejecutar un comando**, sin compilar nada ni
instalar ROS 2.

### Visión general

```
   EN TU PC                                         EN LA FÁBRICA
   ────────                                         ─────────────
   1. Crear la versión          ──── USB ────►      3. Instalar la versión
      ./scripts/release.sh 0.1.0                       docker load ...
                                                    4. Configurar la célula
   2. Probarla contra URSim                            .env  (IP del robot)
                                                    5. Arrancar
                                                       docker compose up -d
                                                              │
                                                              ▼
                                                       Robot real funcionando
```

### Paso 1 — Crear la versión (en tu PC)

```bash
./scripts/release.sh 0.1.0
```

El script hace tres cosas:

1. **Ejecuta las pruebas.** Si alguna falla, se para: no se entrega nada que no pase las pruebas.
2. **Construye la imagen de entrega**: tu código ya compilado + ROS 2 + MoveIt + driver UR. Sin código fuente ni
   herramientas de desarrollo.
3. **Lo empaqueta todo** en la carpeta `dist/0.1.0/`:

```
dist/0.1.0/
├── cobot-base_0.1.0.tar.gz    la imagen (todo el software en un solo fichero)
├── compose.yaml               cómo se arranca
├── .env.example               configuración de la célula, para rellenar
└── SHA256SUMS                 para comprobar que el fichero se copió sin errores
```

El número (`0.1.0`) es la versión. Cada entrega lleva un número distinto; así se sabe qué hay instalado en cada
cliente y se puede volver atrás.

### Paso 2 — Probar la versión antes de ir (en tu PC)

```bash
./scripts/ursim_up.sh                              # arranca el robot simulado
cd dist/0.1.0
cp .env.example .env                               # en .env pon ROBOT_IP=192.168.56.101 (URSim)
docker load < cobot-base_0.1.0.tar.gz
docker compose up                                  # debe conectarse al robot simulado
```

Es exactamente lo que harás en la fábrica, pero contra el robot simulado.

### Paso 3 — Preparar el PC de la fábrica (una sola vez)

- Ubuntu 24.04 (o cualquier Linux).
- Docker: `sudo apt install docker.io docker-compose-v2`
- Conectado al robot por **cable de red**, con una IP fija en la misma red que el robot.
  Ejemplo: PC `192.168.1.20`, robot `192.168.1.10`.
- No necesita tarjeta gráfica, pantalla ni internet.

### Paso 4 — Preparar el robot (una sola vez, en el teach pendant)

1. **Red**: asignar una IP fija al robot (la misma que pondrás en `.env`).
2. **Control remoto**: *Ajustes → Sistema → Control remoto → Activar*, y poner el pendant en modo **Remoto**.
   Así el robot acepta órdenes del PC.
3. **Seguridad**: los límites de velocidad, zonas y paradas de emergencia se configuran en el propio robot según
   la evaluación de riesgos de la célula. **Nuestro software no sustituye a la seguridad del robot.**

### Paso 5 — Instalar y arrancar (en el PC de la fábrica)

```bash
# Copiar la carpeta 0.1.0/ al PC (por USB o red) y entrar en ella
sha256sum -c SHA256SUMS                  # 1. comprobar que la copia está bien
docker load < cobot-base_0.1.0.tar.gz     # 2. instalar la imagen
cp .env.example .env                     # 3. crear la configuración...
nano .env                                #    ...y poner ROBOT_IP y UR_TYPE de esta célula
docker compose run --rm calibrate        # 4. leer la calibración del robot (ver abajo)...
nano .env                                #    ...y poner KINEMATICS_FILE=/calibration/robot_calibration.yaml
docker compose up -d                     # 5. arrancar
```

A partir de ahora **el programa arranca solo cada vez que se enciende el PC**, y si se cae, se reinicia.

### Calibración del robot

Cada robot UR sale de fábrica con su propia calibración: las medidas reales de sus piezas, que difieren un poco
de las del modelo. Sin ella el robot funciona, pero las posiciones que calcula ROS 2 pueden desviarse algunos
milímetros. Por eso **en producción hay que usarla siempre**.

- `docker compose run --rm calibrate` se conecta al robot (que debe estar encendido) y guarda su calibración en
  `calibration/robot_calibration.yaml`, junto a `compose.yaml`.
- En `.env`, `KINEMATICS_FILE=/calibration/robot_calibration.yaml` hace que el programa la use.
  Con `KINEMATICS_FILE=nominal` se usan los valores genéricos del modelo (vale para URSim).
- La calibración es **de ese robot concreto**: si se cambia el robot, hay que repetirla. Guarda una copia junto al
  `.env` de ese cliente.

### Comprobar que funciona

```bash
docker compose ps          # el contenedor "cobot" debe aparecer como "running"
docker compose logs -f     # debe aparecer: "Robot connected to reverse interface"
```

### Actualizar a una versión nueva, o volver atrás

| Quiero... | Qué hago en el PC de la fábrica |
|---|---|
| Instalar la versión 0.2.0 | `docker load < cobot-base_0.2.0.tar.gz`, poner `VERSION=0.2.0` en `.env`, `docker compose up -d` |
| Volver a la 0.1.0 | Poner `VERSION=0.1.0` en `.env` y `docker compose up -d` (la versión anterior sigue guardada) |
| Parar el programa | `docker compose down` |
| Saber qué versión funciona | `docker compose images` |

---

## 9. Problemas frecuentes

| Problema | Solución |
|---|---|
| `permission denied ... docker.sock` | Tu usuario aún no puede usar Docker: `newgrp docker` o cierra sesión y vuelve a entrar. |
| RViz no se abre desde el contenedor | Usa siempre `./scripts/dev.sh` (da permiso a la pantalla). |
| El robot no se mueve en Isaac Sim | Comprueba que Isaac Sim está en marcha antes de lanzar `isaac_moveit.launch.py`. Si reiniciaste Isaac Sim, relanza el launch. |
| `ros2 topic list` no muestra los tópicos de Isaac Sim | Isaac Sim y el contenedor deben tener el mismo `ROS_DOMAIN_ID` (está en `.env`, por defecto 0). |
| Isaac Sim falla con `libtinyxml2.so.9` | Arráncalo siempre con `./scripts/isaac_cell.sh`, no directamente. |

---

## 10. Para saber más (detalles técnicos)

Estos ajustes ya están hechos; solo hace falta entenderlos si vas a modificar la configuración de Docker.

- **El contenedor comparte la red del PC** (`network_mode: host`): ROS 2 busca a los demás programas por la red; si
  el contenedor tuviera su propia red, no encontraría a Isaac Sim ni al robot.
- **El contenedor comparte la memoria del PC** (`ipc: host`): ROS 2 la usa para enviar imágenes rápido.
- **Mismo usuario dentro y fuera**: los ficheros que crea el contenedor en tu carpeta son tuyos, no del
  administrador.
- **Un solo Dockerfile, varias imágenes**: `dev` (para programar, con herramientas), `test` (ejecuta las pruebas) y
  `runtime` (la que se entrega, solo lo necesario).
- **`isaac_cell.sh`**: Isaac Sim trae su propia copia de ROS 2 y el script la activa antes de arrancar.

## 11. Empezar un proyecto nuevo con esta plantilla

1. Copia la carpeta entera y cambia el nombre.
2. Añade tu código en `src/`, **en un paquete propio** (por ejemplo `cobot_app`, creado con `ros2 pkg create`
   dentro del contenedor), no dentro de `cobot_bringup`. Así `cobot_bringup` sigue siendo el de la plantilla y
   puedes traer sus mejoras sin mezclarlas con la lógica del cliente.
3. Apunta siempre las dependencias en `package.xml`.
4. Para cada cliente, guarda su `deploy/.env` con la IP y la versión entregada.
