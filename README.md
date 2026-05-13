# 🤖 CarryBot

Sistema de navegación autónoma para un robot **TurtleBot3 Burger** en un almacén simulado, construido con **ROS 2 Jazzy** y **Gazebo Harmonic**. El robot navega por el entorno, recoge y entrega paquetes evitando obstáculos, y puede controlarse tanto desde consola como desde una interfaz web React.

> **GTI 2026 · Grupo 03**  
> Emilio Sánchez Granado · Jaime Sánchez Martí · Endika Matute Blanco · Sandra Moll Cots · Rocío Piquer Bacete

---

## 📋 Índice

- [Funcionalidades](#-funcionalidades)
- [Arquitectura del sistema](#-arquitectura-del-sistema)
- [Requisitos previos](#-requisitos-previos)
- [Estructura del repositorio](#-estructura-del-repositorio)
- [Instalación](#-instalación)
- [Secuencia de lanzamiento](#-secuencia-de-lanzamiento)
- [Paquetes](#-paquetes)
- [Interfaz web](#-interfaz-web)
- [Destinos disponibles](#-destinos-disponibles)
- [Diagrama de comunicación entre nodos](#-diagrama-de-comunicación-entre-nodos)

---

## ✨ Funcionalidades

- 🗺️ **Navegación a punto concreto** — envía el robot a una estantería o al punto de carga por nombre
- 🔄 **Patrulla por sectores** — recorre una zona completa del almacén mediante waypoints en zigzag
- 📦 **Ruta de recogida automática** — procesa todos los pedidos de un JSON en orden sin intervención del usuario
- 🌐 **Panel de control web** — cámara, minimapa, joystick de teleoperación y botones de navegación
- 🛑 **Cancelación remota** — detiene cualquier goal de navegación activo desde la web
- 🔌 **Puente ROS 2 ↔ Web** — comunicación bidireccional en tiempo real mediante rosbridge WebSocket

---

## 🏗️ Arquitectura del sistema

El sistema está compuesto por **cinco paquetes ROS 2** y una aplicación web React independiente:

| Paquete | Tipo de build | Responsabilidad |
|---|---|---|
| `carrybot_mundo` | ament_cmake | Simulación Gazebo Harmonic (mundo warehouse + TurtleBot3) |
| `carrybot_nav2_punto` | ament_python | Nav2 + AMCL + mapa + navegación a un punto concreto |
| `carrybot_nav2_sector` | ament_python | Patrulla por sectores con múltiples waypoints |
| `carrybot_nav_recogida` | ament_python | Ruta fija automática de recogida desde pedidos JSON |
| `carrybot_web_bridge` | ament_python | Puente ROS 2 ↔ Web (rosbridge WebSocket) |
| `carrybot_web` | React/Vite | Interfaz web de control (fuera del workspace ROS 2) |

---

## 🔧 Requisitos previos

| Dependencia | Versión |
|---|---|
| ROS 2 | Jazzy |
| Gazebo | Harmonic |
| Nav2 | (incluido con ROS 2 Jazzy) |
| rosbridge_suite | `ros-jazzy-rosbridge-suite` |
| nav2_simple_commander | `ros-jazzy-nav2-simple-commander` |
| Node.js | ≥ 18 |
| Python | ≥ 3.10 |

```bash
# Instalación de dependencias ROS 2
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
                 ros-jazzy-rosbridge-suite ros-jazzy-nav2-simple-commander
```

---

## 📁 Estructura del repositorio

```
carrybot/
├── carrybot/                        # Meta-paquete raíz
├── carrybot_mundo/                  # Simulación Gazebo
│   ├── launch/                      # my_world, spawn_turtlebot3, robot_state_publisher
│   ├── worlds/warehouse.world       # Mundo SDF del almacén
│   ├── models/                      # Modelos SDF del TurtleBot3 y del almacén
│   └── urdf/                        # URDF del TurtleBot3 Burger
├── carrybot_nav2_punto/             # Navegación a punto concreto
│   ├── carrybot_nav2_punto/
│   │   ├── initial_pose_pub.py      # Publica la pose inicial para AMCL
│   │   └── nav_to_point.py          # Cliente de acción: navega a un destino por nombre
│   ├── config/locations.json        # Destinos con coordenadas (x, y, cuaternión)
│   ├── map/                         # Mapa del almacén (PGM + YAML)
│   ├── param/burger.yaml            # Parámetros Nav2 (AMCL, costmaps, planner…)
│   └── launch/
│       ├── my_tb3_navigator.launch.py
│       └── start_all.launch.py      # Lanzamiento único: Gazebo + Nav2 + initial_pose_pub
├── carrybot_nav2_sector/            # Patrulla por zonas
│   ├── carrybot_nav2_sector/sector_navigator.py
│   └── config/sectores.json         # Definición de zonas con waypoints
├── carrybot_nav_recogida/           # Ruta de recogida automática
│   ├── carrybot_nav_recogida/ruta_fija.py
│   └── config/coordenadas.json      # Lista de pedidos + punto de entrega
└── carrybot_web_bridge/             # Puente ROS 2 ↔ Web
    ├── carrybot_web_bridge/web_bridge.py
    └── config/                      # Copias de locations, sectores y coordenadas
```

---

## 🚀 Instalación

```bash
# 1. Clonar el repositorio en el workspace de ROS 2
cd ~/ros2_ws/src
git clone https://github.com/JaimeSM21/carrybot.git

# 2. Instalar dependencias
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. Compilar el workspace
colcon build --symlink-install

# 4. Cargar el entorno
source install/setup.bash
```

---

## ▶️ Secuencia de lanzamiento

Lanzar cada paso en una **terminal separada**, en orden:

```bash
# Terminal 1 — Simulación Gazebo + Nav2 + AMCL
ros2 launch carrybot_nav2_punto start_all.launch.py use_sim_time:=True

# Terminal 2 — Servidor WebSocket rosbridge (puerto 9090)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml delay_between_messages:=0.0

# Terminal 3 — Nodo puente web
ros2 run carrybot_web_bridge web_bridge

# Terminal 4 — Interfaz web React
cd src/carrybot/carrybot_web
npm install   # solo la primera vez
npm run dev   # disponible en http://localhost:5173
```

---

## 📦 Paquetes

### `carrybot_mundo`
Proporciona la simulación Gazebo Harmonic. Instancia el TurtleBot3 Burger en el mundo warehouse y publica todos los topics de sensores de bajo nivel.

| Topic | Tipo | Dirección |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Publicado |
| `/odom` | `nav_msgs/Odometry` | Publicado |
| `/joint_states` | `sensor_msgs/JointState` | Publicado |
| `/clock` | `rosgraph_msgs/Clock` | Publicado |
| `/cmd_vel_nav` | `geometry_msgs/TwistStamped` | Suscrito |

---

### `carrybot_nav2_punto`
Stack Nav2 completo con localización AMCL. Expone dos nodos:

- **`initial_pose_pub`** — publica un único mensaje en `/initialpose` al arrancar para que AMCL conozca la posición inicial del robot.
- **`nav_to_point`** — cliente de la acción `NavigateToPose`. Acepta un nombre de destino por consola, carga las coordenadas desde `locations.json` y envía el goal a Nav2 con feedback de distancia en tiempo real.

```bash
# Uso independiente (Nav2 debe estar ya en ejecución)
ros2 run carrybot_nav2_punto nav_to_point
# > Introduce destino: Estanteria1
```

---

### `carrybot_nav2_sector`
Patrulla por zonas usando `FollowWaypoints`. El nodo `sector_navigator` lee `sectores.json`, construye la lista completa de poses de la zona elegida y la envía a Nav2 en una sola llamada a la acción.

```bash
ros2 run carrybot_nav2_sector sector_navigator
# > Introduce zona: Zona1
```

---

### `carrybot_nav_recogida`
Ruta de recogida completamente automática. El nodo `ruta_fija` usa `BasicNavigator` para iterar sobre todos los pedidos de `coordenadas.json`: por cada pedido navega al punto de recogida, espera 5 segundos y navega al punto de entrega, sin intervención del usuario.

```bash
ros2 run carrybot_nav_recogida ruta_fija
```

---

### `carrybot_web_bridge`
Puente entre la aplicación web React y Nav2. Se suscribe a los topics de comandos web y lanza las acciones Nav2 correspondientes; publica el estado de la navegación y los anuncios de entrega de vuelta hacia la web.

**Topics suscritos (comandos desde la web):**

| Topic | Mensaje | Acción que lanza |
|---|---|---|
| `/web/nav_goal` | `std_msgs/String` | `NavigateToPose` al destino indicado |
| `/web/patrol_goal` | `std_msgs/String` | `FollowWaypoints` para la zona indicada |
| `/web/ruta_fija` | `std_msgs/String` | Lanza `ruta_fija` como subproceso |
| `/web/cancel` | `std_msgs/String` | Cancela el goal activo + publica velocidad cero |

**Topics publicados (estado hacia la web):**

| Topic | Mensaje | Descripción |
|---|---|---|
| `/web/status` | `std_msgs/String` | `navegando`, `llegado`, `fallo`, `cancelado`… |
| `/delivery/announcement` | `std_msgs/String` | Eventos de recogida y entrega de paquetes |

---

## 📍 Destinos disponibles

Definidos en `carrybot_nav2_punto/config/locations.json`:

| Nombre | x (m) | y (m) | Descripción |
|---|---|---|---|
| `Estanteria1` | 6.917 | 2.282 | Primera estantería del almacén |
| `Estanteria2` | 8.808 | 2.295 | Segunda estantería del almacén |
| `PuntoDeCarga` | 4.663 | 1.682 | Zona de carga y descarga |

Zonas de patrulla definidas en `carrybot_nav2_sector/config/sectores.json`:

| Zona | Waypoints | Patrón |
|---|---|---|
| `Zona1` | 6 | Zigzag por el pasillo de la primera zona |
| `Zona2` | 7 | Recorrido completo de ida y vuelta por el segundo pasillo |

---

## 🔗 Diagrama de comunicación entre nodos

```
[carrybot_web]  ──pub /web/nav_goal──────▶  [carrybot_web_bridge]
                ──pub /web/patrol_goal───▶  [carrybot_web_bridge]
                ──pub /web/ruta_fija─────▶  [carrybot_web_bridge]
                ──pub /web/cancel────────▶  [carrybot_web_bridge]
                ◀──sub /web/status─────────  [carrybot_web_bridge]
                ◀──sub /delivery/announce──  [carrybot_web_bridge]

[carrybot_web_bridge]  ──ACCIÓN NavigateToPose──▶  [Nav2 / carrybot_nav2_punto]
                       ──ACCIÓN FollowWaypoints─▶  [Nav2 / carrybot_nav2_punto]
                       ──subproceso─────────────▶  [carrybot_nav_recogida]

[Nav2]  ◀── /scan  (LaserScan) ──  [carrybot_mundo / LiDAR]
        ◀── /odom  (Odometry)  ──  [carrybot_mundo / encoders]
        ──▶ /cmd_vel_nav       ──▶  [carrybot_mundo / motores]
```

---

## 📄 Licencia

Apache 2.0 — consulta el fichero [`LICENSE`](LICENSE) para más detalles.
