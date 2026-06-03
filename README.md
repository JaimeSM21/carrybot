# CarryBot — Simulador ROS 2

> **GTI 2026 · Grupo 03**  
> Emilio Sánchez Granado · Jaime Sánchez Martí · Endika Matute Blanco · Sandra Moll Cots · Rocío Piquer Bacete

Sistema de navegación autónoma para un robot **TurtleBot3 Burger** en un almacén simulado, construido con **ROS 2 Jazzy** y **Gazebo Harmonic**. El robot navega por el entorno, detecta y entrega paquetes evitando obstáculos, y puede controlarse tanto desde consola como desde la [interfaz web React](https://github.com/JaimeSM21/carrybot_web).

---

## Tabla de contenidos

- [Funcionalidades](#funcionalidades)
- [Arquitectura del sistema](#arquitectura-del-sistema)
- [Paquetes ROS 2](#paquetes-ros-2)
- [Requisitos previos](#requisitos-previos)
- [Estructura del repositorio](#estructura-del-repositorio)
- [Instalación](#instalación)
- [Secuencia de lanzamiento](#secuencia-de-lanzamiento)
- [Modos de operación](#modos-de-operación)
- [Destinos y zonas disponibles](#destinos-y-zonas-disponibles)
- [Visión artificial y detección de paquetes](#visión-artificial-y-detección-de-paquetes)
- [Generación de texturas QR](#generación-de-texturas-qr)
- [Diagrama de topics y nodos](#diagrama-de-topics-y-nodos)

---

## Funcionalidades

- **Navegación a punto concreto** — envía el robot a una estantería o al punto de carga por nombre.
- **Patrulla por sectores** — recorre una zona completa del almacén mediante waypoints en zigzag usando `FollowWaypoints`.
- **Ruta de recogida automática** — procesa todos los pedidos de un JSON en orden sin intervención del usuario.
- **Pedido individual** — recoge y entrega un pedido concreto seleccionado desde consola o desde la web.
- **Detección de paquetes por cámara** — visión artificial con OpenCV: detecta cajas por contornos y lee códigos QR en tiempo real.
- **Panel de control web** — cámara en vivo, minimapa del entorno, joystick de teleoperación y botones de navegación (ver repositorio `carrybot_web`).
- **Cancelación remota** — detiene cualquier goal de navegación activo desde la web o la consola.
- **Puente ROS 2 ↔ Web** — comunicación bidireccional en tiempo real mediante rosbridge WebSocket.

---

## Arquitectura del sistema

```
┌─────────────────────────────────────────────────────────────┐
│                   carrybot_web  (React)                      │
│   Panel web · Cámara · Mapa · Joystick · Botones Nav        │
└──────────────────────┬──────────────────────────────────────┘
                       │ WebSocket (rosbridge :9090)
┌──────────────────────▼──────────────────────────────────────┐
│              carrybot_web_bridge  (ROS 2 node)               │
│   Traduce topics /web/* → acciones Nav2 y subprocesos        │
└────┬───────────────────────────────────────────┬────────────┘
     │ NavigateToPose / FollowWaypoints           │ subprocess
┌────▼──────────────────┐            ┌────────────▼──────────┐
│   Nav2 + AMCL         │            │  carrybot_nav_recogida│
│  carrybot_nav2_punto  │            │  ruta_fija.py         │
│  carrybot_nav2_sector │            └───────────────────────┘
└────┬──────────────────┘
     │ /scan  /odom  /cmd_vel
┌────▼──────────────────────────────────────────────────────────┐
│              carrybot_mundo  (Gazebo Harmonic)                 │
│   Mundo warehouse · TurtleBot3 Burger con cámara · LiDAR      │
└───────────────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────────┐
│              carrybot_vision  (ROS 2 node)                     │
│   /camera/image_raw → detección cajas + QR → /camera/processed│
└───────────────────────────────────────────────────────────────┘
```

---

## Paquetes ROS 2

| Paquete | Build | Responsabilidad |
|---|---|---|
| `carrybot_mundo` | ament_cmake | Simulación Gazebo Harmonic: mundo warehouse + TurtleBot3 |
| `carrybot_nav2_punto` | ament_python | Nav2 + AMCL + mapa + navegación a un punto concreto |
| `carrybot_nav2_sector` | ament_python | Patrulla por sectores con múltiples waypoints |
| `carrybot_nav_recogida` | ament_python | Ruta fija automática de recogida desde pedidos JSON |
| `carrybot_vision` | ament_python | Detección de cajas y QR por cámara (OpenCV + cv_bridge) |
| `carrybot_web_bridge` | ament_python | Puente ROS 2 ↔ Web (rosbridge WebSocket) |

---

## Requisitos previos

| Dependencia | Versión |
|---|---|
| ROS 2 | Jazzy |
| Gazebo | Harmonic |
| Nav2 | incluido con ROS 2 Jazzy |
| rosbridge_suite | `ros-jazzy-rosbridge-suite` |
| nav2_simple_commander | `ros-jazzy-nav2-simple-commander` |
| cv_bridge | `ros-jazzy-cv-bridge` |
| Python | ≥ 3.10 |
| OpenCV | `python3-opencv` |
| NumPy | `python3-numpy` |

```bash
# Dependencias ROS 2 y sistema
sudo apt install \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-rosbridge-suite \
  ros-jazzy-nav2-simple-commander \
  ros-jazzy-cv-bridge \
  python3-opencv \
  python3-numpy
```

---

## Estructura del repositorio

```
carrybot/
├── carrybot/                             # Meta-paquete raíz (vacío, solo package.xml)
│
├── carrybot_mundo/                       # Simulación Gazebo
│   ├── launch/
│   │   ├── my_world.launch.py            # Arranca Gazebo + spawn del robot
│   │   ├── spawn_turtlebot3.launch.py    # Solo spawn del robot
│   │   └── robot_state_publisher.launch.py
│   ├── worlds/warehouse.world            # Mundo SDF del almacén
│   ├── models/
│   │   ├── caja_qr/                      # Modelo SDF de la caja con textura QR
│   │   ├── turtlebot3_burger/            # TurtleBot3 sin cámara
│   │   ├── turtlebot3_burger_cam/        # TurtleBot3 con cámara integrada
│   │   ├── turtlebot3_common/            # Meshes compartidas (STL, DAE)
│   │   └── workcell/                     # Modelo 3D del almacén con texturas
│   ├── urdf/
│   │   ├── turtlebot3_burger.urdf        # URDF base del TurtleBot3
│   │   └── turtlebot3_burger_cam.urdf    # URDF con cámara integrada
│   ├── params/
│   │   ├── turtlebot3_burger_bridge.yaml       # Puente Gazebo↔ROS (sin cámara)
│   │   └── turtlebot3_burger_cam_bridge.yaml   # Puente Gazebo↔ROS (con cámara)
│   ├── include/ y src/                   # Plugins C++ de Gazebo (obstáculos, semáforos)
│   ├── qr_fuente/                        # Carpeta donde depositar la imagen QR fuente
│   └── generar_qr_textura.py             # Script para preparar la textura QR
│
├── carrybot_nav2_punto/                  # Navegación a punto concreto
│   ├── carrybot_nav2_punto/
│   │   ├── initial_pose_pub.py           # Publica la pose inicial para AMCL
│   │   └── nav_to_point.py              # Cliente de acción: navega a un destino por nombre
│   ├── config/locations.json            # Estanterías y punto de carga con coordenadas
│   ├── map/my_map.{pgm,yaml}            # Mapa del almacén
│   ├── param/
│   │   ├── burger.yaml                  # Parámetros Nav2 (sin cámara)
│   │   └── burger_cam.yaml              # Parámetros Nav2 (con cámara)
│   └── launch/
│       ├── my_tb3_navigator.launch.py   # Nav2 + AMCL + RViz2
│       └── start_all.launch.py          # Lanzamiento único: Gazebo + Nav2 + initial_pose
│
├── carrybot_nav2_sector/                 # Patrulla por zonas
│   ├── carrybot_nav2_sector/sector_navigator.py
│   └── config/sectors.json             # Zonas con lista de waypoints
│
├── carrybot_nav_recogida/                # Ruta de recogida automática
│   ├── carrybot_nav_recogida/ruta_fija.py
│   ├── config/coordenadas.json         # Pedidos + punto de entrega
│   └── launch/arrancar_sistema.launch.py
│
├── carrybot_vision/                      # Visión artificial
│   ├── carrybot_vision/package_detector.py
│   └── detecciones.json                 # Log de paquetes detectados (generado en runtime)
│
└── carrybot_web_bridge/                  # Puente ROS 2 ↔ Web
    ├── carrybot_web_bridge/web_bridge.py
    └── config/
        ├── locations.json               # Copia de destinos (sincronizada con nav2_punto)
        ├── sectors.json                 # Copia de zonas (sincronizada con nav2_sector)
        └── coordenadas.json             # Copia de pedidos (sincronizada con nav_recogida)
```

---

## Instalación

```bash
# 1. Clonar el repositorio dentro del workspace ROS 2
cd ~/turtlebot3_ws/src
git clone -b release04 https://github.com/JaimeSM21/carrybot.git

# 2. Instalar dependencias declaradas en package.xml
cd ~/turtlebot3_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. Compilar el workspace
colcon build --symlink-install

# 4. Cargar el entorno en cada terminal nueva
source install/setup.bash
```

---

## Secuencia de lanzamiento

Abrir cada paso en una **terminal separada**, en el orden indicado. Asegúrate de ejecutar `source install/setup.bash` en cada una.

```bash
# Terminal 1 — Simulación completa: Gazebo + Nav2 + AMCL + RViz2 + pose inicial
ros2 launch carrybot_nav2_punto start_all.launch.py use_sim_time:=True
# Espera ~20 s hasta que RViz2 muestre el mapa y la pose localizada del robot.

# Terminal 2 — Servidor WebSocket rosbridge (puerto 9090)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml delay_between_messages:=0.0

# Terminal 3 — Nodo puente web (gestiona los comandos desde el navegador)
ros2 run carrybot_web_bridge web_bridge

# Terminal 4 (opcional) — Detección de paquetes por cámara
ros2 run carrybot_vision package_detector
```

Para usar la interfaz web, consulta el repositorio [carrybot_web](https://github.com/JaimeSM21/carrybot_web).

---

## Modos de operación

### Navegación a punto concreto

Navega a una estantería o al punto de carga indicando su nombre. Nav2 calcula la ruta óptima con AMCL y la ejecuta publicando en `/cmd_vel`.

```bash
ros2 run carrybot_nav2_punto nav_to_point
# > Introduce destino: Estanteria1
```

También funciona desde la web publicando en `/web/nav_goal` (gestionado por `web_bridge`).

---

### Patrulla por sectores

Recorre una zona completa del almacén usando la acción `FollowWaypoints`. Los waypoints definen un patrón en zigzag que cubre el pasillo completo.

```bash
ros2 run carrybot_nav2_sector sector_navigator
# > Introduce zona: Zona1
```

Desde la web: publica en `/web/patrol_goal`.

---

### Ruta de recogida automática (completa)

Itera sobre todos los pedidos de `coordenadas.json` en orden: por cada pedido navega al punto de recogida, espera 5 segundos simulando la carga, y navega al punto de entrega. Sin intervención del usuario.

```bash
ros2 run carrybot_nav_recogida ruta_fija
```

Desde la web: publica en `/web/ruta_fija`.

---

### Pedido individual

Ejecuta la recogida y entrega de un único pedido seleccionado por nombre.

```bash
ros2 run carrybot_nav_recogida ruta_fija --pedido pedido1
```

Desde la web: publica en `/web/pedido` con el nombre del pedido.

---

### Cancelación

Detiene inmediatamente cualquier goal activo y publica velocidad cero.

```bash
# Desde consola
ros2 topic pub --once /web/cancel std_msgs/msg/String "data: 'cancel'"
```

Desde la web: botón **Stop** del panel de control.

---

## Destinos y zonas disponibles

### Puntos de navegación — `locations.json`

| Nombre | x (m) | y (m) | Descripción |
|---|---|---|---|
| `Estanteria1` | 6.917 | 2.282 | Primera estantería del almacén |
| `Estanteria2` | 8.808 | 2.295 | Segunda estantería del almacén |
| `PuntoDeCarga` | 4.663 | 1.682 | Zona de carga y descarga |

### Zonas de patrulla — `sectors.json`

| Zona | Waypoints | Descripción |
|---|---|---|
| `Zona1` | 6 | Zigzag por el pasillo de la primera zona del almacén |
| `Zona2` | 7 | Recorrido de ida y vuelta por el segundo pasillo |

### Pedidos de recogida — `coordenadas.json`

| Pedido | x (m) | y (m) | Punto de entrega |
|---|---|---|---|
| `pedido1` | 0.591 | 0.008 | x=2.6, y=0.55 |
| `pedido2` | -0.009 | 1.997 | x=2.6, y=0.55 |
| `pedido3` | 2.900 | 1.650 | x=2.6, y=0.55 |

---

## Visión artificial y detección de paquetes

El nodo `package_detector` (`carrybot_vision`) procesa el stream de la cámara del robot en tiempo real usando OpenCV.

### Pipeline de detección

1. **Detección de QR** — prueba cuatro variantes del frame (original, umbral Otsu, escalado x2, escalado x2 + Otsu) con `cv2.QRCodeDetector` para maximizar la tasa de lectura.
2. **Detección de cajas** — aplica Canny + dilatación y filtra contornos de 4 vértices con área mínima de 3000 px² y relación de aspecto entre 0.4 y 3.0.
3. **Anotación** — dibuja bounding boxes y etiquetas sobre el frame y lo publica como imagen procesada.
4. **Log automático** — cada detección nueva (respetando un cooldown de 5 s) se añade a `detecciones.json` con un ID auto-incremental (`PKG-001`, `PKG-002`…).

### Topics

| Topic | Tipo | Dirección | Descripción |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | Suscripción | Stream de la cámara del robot |
| `/package/detection` | `std_msgs/String` | Publicación | JSON con resultado de detección |
| `/camera/processed` | `sensor_msgs/Image` | Publicación | Frame anotado con bounding boxes |

### Formato JSON de `/package/detection`

```json
{
  "box_detected": true,
  "boxes": [{"x": 120, "y": 80, "w": 200, "h": 150}],
  "qr_detected": true,
  "qr_data": "{\"id\":\"001\",\"dest\":\"Estanteria1\",\"weight\":\"2.5kg\"}",
  "qr_parsed": {"id": "001", "dest": "Estanteria1", "weight": "2.5kg", "priority": "normal"}
}
```

---

## Generación de texturas QR

El script `generar_qr_textura.py` prepara la textura PNG que Gazebo aplica al modelo de la caja (`caja_qr`).

```bash
# 1. Coloca tu imagen QR (PNG, JPG, BMP, WEBP) en:
#    carrybot_mundo/qr_fuente/

# 2. Ejecuta el script desde el directorio del paquete:
cd ~/turtlebot3_ws/src/carrybot/carrybot_mundo
python3 generar_qr_textura.py

# La textura resultante queda en:
# models/caja_qr/materials/textures/QR-prueba1.png
```

El script escala la imagen a 1024×1024 px con fondo blanco, que es el tamaño óptimo para la renderización en Gazebo Harmonic.

---

## Diagrama de topics y nodos

```
[carrybot_web]
  ──pub /web/nav_goal    ──▶  [web_bridge]  ──▶  NavigateToPose  ──▶  [Nav2]
  ──pub /web/patrol_goal ──▶  [web_bridge]  ──▶  FollowWaypoints ──▶  [Nav2]
  ──pub /web/ruta_fija   ──▶  [web_bridge]  ──▶  subprocess ruta_fija
  ──pub /web/pedido      ──▶  [web_bridge]  ──▶  subprocess ruta_fija --pedido
  ──pub /web/cancel      ──▶  [web_bridge]  ──▶  CancelGoal + /cmd_vel 0
  ◀──sub /web/status         [web_bridge] ◀──────────────────────────────────
  ◀──sub /delivery/announcement

[Nav2]
  ◀── /scan    (LaserScan)  ── [carrybot_mundo / LiDAR]
  ◀── /odom    (Odometry)   ── [carrybot_mundo / encoders]
  ◀── /tf                   ── [carrybot_mundo / Gazebo bridge]
  ──▶ /cmd_vel              ──▶ [carrybot_mundo / DiffDrive]

[carrybot_vision]
  ◀── /camera/image_raw     ── [carrybot_mundo / cámara Gazebo]
  ──▶ /package/detection    ──▶ [carrybot_web_bridge / web]
  ──▶ /camera/processed     ──▶ [web_video_server / carrybot_web]
```

### Puente Gazebo ↔ ROS 2 (ros_gz_bridge)

| Topic ROS 2 | Dirección | Tipo ROS 2 |
|---|---|---|
| `/clock` | GZ → ROS | `rosgraph_msgs/Clock` |
| `/joint_states` | GZ → ROS | `sensor_msgs/JointState` |
| `/odom` | GZ → ROS | `nav_msgs/Odometry` |
| `/tf` | GZ → ROS | `tf2_msgs/TFMessage` |
| `/imu` | GZ → ROS | `sensor_msgs/Imu` |
| `/scan` | GZ → ROS | `sensor_msgs/LaserScan` |
| `/camera/image_raw` | GZ → ROS | `sensor_msgs/Image` |
| `/cmd_vel` | ROS → GZ | `geometry_msgs/TwistStamped` |

---

## Licencia

Apache 2.0 — consulta el fichero [`LICENSE`](LICENSE) para más detalles.