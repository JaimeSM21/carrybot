# CarryBot — Robot Real

> **GTI 2026 · Grupo 03**  
> Emilio Sánchez Granado · Jaime Sánchez Martí · Endika Matute Blanco · Sandra Moll Cots · Rocío Piquer Bacete

Sistema de navegación autónoma para un **TurtleBot3 Burger físico** en un entorno de almacén real, construido con **ROS 2 Jazzy**. Esta rama (`RobotReal`) adapta el stack del simulador para operar sobre el hardware real: sin Gazebo, con mapa cartografiado del entorno físico, coordenadas calibradas sobre el suelo real y un nodo de visión con ventana OpenCV interactiva.

> ⚠️ Esta rama **no se integra con la interfaz web** (`carrybot_web`). El control y la visualización se realizan íntegramente desde consola y RViz2.

---

## Tabla de contenidos

- [Diferencias respecto al simulador](#diferencias-respecto-al-simulador)
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
- [Diagrama de topics y nodos](#diagrama-de-topics-y-nodos)

---

## Diferencias respecto al simulador

| Aspecto | Simulador (`main`) | Robot real (`RobotReal`) |
|---|---|---|
| Entorno | Gazebo Harmonic + mundo SDF | Hardware físico TurtleBot3 |
| Tiempo de simulación | `use_sim_time:=true` | `use_sim_time:=false` |
| Mapa | `my_map.pgm` (Gazebo) | `real_map.pgm` (cartografiado con SLAM) |
| Coordenadas | Escala del mundo virtual (~9 m) | Escala del espacio físico real (~1 m) |
| Integración web | `carrybot_web_bridge` activo | Sin integración web |
| Visión | `package_detector.py` (solo topics) | `carrybot_detector_ros2.py` (ventana OpenCV interactiva) |
| `carrybot_mundo` | Necesario (Gazebo) | No se lanza |

---

## Funcionalidades

- **Navegación a punto concreto** — envía el robot a una estantería o al punto de carga indicando su nombre; Nav2 calcula la ruta óptima con AMCL.
- **Patrulla por sectores** — recorre una zona completa del entorno real mediante waypoints usando `FollowWaypoints`.
- **Ruta de recogida automática** — procesa todos los pedidos de `coordenadas.json` en orden: navega al punto de recogida, espera 5 s y navega al punto de entrega, sin intervención del usuario.
- **Pedido individual** — recoge y entrega un único pedido seleccionado por nombre desde consola.
- **Detección de paquetes con visor** — nodo `carrybot_detector_ros2` con ventana OpenCV interactiva: detecta cajas y códigos QR, guarda capturas en disco y permite congelar/reanudar el stream.

---

## Arquitectura del sistema

```
┌───────────────────────────────────────────────────────────┐
│              TurtleBot3 Burger (hardware real)             │
│   LiDAR · encoders · cámara · OpenCR                      │
└──────────────────────┬────────────────────────────────────┘
                       │ /scan  /odom  /cmd_vel  /camera/image_raw
┌──────────────────────▼────────────────────────────────────┐
│               Nav2 + AMCL  (use_sim_time=false)            │
│   carrybot_nav2_punto  ·  carrybot_nav2_sector             │
│   carrybot_nav_recogida                                    │
└───────────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────┐
│           carrybot_detector_ros2  (visión)                 │
│   /camera/image_raw → detección cajas + QR                │
│   Ventana OpenCV interactiva · log JSON en disco           │
└───────────────────────────────────────────────────────────┘
```

---

## Paquetes ROS 2

| Paquete | Build | Responsabilidad |
|---|---|---|
| `carrybot_nav2_punto` | ament_python | Nav2 + AMCL + mapa real + navegación a un punto concreto |
| `carrybot_nav2_sector` | ament_python | Patrulla por sectores con múltiples waypoints |
| `carrybot_nav_recogida` | ament_python | Ruta fija automática de recogida desde pedidos JSON |
| `carrybot_web_bridge` | ament_python | Contiene `carrybot_detector_ros2.py`; el puente web no se utiliza en esta rama |

> El paquete `carrybot_mundo` (Gazebo) **no se lanza** en el robot real.

---

## Requisitos previos

| Dependencia | Versión |
|---|---|
| ROS 2 | Jazzy |
| Nav2 | incluido con ROS 2 Jazzy |
| nav2_simple_commander | `ros-jazzy-nav2-simple-commander` |
| cv_bridge | `ros-jazzy-cv-bridge` |
| Python | ≥ 3.10 |
| OpenCV | `python3-opencv` |
| NumPy | `python3-numpy` |

```bash
sudo apt install \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-simple-commander \
  ros-jazzy-cv-bridge \
  python3-opencv \
  python3-numpy
```

El robot debe estar arrancado con los nodos de bajo nivel publicando `/scan`, `/odom` y `/cmd_vel` antes de lanzar Nav2. Consulta la documentación de TurtleBot3 para el bringup del hardware.

---

## Estructura del repositorio

```
carrybot/
├── carrybot/                               # Meta-paquete raíz (vacío, solo package.xml)
│
├── carrybot_nav2_punto/                    # Navegación a punto concreto
│   ├── carrybot_nav2_punto/
│   │   ├── initial_pose_pub.py             # Publica la pose inicial para AMCL (x=0, y=0)
│   │   └── nav_to_point.py                 # Cliente NavigateToPose: navega a un destino por nombre
│   ├── config/locations.json               # Estanterías y punto de carga (coordenadas reales)
│   ├── map/
│   │   ├── my_map.{pgm,yaml}              # Mapa del simulador (no usado en esta rama)
│   │   └── real_map.{pgm,yaml}            # Mapa cartografiado del entorno físico ← usado
│   ├── param/
│   │   ├── burger.yaml                    # Parámetros Nav2 para el robot real
│   │   └── burger_cam.yaml                # Parámetros Nav2 con cámara
│   └── launch/
│       ├── my_tb3_navigator.launch.py     # Nav2 + AMCL + RViz2 (carga real_map.yaml)
│       └── start_all.launch.py            # ⚠ Lanza también Gazebo; no usar en robot real
│
├── carrybot_nav2_sector/                   # Patrulla por zonas
│   ├── carrybot_nav2_sector/sector_navigator.py
│   └── config/sectors.json               # Zonas del entorno real con lista de waypoints
│
├── carrybot_nav_recogida/                  # Ruta de recogida automática
│   ├── carrybot_nav_recogida/
│   │   ├── initial_pose_pub.py
│   │   └── ruta_fija.py
│   ├── config/coordenadas.json           # Pedidos con coordenadas reales + punto de entrega
│   ├── maps/my_map.{pgm,yaml}            # Mapa para arrancar Nav2 autónomamente
│   ├── param/nav2_params.yaml
│   └── launch/arrancar_sistema.launch.py # Arranca Nav2 + RViz2 sin Gazebo
│
└── carrybot_web_bridge/                    # Solo se usa carrybot_detector_ros2.py
    ├── carrybot_web_bridge/
    │   ├── carrybot_detector_ros2.py      # Nodo de visión con ventana OpenCV interactiva
    │   └── web_bridge.py                  # No se utiliza en esta rama
    └── config/
        ├── locations.json
        ├── sectors.json
        └── coordenadas.json
```

---

## Instalación

```bash
# 1. Clonar la rama RobotReal en el workspace ROS 2
cd ~/turtlebot3_ws/src
git clone -b RobotReal https://github.com/JaimeSM21/carrybot.git

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

Abrir cada paso en una **terminal separada**, en el orden indicado. Ejecuta `source install/setup.bash` en cada una antes de lanzar.

```bash
# Terminal 1 — Bringup del hardware TurtleBot3
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
# Espera a que /scan y /odom estén publicando antes de continuar.

# Terminal 2 — Nav2 + AMCL + RViz2 (mapa real, sin simulación)
export TURTLEBOT3_MODEL=burger
ros2 launch carrybot_nav2_punto my_tb3_navigator.launch.py use_sim_time:=false

# En el Rviz usar 2D Pose Estimate en la posicion del robot

# Terminal 3 (opcional) — Detección de paquetes con visor interactivo
ros2 run carrybot_web_bridge carrybot_detector_ros2
```

> **Nota:** No uses `start_all.launch.py` en el robot real; ese launch file está diseñado para el simulador e intenta arrancar Gazebo.

---

## Modos de operación

### Navegación a punto concreto

Navega a una estantería o al punto de carga indicando su nombre. Nav2 calcula la ruta óptima con AMCL y la ejecuta.

```bash
ros2 run carrybot_nav2_punto nav_to_point
# > Introduce destino: Estanteria1
```

---

### Patrulla por sectores

Recorre una zona completa del entorno usando la acción `FollowWaypoints`. Los waypoints se cargan de `sectors.json` y definen el recorrido en patrón de bucle.

```bash
ros2 run carrybot_nav2_sector sector_navigator
# > Introduce zona: Zona1
```

---

### Ruta de recogida automática (completa)

Itera sobre todos los pedidos de `coordenadas.json` en orden: navega al punto de recogida, espera 5 s simulando la carga y navega al punto de entrega. Sin intervención del usuario.

```bash
ros2 run carrybot_nav_recogida ruta_fija
```

Alternativamente, arranca Nav2 y la ruta en una sola terminal:

```bash
ros2 launch carrybot_nav_recogida arrancar_sistema.launch.py use_sim_time:=false
```

---

### Pedido individual

Ejecuta la recogida y entrega de un único pedido seleccionado por nombre.

```bash
ros2 run carrybot_nav_recogida ruta_fija --pedido pedido1
```


---

## Destinos y zonas disponibles

### Puntos de navegación — `locations.json`

Coordenadas calibradas sobre el entorno físico real (origen en la posición de arranque del robot, x=0, y=0).

| Nombre | x (m) | y (m) | Descripción |
|---|---|---|---|
| `Estanteria1` | 0.856 | −0.028 | Primera estantería del almacén |
| `Estanteria2` | 0.867 | −0.957 | Segunda estantería del almacén |
| `PuntoDeCarga` | −0.096 | −0.914 | Zona de carga y descarga |

### Zonas de patrulla — `sectors.json`

| Zona | Waypoints | Descripción |
|---|---|---|
| `Zona1` | 4 | Bucle por las tres posiciones principales del entorno real |
| `Zona2` | 7 | Recorrido de prueba (coordenadas de respaldo) |

### Pedidos de recogida — `coordenadas.json`

| Pedido | x recogida (m) | y recogida (m) | Punto de entrega |
|---|---|---|---|
| `pedido1` | 0.856 | −0.028 | x=−0.098, y=−0.031 |
| `pedido2` | 0.867 | −0.957 | x=−0.098, y=−0.031 |
| `pedido3` | −0.096 | −0.914 | x=−0.098, y=−0.031 |

---

## Visión artificial y detección de paquetes

El nodo `carrybot_detector_ros2` (paquete `carrybot_web_bridge`) procesa el stream de la cámara del robot en tiempo real usando OpenCV y abre una ventana interactiva en el equipo donde se ejecuta.

### Pipeline de detección

1. **Detección de cajas** — aplica Gaussian Blur + Canny + dilatación y filtra contornos de 4 vértices con área mínima de 4000 px² y relación de aspecto entre 0.35 y 3.5.
2. **Detección de QR** — decodifica con `cv2.QRCodeDetector`; cada QR nuevo se registra (con cooldown de sesión) en el log JSON y se publica en el topic `/carrybot/qr_detect`.
3. **HUD en pantalla** — muestra FPS, número de cajas, estado del QR y total de detecciones guardadas.
4. **Botón Actualizar** — congela/descongela el frame actual con un clic en la esquina superior derecha de la ventana, o pulsando `R`.

### Controles del visor

| Tecla / acción | Efecto |
|---|---|
| `Q` / `ESC` | Cerrar el nodo |
| `S` | Guardar captura JPG en disco |
| `C` | Limpiar el historial de QRs de la sesión actual |
| `R` | Congelar / reanudar el stream |
| Clic en botón | Congelar / reanudar el stream |

### Topics

| Topic | Tipo | Dirección | Descripción |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | Suscripción | Stream de la cámara del robot |
| `/carrybot/qr_detect` | `std_msgs/String` | Publicación | JSON con cada nuevo QR detectado |

### Formato JSON publicado en `/carrybot/qr_detect`

```json
{
  "timestamp": "2026-06-03T10:45:00",
  "qr_raw": "{\"id\":\"PKG-001\",\"dest\":\"Estanteria1\",\"weight\":\"2.5kg\"}",
  "qr_parsed": {"id": "PKG-001", "dest": "Estanteria1", "weight": "2.5kg"}
}
```

Las detecciones se guardan acumulativamente en `/home/emilio/capturas_robot/qr_log.json` y las capturas de pantalla en el mismo directorio.

---

## Diagrama de topics y nodos

```
[TurtleBot3 hardware]
  ──pub /scan       (LaserScan)  ──▶  [Nav2 / AMCL]
  ──pub /odom       (Odometry)   ──▶  [Nav2 / AMCL]
  ──pub /camera/image_raw        ──▶  [carrybot_detector_ros2]
  ◀──sub /cmd_vel   (Twist)      ──   [Nav2]

[carrybot_nav2_punto / initial_pose_pub]
  ──pub /initialpose             ──▶  [Nav2 / AMCL]

[carrybot_nav2_punto / nav_to_point]
  ──ACCIÓN NavigateToPose        ──▶  [Nav2]

[carrybot_nav2_sector / sector_navigator]
  ──ACCIÓN FollowWaypoints       ──▶  [Nav2]

[carrybot_nav_recogida / ruta_fija]
  ──ACCIÓN NavigateToPose        ──▶  [Nav2]

[carrybot_detector_ros2]
  ◀──sub /camera/image_raw       ──   [TurtleBot3 hardware]
  ──pub /carrybot/qr_detect      ──▶  (consola / log JSON)
```

---

## Licencia

Apache 2.0 — consulta el fichero [`LICENSE`](LICENSE) para más detalles.