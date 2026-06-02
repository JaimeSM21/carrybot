"""
Modulo de recogida y entrega de pedidos del warehouse.

Implementa dos modos de operacion:
    - Ruta fija completa: recoge todos los pedidos del JSON en orden y los entrega.
    - Pedido individual: recoge un pedido concreto elegido por el usuario y lo entrega.

Al llegar a cada punto de recogida, activa el detector de QR durante 10 segundos
antes de continuar hacia el punto de entrega.

Classes:
    NavegacionError: Excepcion personalizada para fallos de navegacion.

Functions:
    crear_pose(x_coord, y_coord): Genera un PoseStamped con las coordenadas indicadas.
    cargar_coordenadas(): Carga el fichero coordenadas.json de forma segura.
    navegar_a(navigator, pose, descripcion): Navega a una pose y verifica el resultado.
    escanear_qr(node, segundos): Activa el detector de QR durante N segundos.
    recoger_y_entregar(navigator, node, nombre_pedido, meta_recogida, meta_entrega):
        Recoge un pedido, escanea el QR y lo entrega en el punto de entrega.
    modo_ruta_completa(navigator, node, datos_rutas): Ejecuta la ruta completa.
    modo_pedido_individual(navigator, node, datos_rutas): Ejecuta un pedido concreto.
    main(): Funcion principal de ejecucion.
"""

import json
import os
import time
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


# ─── Configuracion del detector QR ───────────────────────────────────────────

LOG_FILE     = "qr_log.json"
CAPTURES_DIR = "capturas/"
SHOW_WINDOW  = True
WINDOW_TITLE = "CarryBot QR — Escaneando paquete..."
SCAN_SECONDS = 10          # segundos de escaneo en cada recogida

COLOR_QR_BORDER = (255, 200,   0)
COLOR_QR_TEXT   = (0,   255, 255)
COLOR_SAVED     = (0,   255, 128)
COLOR_HUD       = (255, 255, 255)
COLOR_TIMER     = (0,   200, 255)


# ─── Excepcion personalizada ─────────────────────────────────────────────────

class NavegacionError(Exception):
    """
    Excepcion que se lanza cuando la navegacion a un destino falla o es cancelada.

    Attributes:
        destino (str): Nombre del destino donde fallo la navegacion.
        resultado (TaskResult): Resultado devuelto por BasicNavigator.
    """

    def __init__(self, destino: str, resultado: TaskResult):
        self.destino   = destino
        self.resultado = resultado
        super().__init__(
            f"La navegacion hacia '{destino}' fallo. "
            f"Resultado Nav2: {resultado}"
        )


# ─── Nodo detector de QR ─────────────────────────────────────────────────────

class QRScannerNode(Node):
    """
    Nodo ROS2 ligero que se suscribe a /camera/image_raw, detecta QR
    y publica los resultados en /carrybot/qr_detect.

    Se instancia una sola vez en main() y se reutiliza en cada recogida
    llamando a escanear_qr(), que lo activa durante SCAN_SECONDS segundos.

    Attributes:
        bridge:       CvBridge para convertir sensor_msgs/Image a OpenCV.
        qr_detector:  Instancia de cv2.QRCodeDetector.
        publisher:    Publicador en /carrybot/qr_detect.
        seen_qrs:     QRs ya guardados en esta sesion.
        latest_frame: Ultimo frame recibido (compartido con el hilo principal).
        frame_lock:   Lock para acceso thread-safe a latest_frame.
        last_qr_data: Ultimo texto QR detectado (None si no hay).
    """

    def __init__(self):
        super().__init__('carrybot_qr_scanner')

        self.bridge      = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()

        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.publisher = self.create_publisher(String, '/carrybot/qr_detect', 10)

        self.seen_qrs:    set   = set()
        self.latest_frame       = None
        self.frame_lock         = threading.Lock()
        self.last_qr_data: str | None = None
        self.saved_until: float = 0.0

    def image_callback(self, msg: Image) -> None:
        """
        Procesa cada frame: detecta QR, guarda en log y publica en ROS2.

        Args:
            msg: Mensaje sensor_msgs/Image recibido del topic.
        """
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as err:
            self.get_logger().error(f'Error convirtiendo imagen: {err}')
            return

        annotated = frame.copy()

        # Detectar QR
        data, bbox, _ = self.qr_detector.detectAndDecode(frame)
        if data and bbox is not None:
            self.last_qr_data = data

            # Dibujar contorno y etiqueta
            pts   = bbox[0].astype(int)
            label = data if len(data) <= 38 else data[:35] + "..."
            cv2.polylines(annotated, [pts], True, COLOR_QR_BORDER, 2)
            cv2.putText(annotated, f"QR: {label}",
                        (pts[0][0], max(pts[0][1] - 10, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, COLOR_QR_TEXT, 2)

            # Guardar si es nuevo en esta sesion
            if data not in self.seen_qrs:
                self.seen_qrs.add(data)
                parsed = self._parse_qr(data)
                entry  = {
                    "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
                    "qr_raw":    data,
                    "qr_parsed": parsed,
                }
                try:
                    self._save_entry(entry)
                    self.saved_until = time.time() + 3.0
                    ros_msg      = String()
                    ros_msg.data = json.dumps(entry, ensure_ascii=False)
                    self.publisher.publish(ros_msg)
                    self.get_logger().info(f'[QR DETECTADO] {data}')
                    if parsed:
                        for k, v in parsed.items():
                            self.get_logger().info(f'               {k}: {v}')
                except OSError as exc:
                    self.get_logger().error(f'No se pudo guardar: {exc}')

            if time.time() < self.saved_until:
                cv2.putText(annotated, "GUARDADO",
                            (pts[0][0], max(pts[0][1] - 30, 38)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_SAVED, 2)

        with self.frame_lock:
            self.latest_frame = annotated

    def get_latest_frame(self):
        """Devuelve una copia thread-safe del ultimo frame anotado."""
        with self.frame_lock:
            if self.latest_frame is not None:
                return self.latest_frame.copy()
        return None

    @staticmethod
    def _parse_qr(data: str) -> dict | None:
        try:
            return json.loads(data)
        except (json.JSONDecodeError, ValueError):
            return None

    @staticmethod
    def _save_entry(entry: dict) -> None:
        log = []
        if os.path.exists(LOG_FILE):
            try:
                with open(LOG_FILE, "r", encoding="utf-8") as f:
                    log = json.load(f)
            except (json.JSONDecodeError, OSError):
                pass
        log.append(entry)
        with open(LOG_FILE, "w", encoding="utf-8") as f:
            json.dump(log, f, ensure_ascii=False, indent=2)


# ─── Funciones auxiliares ─────────────────────────────────────────────────────

def crear_pose(x_coord, y_coord):
    """
    Genera un PoseStamped para enviar coordenadas a Nav2.

    Args:
        x_coord (float): Coordenada X en el frame map.
        y_coord (float): Coordenada Y en el frame map.

    Returns:
        PoseStamped: Pose con orientacion neutra respecto al frame map.
    """
    pose = PoseStamped()
    pose.header.frame_id    = 'map'
    pose.pose.position.x    = float(x_coord)
    pose.pose.position.y    = float(y_coord)
    pose.pose.orientation.w = 1.0
    return pose


def cargar_coordenadas():
    """
    Carga de forma segura el archivo coordenadas.json del paquete.

    Returns:
        dict: Diccionario con 'pedidos' y 'entrega'.

    Raises:
        FileNotFoundError: Si el fichero no existe.
        json.JSONDecodeError: Si el formato JSON es invalido.
        KeyError: Si faltan las claves 'pedidos' o 'entrega'.
    """
    try:
        paquete_dir = get_package_share_directory('carrybot_nav_recogida')
        ruta_json   = os.path.join(paquete_dir, 'config', 'coordenadas.json')
        with open(ruta_json, 'r') as archivo:
            datos = json.load(archivo)

        if 'pedidos' not in datos or 'entrega' not in datos:
            raise KeyError(
                "El fichero coordenadas.json debe contener 'pedidos' y 'entrega'."
            )
        return datos

    except FileNotFoundError:
        print("[ERROR] No se encontro el fichero coordenadas.json.")
        raise
    except json.JSONDecodeError as err:
        print(f"[ERROR] Formato JSON invalido en coordenadas.json: {err}")
        raise


def navegar_a(navigator: BasicNavigator, pose: PoseStamped, descripcion: str):
    """
    Navega a una pose y verifica que la tarea se complete correctamente.

    Args:
        navigator (BasicNavigator): Instancia del navegador Nav2.
        pose (PoseStamped): Pose destino en el frame map.
        descripcion (str): Descripcion del destino para los mensajes de log.

    Returns:
        bool: True si la navegacion se completo correctamente.

    Raises:
        NavegacionError: Si la navegacion falla o es cancelada.
    """
    print(f">>> Navegando hacia: {descripcion}...")
    navigator.goToPose(pose)

    while not navigator.isTaskComplete():
        try:
            feedback = navigator.getFeedback()
            if feedback:
                print(f"    Distancia restante: {feedback.distance_remaining:.2f} m")
        except Exception:
            pass
        time.sleep(1)

    resultado = navigator.getResult()
    if resultado == TaskResult.SUCCEEDED:
        print(f"[OK] Llegada a: {descripcion}")
        return True
    else:
        raise NavegacionError(descripcion, resultado)


def escanear_qr(scanner: QRScannerNode, segundos: int = SCAN_SECONDS) -> str | None:
    """
    Activa la ventana de camara y escanea QR durante N segundos.

    Muestra una barra de cuenta atras en pantalla. Si detecta un QR
    antes de que acabe el tiempo, lo muestra en verde. Al terminar
    cierra la ventana automaticamente.

    Args:
        scanner: Nodo QRScannerNode ya inicializado y con spin activo.
        segundos: Duracion del escaneo en segundos (por defecto SCAN_SECONDS).

    Returns:
        str | None: Texto del QR detectado, o None si no se detecto ninguno.
    """
    print(f"\n[QR] Iniciando escaneo de {segundos} segundos...")
    inicio    = time.time()
    qr_result = None

    if SHOW_WINDOW:
        cv2.namedWindow(WINDOW_TITLE)

    while True:
        transcurrido = time.time() - inicio
        restante     = segundos - transcurrido

        if restante <= 0:
            break

        frame = scanner.get_latest_frame()

        if frame is not None:
            # Barra de progreso en la parte inferior
            h, w = frame.shape[:2]
            progreso = int(w * (transcurrido / segundos))
            cv2.rectangle(frame, (0, h - 12), (progreso, h), (0, 200, 80), -1)
            cv2.rectangle(frame, (0, h - 12), (w, h),        (60, 60, 60),  1)

            # Cuenta atras
            cv2.putText(frame,
                        f"Escaneando... {restante:.1f}s",
                        (10, h - 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_TIMER, 2)

            # Si ya hay QR, mostrarlo en el HUD
            if scanner.last_qr_data:
                qr_result = scanner.last_qr_data
                cv2.putText(frame,
                            f"QR OK: {qr_result[:40]}",
                            (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_SAVED, 2)

            if SHOW_WINDOW:
                cv2.imshow(WINDOW_TITLE, frame)

        if SHOW_WINDOW:
            cv2.waitKey(1)

        time.sleep(0.05)

    if SHOW_WINDOW:
        cv2.destroyWindow(WINDOW_TITLE)

    if qr_result:
        print(f"[QR] Escaneo completado. QR leido: {qr_result}")
    else:
        print("[QR] Escaneo completado. No se detecto ningun QR.")

    # Resetear para el siguiente paquete
    scanner.last_qr_data = None

    return qr_result


# ─── Ciclo de recogida y entrega ──────────────────────────────────────────────

def recoger_y_entregar(navigator: BasicNavigator, scanner: QRScannerNode,
                       nombre_pedido: str, meta_recogida: PoseStamped,
                       meta_entrega: PoseStamped):
    """
    Recoge un pedido, escanea su QR durante SCAN_SECONDS y lo entrega.

    Secuencia:
        1. Navegar al punto de recogida.
        2. Escanear QR durante SCAN_SECONDS segundos.
        3. Navegar al punto de entrega.

    Args:
        navigator (BasicNavigator): Instancia del navegador Nav2.
        scanner (QRScannerNode): Nodo detector de QR.
        nombre_pedido (str): Nombre identificativo del pedido.
        meta_recogida (PoseStamped): Pose de la posicion de recogida.
        meta_entrega (PoseStamped): Pose del punto de entrega.

    Returns:
        bool: True si todo el ciclo se completo correctamente.

    Raises:
        NavegacionError: Si falla la navegacion en alguno de los dos tramos.
    """
    try:
        # 1. Ir a recoger
        print(f"\n>>> [MISION]: Voy a recoger el {nombre_pedido}...")
        navegar_a(navigator, meta_recogida, f"recogida de {nombre_pedido}")

        # 2. Escanear QR
        print(f"[OK] Llegue a {nombre_pedido}. Iniciando escaneo QR...")
        qr = escanear_qr(scanner, segundos=SCAN_SECONDS)

        if qr:
            print(f"[QR] Paquete identificado: {qr}")
        else:
            print(f"[WARN] No se leyo QR en {nombre_pedido}. Continuando igualmente.")

        # 3. Ir a entregar
        print(f">>> [ENTREGA]: Voy a entregar el {nombre_pedido}...")
        navegar_a(navigator, meta_entrega, "punto de entrega")

        print(f"[HECHO] {nombre_pedido} entregado en el punto central.\n")
        time.sleep(2)
        return True

    except NavegacionError as err:
        print(f"[ERROR] Fallo en el ciclo de {nombre_pedido}: {err}")
        raise


# ─── Modos de operacion ───────────────────────────────────────────────────────

def modo_ruta_completa(navigator: BasicNavigator, scanner: QRScannerNode,
                       datos_rutas: dict):
    """
    Ejecuta la ruta completa: recoge todos los pedidos del JSON en orden.

    Args:
        navigator (BasicNavigator): Instancia del navegador Nav2.
        scanner (QRScannerNode): Nodo detector de QR.
        datos_rutas (dict): Diccionario cargado de coordenadas.json.
    """
    coord_entrega    = datos_rutas['entrega']
    meta_entrega     = crear_pose(coord_entrega['x'], coord_entrega['y'])
    pedidos_fallidos = []

    print("\n--- SISTEMA DE TRANSPORTE CARRYBOT ACTIVADO (RUTA COMPLETA) ---")
    print(f"Total de pedidos: {len(datos_rutas['pedidos'])}\n")

    for nombre_pedido, coords in datos_rutas['pedidos'].items():
        meta_recogida = crear_pose(coords['x'], coords['y'])
        try:
            recoger_y_entregar(
                navigator, scanner, nombre_pedido, meta_recogida, meta_entrega
            )
        except NavegacionError as err:
            print(f"[WARN] Saltando {nombre_pedido} por fallo: {err}")
            pedidos_fallidos.append(nombre_pedido)
            continue

    print("\n--- RUTA COMPLETADA ---")
    if pedidos_fallidos:
        print(f"Pedidos con fallo: {pedidos_fallidos}")
    else:
        print("Todos los pedidos entregados correctamente.")


def modo_pedido_individual(navigator: BasicNavigator, scanner: QRScannerNode,
                           datos_rutas: dict):
    """
    Ejecuta la recogida y entrega de un unico pedido elegido por el usuario.

    Args:
        navigator (BasicNavigator): Instancia del navegador Nav2.
        scanner (QRScannerNode): Nodo detector de QR.
        datos_rutas (dict): Diccionario cargado de coordenadas.json.
    """
    coord_entrega = datos_rutas['entrega']
    meta_entrega  = crear_pose(coord_entrega['x'], coord_entrega['y'])

    print("\n--- SISTEMA DE TRANSPORTE CARRYBOT ACTIVADO (PEDIDO INDIVIDUAL) ---")
    print("Pedidos disponibles:")
    for nombre in datos_rutas['pedidos']:
        print(f"  - {nombre}")

    while True:
        nombre_pedido = input("\nEscribe el nombre del pedido a recoger: ").strip()
        if nombre_pedido in datos_rutas['pedidos']:
            break
        print(f"[ERROR] '{nombre_pedido}' no existe. "
              f"Opciones: {list(datos_rutas['pedidos'].keys())}")

    coords        = datos_rutas['pedidos'][nombre_pedido]
    meta_recogida = crear_pose(coords['x'], coords['y'])

    try:
        recoger_y_entregar(
            navigator, scanner, nombre_pedido, meta_recogida, meta_entrega
        )
    except NavegacionError as err:
        print(f"[ERROR] No se pudo completar la entrega: {err}")


# ─── Main ─────────────────────────────────────────────────────────────────────

def main(args=None):
    """
    Inicializa rclpy, el navegador Nav2 y el nodo QR, luego ejecuta
    el modo seleccionado por el usuario.

    El nodo QR corre en un hilo secundario con MultiThreadedExecutor
    para no bloquear la navegacion.

    Args:
        args (list, optional): Argumentos de linea de comandos para rclpy.
    """
    rclpy.init(args=args)

    navigator = BasicNavigator()
    scanner   = QRScannerNode()

    # Lanzar el spin del nodo QR en hilo secundario
    executor   = MultiThreadedExecutor()
    executor.add_node(scanner)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        navigator.waitUntilNav2Active()
        datos_rutas = cargar_coordenadas()

        print("\n=== CARRYBOT — SISTEMA DE ENTREGAS ===")
        print("  1) Ruta completa  (recoge todos los pedidos en orden)")
        print("  2) Pedido individual (elige un pedido concreto)")

        while True:
            modo = input("\nElige el modo (1 o 2): ").strip()
            if modo in ('1', '2'):
                break
            print("[ERROR] Opcion no valida. Escribe 1 o 2.")

        if modo == '1':
            modo_ruta_completa(navigator, scanner, datos_rutas)
        else:
            modo_pedido_individual(navigator, scanner, datos_rutas)

    except FileNotFoundError:
        print("[ERROR] No se pudo cargar el fichero de coordenadas.")
    except json.JSONDecodeError:
        print("[ERROR] El fichero de coordenadas tiene un formato invalido.")
    except KeyboardInterrupt:
        print("\n[INFO] Navegacion interrumpida por el usuario.")
    finally:
        print("\n--- VOLVIENDO A MODO ESPERA ---")
        cv2.destroyAllWindows()
        scanner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()