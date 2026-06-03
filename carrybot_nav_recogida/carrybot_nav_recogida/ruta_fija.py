"""
Modulo de recogida y entrega de pedidos del warehouse.

Implementa dos modos de operacion:
    - Ruta fija completa: recoge todos los pedidos del JSON en orden y los entrega.
    - Pedido individual: recoge un pedido concreto elegido por el usuario y lo entrega.

Classes:
    NavegacionError: Excepcion personalizada para fallos de navegacion.

Functions:
    crear_pose(x_coord, y_coord): Genera un PoseStamped con las coordenadas indicadas.
    cargar_coordenadas(): Carga el fichero coordenadas.json de forma segura.
    navegar_a(navigator, pose, descripcion): Navega a una pose y verifica el resultado.
    recoger_y_entregar(navigator, nombre_pedido, meta_recogida, meta_entrega):
        Recoge un pedido y lo entrega en el punto de entrega.
    modo_ruta_completa(navigator, datos_rutas): Ejecuta la ruta completa.
    modo_pedido_individual(navigator, datos_rutas): Ejecuta un pedido concreto.
    main(): Funcion principal de ejecucion.
"""

import argparse
import json
import os
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


# ─── Excepcion personalizada ─────────────────────────────────────────────────

class NavegacionError(Exception):
    """
    Excepcion que se lanza cuando la navegacion a un destino falla o es cancelada.

    Attributes:
        destino (str): Nombre del destino donde fallo la navegacion.
        resultado (TaskResult): Resultado devuelto por BasicNavigator.
    """

    def __init__(self, destino: str, resultado: TaskResult):
        """
        Inicializa la excepcion con el destino fallido y el resultado de Nav2.

        Args:
            destino (str): Nombre descriptivo del destino donde fallo la navegacion.
            resultado (TaskResult): Resultado de Nav2 (CANCELED, FAILED, etc.).
        """
        self.destino = destino
        self.resultado = resultado
        super().__init__(
            f"La navegacion hacia '{destino}' fallo. "
            f"Resultado Nav2: {resultado}"
        )


# ─── Funciones auxiliares ─────────────────────────────────────────────────────

def crear_pose(x_coord, y_coord):
    """
    Genera un mensaje de tipo PoseStamped para enviar coordenadas a Nav2.

    Args:
        x_coord (float): Coordenada del eje X en el frame map.
        y_coord (float): Coordenada del eje Y en el frame map.

    Returns:
        PoseStamped: Objeto con la posicion y orientacion respecto al frame map.
    """
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = float(x_coord)
    pose.pose.position.y = float(y_coord)
    pose.pose.orientation.w = 1.0
    return pose


def cargar_coordenadas():
    """
    Carga de forma segura el archivo coordenadas.json del paquete.

    Returns:
        dict: Diccionario con los pedidos y el punto de entrega.

    Raises:
        FileNotFoundError: Si el fichero coordenadas.json no existe.
        json.JSONDecodeError: Si el fichero tiene un formato JSON invalido.
        KeyError: Si el fichero no contiene las claves 'pedidos' o 'entrega'.
    """
    try:
        paquete_dir = get_package_share_directory('carrybot_nav_recogida')
        ruta_json = os.path.join(paquete_dir, 'config', 'coordenadas.json')
        with open(ruta_json, 'r') as archivo:
            datos = json.load(archivo)

        # Validar que el JSON tiene la estructura esperada
        if 'pedidos' not in datos or 'entrega' not in datos:
            raise KeyError(
                "El fichero coordenadas.json debe contener las claves "
                "'pedidos' y 'entrega'."
            )
        return datos

    except FileNotFoundError:
        print(f"[ERROR] No se encontro el fichero coordenadas.json.")
        raise
    except json.JSONDecodeError as err:
        print(f"[ERROR] Formato JSON invalido en coordenadas.json: {err}")
        raise


def navegar_a(navigator: BasicNavigator, pose: PoseStamped,
              descripcion: str):
    """
    Navega a una pose y verifica que la tarea se completa correctamente.

    Args:
        navigator (BasicNavigator): Instancia del navegador Nav2.
        pose (PoseStamped): Pose destino en el frame map.
        descripcion (str): Descripcion del destino para los mensajes de log.

    Returns:
        bool: True si la navegacion se completo correctamente.

    Raises:
        NavegacionError: Si la navegacion falla o es cancelada por Nav2.
    """
    print(f">>> Navegando hacia: {descripcion}...")
    navigator.goToPose(pose)

    while not navigator.isTaskComplete():
        try:
            feedback = navigator.getFeedback()
            if feedback:
                distancia = feedback.distance_remaining
                print(f"    Distancia restante: {distancia:.2f} m")
        except Exception:
            pass
        time.sleep(1)

    resultado = navigator.getResult()

    if resultado == TaskResult.SUCCEEDED:
        print(f"[OK] Llegada a: {descripcion}")
        return True
    else:
        raise NavegacionError(descripcion, resultado)


def recoger_y_entregar(navigator: BasicNavigator, nombre_pedido: str,
                       meta_recogida: PoseStamped, meta_entrega: PoseStamped):
    """
    Recoge un pedido concreto y lo lleva al punto de entrega.

    Navega primero a la posicion de recogida, espera 5 segundos simulando
    la recogida fisica, y luego navega al punto de entrega.

    Args:
        navigator (BasicNavigator): Instancia del navegador Nav2.
        nombre_pedido (str): Nombre identificativo del pedido.
        meta_recogida (PoseStamped): Pose de la posicion de recogida del pedido.
        meta_entrega (PoseStamped): Pose del punto central de entrega.

    Returns:
        bool: True si todo el ciclo (recogida + entrega) se completo correctamente.

    Raises:
        NavegacionError: Si falla la navegacion hacia la recogida o la entrega.
    """
    try:
        # Ir a recoger el pedido
        print(f"\n>>> [MISION]: Voy a recoger el {nombre_pedido}...")
        navegar_a(navigator, meta_recogida, f"recogida de {nombre_pedido}")

        print(f"LLEGADA! Recogiendo {nombre_pedido}. "
              f"Por favor, espere 5 segundos...")
        time.sleep(5)

        # Ir a entregar el pedido
        print(f">>> [ENTREGA]: Voy de camino a entregar el {nombre_pedido}...")
        navegar_a(navigator, meta_entrega, "punto de entrega")

        print(f"HECHO! {nombre_pedido} listo para la entrega "
              f"en el punto central.\n")
        time.sleep(2)
        return True

    except NavegacionError as err:
        print(f"[ERROR] Fallo en el ciclo de {nombre_pedido}: {err}")
        raise


# ─── Modos de operacion ───────────────────────────────────────────────────────

def modo_ruta_completa(navigator: BasicNavigator, datos_rutas: dict):
    """
    Ejecuta la ruta completa: recoge todos los pedidos del JSON en orden.

    Itera sobre todos los pedidos definidos en coordenadas.json y para cada
    uno ejecuta el ciclo completo de recogida y entrega. Si un pedido falla,
    registra el error y continua con el siguiente.

    Args:
        navigator (BasicNavigator): Instancia del navegador Nav2.
        datos_rutas (dict): Diccionario cargado de coordenadas.json con
            las claves 'pedidos' y 'entrega'.
    """
    coord_entrega = datos_rutas['entrega']
    meta_entrega = crear_pose(coord_entrega['x'], coord_entrega['y'])
    pedidos_fallidos = []

    print("\n--- SISTEMA DE TRANSPORTE CARRYBOT ACTIVADO (RUTA COMPLETA) ---")
    print(f"Total de pedidos: {len(datos_rutas['pedidos'])}\n")

    for nombre_pedido, coords in datos_rutas['pedidos'].items():
        meta_recogida = crear_pose(coords['x'], coords['y'])

        try:
            recoger_y_entregar(
                navigator, nombre_pedido, meta_recogida, meta_entrega
            )
        except NavegacionError as err:
            print(f"[WARN] Saltando {nombre_pedido} por fallo: {err}")
            pedidos_fallidos.append(nombre_pedido)
            continue

    # Resumen final
    print("\n--- RUTA COMPLETADA ---")
    if pedidos_fallidos:
        print(f"Pedidos con fallo: {pedidos_fallidos}")
    else:
        print("Todos los pedidos entregados correctamente.")


def modo_pedido_individual(navigator: BasicNavigator, datos_rutas: dict):
    """
    Ejecuta la recogida y entrega de un unico pedido elegido por el usuario.

    Muestra la lista de pedidos disponibles, pide al usuario que seleccione
    uno por nombre y ejecuta el ciclo de recogida y entrega para ese pedido.

    Args:
        navigator (BasicNavigator): Instancia del navegador Nav2.
        datos_rutas (dict): Diccionario cargado de coordenadas.json con
            las claves 'pedidos' y 'entrega'.

    Raises:
        KeyError: Si el pedido introducido no existe en el JSON (capturado
            internamente, se pide al usuario que lo intente de nuevo).
    """
    coord_entrega = datos_rutas['entrega']
    meta_entrega = crear_pose(coord_entrega['x'], coord_entrega['y'])

    print("\n--- SISTEMA DE TRANSPORTE CARRYBOT ACTIVADO (PEDIDO INDIVIDUAL) ---")
    print("Pedidos disponibles:")
    for nombre in datos_rutas['pedidos']:
        print(f"  - {nombre}")

    while True:
        nombre_pedido = input(
            "\nEscribe el nombre del pedido a recoger: "
        ).strip()

        try:
            if nombre_pedido not in datos_rutas['pedidos']:
                raise KeyError(
                    f"El pedido '{nombre_pedido}' no existe. "
                    f"Opciones: {list(datos_rutas['pedidos'].keys())}"
                )
            break
        except KeyError as err:
            print(f"[ERROR] {err}")

    coords = datos_rutas['pedidos'][nombre_pedido]
    meta_recogida = crear_pose(coords['x'], coords['y'])

    try:
        recoger_y_entregar(
            navigator, nombre_pedido, meta_recogida, meta_entrega
        )
    except NavegacionError as err:
        print(f"[ERROR] No se pudo completar la entrega: {err}")


# ─── Main ─────────────────────────────────────────────────────────────────────

def main(args=None):
    """
    Funcion principal de ejecucion del sistema de transporte Carrybot.

    Si se llama con --pedido <nombre>, ejecuta directamente ese pedido
    individual sin pedir input al usuario (modo web).
    Si se llama sin argumentos, muestra el menu interactivo.

    Args:
        args (list, optional): Argumentos de linea de comandos. Por defecto None.
    """
    # Parsear argumentos antes de inicializar rclpy
    parser = argparse.ArgumentParser(description='CarryBot — Sistema de entregas')
    parser.add_argument(
        '--pedido', type=str, default=None,
        help='Nombre del pedido a ejecutar directamente (modo web)'
    )
    parsed_args, _ = parser.parse_known_args()

    rclpy.init(args=args)
    navigator = BasicNavigator()

    try:
        navigator.waitUntilNav2Active()
        datos_rutas = cargar_coordenadas()

        # ── Modo web: pedido directo por argumento ────────────────────────────
        if parsed_args.pedido:
            nombre = parsed_args.pedido
            if nombre not in datos_rutas['pedidos']:
                print(
                    f"[ERROR] Pedido '{nombre}' no encontrado. "
                    f"Disponibles: {list(datos_rutas['pedidos'].keys())}"
                )
                return
            coord_entrega = datos_rutas['entrega']
            meta_entrega  = crear_pose(coord_entrega['x'], coord_entrega['y'])
            coords        = datos_rutas['pedidos'][nombre]
            meta_recogida = crear_pose(coords['x'], coords['y'])
            try:
                recoger_y_entregar(navigator, nombre, meta_recogida, meta_entrega)
            except NavegacionError as err:
                print(f"[ERROR] No se pudo completar la entrega: {err}")
            return

        # ── Modo interactivo: menu de seleccion ───────────────────────────────
        print("\n=== CARRYBOT — SISTEMA DE ENTREGAS ===")
        print("Modos disponibles:")
        print("  1) Ruta completa  (recoge todos los pedidos en orden)")
        print("  2) Pedido individual (elige un pedido concreto)")

        while True:
            modo = input("\nElige el modo (1 o 2): ").strip()
            if modo in ('1', '2'):
                break
            print("[ERROR] Opcion no valida. Escribe 1 o 2.")

        if modo == '1':
            modo_ruta_completa(navigator, datos_rutas)
        else:
            modo_pedido_individual(navigator, datos_rutas)

    except FileNotFoundError:
        print("[ERROR] No se pudo cargar el fichero de coordenadas.")
    except json.JSONDecodeError:
        print("[ERROR] El fichero de coordenadas tiene un formato invalido.")
    except KeyboardInterrupt:
        print("\n[INFO] Navegacion interrumpida por el usuario.")
    finally:
        print("\n--- VOLVIENDO A MODO ESPERA ---")
        rclpy.shutdown()


if __name__ == '__main__':
    main()