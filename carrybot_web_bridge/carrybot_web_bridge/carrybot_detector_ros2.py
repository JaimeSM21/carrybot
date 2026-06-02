#!/usr/bin/env python3
"""
carrybot_detector_ros2.py
=========================
Nodo ROS2 del detector de paquetes de CarryBot.

Se suscribe al topic /camera/image_raw (sensor_msgs/Image), detecta
cajas y códigos QR. La ventana OpenCV se gestiona en el hilo principal.

Botón "Actualizar" en pantalla: congela/descongela el frame actual.
"""

import json
import os
import time
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


# ── Configuración ─────────────────────────────────────────────────────────────

LOG_FILE = "/home/emilio/capturas_robot/qr_log.json"
CAPTURES_DIR = "/home/emilio/capturas_robot/"
SHOW_WINDOW  = True
WINDOW_TITLE = "CarryBot QR Detector  |  Q/ESC: salir  S: captura  C: limpiar"

MIN_BOX_AREA   = 4000
MAX_BOXES      = 5
ASPECT_MIN     = 0.35
ASPECT_MAX     = 3.5
CANNY_LOW      = 40
CANNY_HIGH     = 130
APPROX_EPSILON = 0.03

COLOR_BOX       = (0,   200,  50)
COLOR_QR_BORDER = (255, 200,   0)
COLOR_QR_TEXT   = (0,   255, 255)
COLOR_HUD       = (255, 255, 255)
COLOR_SAVED     = (0,   255, 128)

# Botón Actualizar
BTN_X, BTN_Y   = 10, 0      # esquina superior derecha (se calcula en draw_button)
BTN_W, BTN_H   = 140, 36
BTN_COLOR_IDLE = (50,  50,  50)
BTN_COLOR_HOV  = (80, 140, 200)
BTN_COLOR_FROZ = (30, 180,  80)
BTN_TEXT_COLOR = (255, 255, 255)


# ── Persistencia ──────────────────────────────────────────────────────────────

def load_log():
    if not os.path.exists(LOG_FILE):
        return []
    try:
        with open(LOG_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
            return data if isinstance(data, list) else []
    except (json.JSONDecodeError, OSError):
        return []


def save_entry(entry):
    log = load_log()
    log.append(entry)
    with open(LOG_FILE, "w", encoding="utf-8") as f:
        json.dump(log, f, ensure_ascii=False, indent=2)


def parse_qr(data):
    try:
        return json.loads(data)
    except (json.JSONDecodeError, ValueError):
        return None


# ── Detección ─────────────────────────────────────────────────────────────────

def detect_qr(frame, detector):
    data, bbox, _ = detector.detectAndDecode(frame)
    if data and bbox is not None:
        return data, bbox
    return None, None


def draw_qr(frame, bbox, data, saved):
    pts   = bbox[0].astype(int)
    label = data if len(data) <= 38 else data[:35] + "..."
    cv2.polylines(frame, [pts], True, COLOR_QR_BORDER, 2)
    cv2.putText(frame, f"QR: {label}",
                (pts[0][0], max(pts[0][1] - 10, 20)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, COLOR_QR_TEXT, 2)
    if saved:
        cv2.putText(frame, "GUARDADO",
                    (pts[0][0], max(pts[0][1] - 30, 38)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_SAVED, 2)


def detect_boxes(frame, annotated):
    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges   = cv2.Canny(blurred, CANNY_LOW, CANNY_HIGH)
    kernel  = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    edges   = cv2.dilate(edges, kernel, iterations=1)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    for cnt in sorted(contours, key=cv2.contourArea, reverse=True):
        if len(boxes) >= MAX_BOXES:
            break
        area = cv2.contourArea(cnt)
        if area < MIN_BOX_AREA:
            break
        peri   = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, APPROX_EPSILON * peri, True)
        if len(approx) != 4:
            continue
        x, y, w, h = cv2.boundingRect(approx)
        aspect = w / h if h > 0 else 0.0
        if not (ASPECT_MIN < aspect < ASPECT_MAX):
            continue
        box_id = len(boxes) + 1
        boxes.append({"x": int(x), "y": int(y), "w": int(w), "h": int(h)})
        cv2.rectangle(annotated, (x, y), (x + w, y + h), COLOR_BOX, 2)
        cv2.putText(annotated, f"Paquete #{box_id}  {w}x{h}px",
                    (x, max(y - 8, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, COLOR_BOX, 2)
    return boxes


def draw_hud(frame, n_boxes, qr_data, total_saved, fps):
    lines = [
        f"FPS: {fps:.1f}",
        f"Cajas: {n_boxes}",
        f"QR: {'SI' if qr_data else 'no'}",
        f"Guardados: {total_saved}",
        "Q/ESC salir | S captura | C limpiar",
    ]
    overlay = frame.copy()
    cv2.rectangle(overlay, (8, 8), (280, 18 + len(lines) * 22), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
    for i, line in enumerate(lines):
        color = COLOR_SAVED if (i == 2 and qr_data) else COLOR_HUD
        cv2.putText(frame, line, (14, 26 + i * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 1)


# ── Botón Actualizar ──────────────────────────────────────────────────────────

def get_btn_rect(frame_w):
    """Devuelve (x1, y1, x2, y2) del botón alineado a la derecha."""
    x1 = frame_w - BTN_W - 10
    y1 = 10
    x2 = x1 + BTN_W
    y2 = y1 + BTN_H
    return x1, y1, x2, y2


def draw_button(frame, frozen, hover):
    """
    Dibuja el botón 'Actualizar' / 'Reanudar' en la esquina superior derecha.

    Args:
        frame:  Imagen BGR sobre la que se dibuja.
        frozen: Si True, la imagen está congelada → mostrar 'Reanudar'.
        hover:  Si True, el ratón está encima → color resaltado.
    """
    h, w = frame.shape[:2]
    x1, y1, x2, y2 = get_btn_rect(w)

    if frozen:
        color = BTN_COLOR_FROZ
        label = "  Reanudar"
    elif hover:
        color = BTN_COLOR_HOV
        label = "  Actualizar"
    else:
        color = BTN_COLOR_IDLE
        label = "  Actualizar"

    # Fondo del botón con transparencia
    overlay = frame.copy()
    cv2.rectangle(overlay, (x1, y1), (x2, y2), color, -1)
    cv2.addWeighted(overlay, 0.75, frame, 0.25, 0, frame)

    # Borde
    cv2.rectangle(frame, (x1, y1), (x2, y2), (200, 200, 200), 1)

    # Icono de flecha circular (Unicode no disponible en OpenCV → dibujamos triángulo)
    icon_cx = x1 + 18
    icon_cy = (y1 + y2) // 2
    if frozen:
        # Triángulo play (reanudar)
        pts = np.array([
            [icon_cx - 5, icon_cy - 7],
            [icon_cx - 5, icon_cy + 7],
            [icon_cx + 7, icon_cy],
        ], np.int32)
        cv2.fillPoly(frame, [pts], BTN_TEXT_COLOR)
    else:
        # Círculo con flecha (actualizar)
        cv2.circle(frame, (icon_cx, icon_cy), 7, BTN_TEXT_COLOR, 2)
        cv2.arrowedLine(frame,
                        (icon_cx + 4, icon_cy - 7),
                        (icon_cx + 9, icon_cy - 2),
                        BTN_TEXT_COLOR, 2, tipLength=0.5)

    # Texto
    cv2.putText(frame, label,
                (x1 + 24, y1 + BTN_H // 2 + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, BTN_TEXT_COLOR, 1)


def is_over_button(mx, my, frame_w):
    """Devuelve True si el cursor (mx, my) está dentro del botón."""
    x1, y1, x2, y2 = get_btn_rect(frame_w)
    return x1 <= mx <= x2 and y1 <= my <= y2


# ── Nodo ROS2 ─────────────────────────────────────────────────────────────────

class QRDetectorNode(Node):
    """
    Nodo ROS2 que procesa imágenes de /camera/image_raw en un hilo
    secundario y comparte el frame anotado con el hilo principal
    mediante un lock.
    """

    def __init__(self):
        super().__init__('carrybot_qr_detector')

        self.bridge      = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()

        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.publisher = self.create_publisher(String, '/carrybot/qr_detect', 10)

        self.seen_qrs:    set   = set()
        self.saved_until: float = 0.0
        self.fps        = 0.0
        self.fps_timer  = time.time()
        self.fps_count  = 0

        # Frame compartido entre hilo ROS y hilo principal
        self.latest_frame  = None
        self.frame_lock    = threading.Lock()

        self.get_logger().info('QR Detector iniciado — suscrito a /camera/image_raw')
        self.get_logger().info(f'QRs guardados en: {os.path.abspath(LOG_FILE)}')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as err:
            self.get_logger().error(f'Error convirtiendo imagen: {err}')
            return

        annotated = frame.copy()
        boxes     = detect_boxes(frame, annotated)
        qr_data, qr_bbox = detect_qr(frame, self.qr_detector)

        if qr_data and qr_data not in self.seen_qrs:
            self.seen_qrs.add(qr_data)
            parsed = parse_qr(qr_data)
            entry  = {
                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "qr_raw":    qr_data,
                "qr_parsed": parsed,
            }
            try:
                save_entry(entry)
                self.saved_until = time.time() + 3.0
                ros_msg      = String()
                ros_msg.data = json.dumps(entry, ensure_ascii=False)
                self.publisher.publish(ros_msg)
                self.get_logger().info(f'[QR DETECTADO] {qr_data}')
                if parsed:
                    for k, v in parsed.items():
                        self.get_logger().info(f'               {k}: {v}')
            except OSError as exc:
                self.get_logger().error(f'No se pudo guardar: {exc}')

        if qr_data and qr_bbox is not None:
            draw_qr(annotated, qr_bbox, qr_data, saved=time.time() < self.saved_until)

        self.fps_count += 1
        elapsed = time.time() - self.fps_timer
        if elapsed >= 1.0:
            self.fps       = self.fps_count / elapsed
            self.fps_count = 0
            self.fps_timer = time.time()

        draw_hud(annotated, len(boxes), qr_data, len(load_log()), self.fps)

        with self.frame_lock:
            self.latest_frame = annotated

    def get_latest_frame(self):
        with self.frame_lock:
            if self.latest_frame is not None:
                return self.latest_frame.copy()
        return None


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    """
    Hilo principal : ventana OpenCV + botón Actualizar.
    Hilo secundario: rclpy.spin() con el nodo ROS2.

    El botón alterna entre modo en vivo (live) y modo congelado (frozen).
    En modo congelado se muestra el último frame capturado sin actualizarlo.
    Al pulsar de nuevo se reanuda el stream en vivo.
    """
    rclpy.init(args=args)
    node = QRDetectorNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Estado del botón
    frozen       = False   # True = imagen congelada
    frozen_frame = None    # Copia guardada cuando se congela
    mouse_x      = 0
    mouse_y      = 0

    def on_mouse(event, mx, my, flags, param):
        """Callback de ratón: detecta hover y clic sobre el botón."""
        nonlocal frozen, frozen_frame, mouse_x, mouse_y
        mouse_x, mouse_y = mx, my

        if event == cv2.EVENT_LBUTTONDOWN:
            current_frame = param.get('frame')
            if current_frame is None:
                return
            h, w = current_frame.shape[:2]
            if is_over_button(mx, my, w):
                if not frozen:
                    # Congelar: guardar copia del frame actual
                    frozen       = True
                    frozen_frame = current_frame.copy()
                    node.get_logger().info('Imagen congelada.')
                else:
                    # Reanudar stream en vivo
                    frozen       = False
                    frozen_frame = None
                    node.get_logger().info('Stream reanudado.')

    # Diccionario compartido con el callback del ratón para pasar el frame
    mouse_param = {'frame': None}

    if SHOW_WINDOW:
        cv2.namedWindow(WINDOW_TITLE)
        cv2.setMouseCallback(WINDOW_TITLE, on_mouse, mouse_param)

        while rclpy.ok():
            # Elegir qué frame mostrar
            if frozen and frozen_frame is not None:
                display = frozen_frame.copy()
            else:
                live = node.get_latest_frame()
                if live is not None:
                    display = live
                else:
                    display = None

            if display is not None:
                h, w = display.shape[:2]
                hover = is_over_button(mouse_x, mouse_y, w)
                draw_button(display, frozen, hover)

                # Actualizar referencia para el callback del ratón
                mouse_param['frame'] = display

                cv2.imshow(WINDOW_TITLE, display)

            key = cv2.waitKey(1) & 0xFF

            if key in (ord('q'), ord('Q'), 27):
                break
            elif key in (ord('s'), ord('S')):
                if display is not None:
                    os.makedirs(CAPTURES_DIR, exist_ok=True)
                    filename = os.path.join(CAPTURES_DIR, f"captura_{time.strftime('%Y%m%d_%H%M%S')}.jpg")
                    cv2.imwrite(filename, display)
                    node.get_logger().info(f'Captura guardada: {filename}')
            elif key in (ord('c'), ord('C')):
                node.seen_qrs.clear()
                node.get_logger().info('QRs de sesión limpiados.')
            elif key in (ord('r'), ord('R')):
                # Atajo de teclado alternativo para congelar/reanudar
                if not frozen:
                    live = node.get_latest_frame()
                    if live is not None:
                        frozen       = True
                        frozen_frame = live
                        node.get_logger().info('Imagen congelada (tecla R).')
                else:
                    frozen       = False
                    frozen_frame = None
                    node.get_logger().info('Stream reanudado (tecla R).')
    else:
        spin_thread.join()

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()