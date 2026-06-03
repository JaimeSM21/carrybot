#!/usr/bin/env python3
"""
package_detector.py — Nodo de visión artificial para CarryBot.

Detecta cajas (contornos rectangulares) y códigos QR en el stream de la cámara.

Suscribe:
    /camera/image_raw   (sensor_msgs/Image)

Publica:
    /package/detection  (std_msgs/String)  — JSON con resultados de detección
    /camera/processed   (sensor_msgs/Image) — Frame anotado con bounding boxes

Uso:
    ros2 run carrybot_vision package_detector

Formato JSON de /package/detection:
    {
        "box_detected": true,
        "boxes": [{"x": 120, "y": 80, "w": 200, "h": 150}],
        "qr_detected": false,
        "qr_data":   null,
        "qr_parsed": null
    }

Lógica de registro en detecciones.json:
    Cada vez que se detecta una caja se busca el siguiente id libre
    (PKG-001, PKG-002, ...) que no exista ya en el fichero y se añade
    una entrada nueva. El cooldown evita registrar el mismo frame
    continuamente.
"""

import json
import os
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class PackageDetectorError(Exception):
    """Error específico del nodo de detección de paquetes."""


class PackageDetector(Node):
    """
    Nodo ROS2 que detecta paquetes (cajas) y códigos QR en la cámara del robot.
    """

    # ── Parámetros de detección ───────────────────────────────────────────────
    MIN_BOX_AREA: int     = 3000
    MAX_BOXES: int        = 5
    ASPECT_MIN: float     = 0.4
    ASPECT_MAX: float     = 3.0
    CANNY_LOW: int        = 40
    CANNY_HIGH: int       = 130
    APPROX_EPSILON: float = 0.03

    # ── Log de detecciones ────────────────────────────────────────────────────
    LOG_FILE: str       = '/home/emilio/turtlebot3_ws/src/carrybot/carrybot_vision/detecciones.json'
    LOG_COOLDOWN: float = 5.0   # segundos mínimos entre registros

    # ── Datos base de cada paquete ────────────────────────────────────────────
    # El id se asigna automáticamente (siguiente libre en el log).
    # El resto de campos son los mismos para todos los paquetes de prueba.
    PACKAGE_BASE: dict = {
        "dest":     "Estanteria1",
        "weight":   "2.5kg",
        "priority": "normal",
    }

    # ── Colores BGR ───────────────────────────────────────────────────────────
    COLOR_BOX: tuple = (0, 200, 50)
    COLOR_QR: tuple  = (0, 255, 0)

    def __init__(self) -> None:
        super().__init__('package_detector')
        self.bridge            = CvBridge()
        self.qr_detector       = cv2.QRCodeDetector()
        self._last_log_time: float = 0.0

        self.sub_image = self.create_subscription(
            Image, '/camera/image_raw', self._image_callback, 10,
        )
        self.pub_detection = self.create_publisher(String, '/package/detection', 10)
        self.pub_processed = self.create_publisher(Image,  '/camera/processed',  10)

        self.get_logger().info(
            'PackageDetector iniciado — escuchando /camera/image_raw'
        )

    # ── Callback principal ────────────────────────────────────────────────────
    def _image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge error: {exc}')
            return

        detection, annotated = self._process_frame(frame)

        det_msg      = String()
        det_msg.data = json.dumps(detection)
        self.pub_detection.publish(det_msg)

        try:
            proc_msg        = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            proc_msg.header = msg.header
            self.pub_processed.publish(proc_msg)
        except Exception as exc:
            self.get_logger().warn(f'Error publicando imagen procesada: {exc}')

    # ── Procesado de frame ────────────────────────────────────────────────────
    def _process_frame(self, frame: np.ndarray) -> tuple[dict, np.ndarray]:
        if frame is None or frame.size == 0:
            raise PackageDetectorError('Frame vacío o inválido recibido.')

        annotated = frame.copy()
        result: dict = {
            'box_detected': False,
            'boxes':        [],
            'qr_detected':  False,
            'qr_data':      None,
            'qr_parsed':    None,
        }

        # 1. Intentar leer QR
        qr_data, qr_bbox = self._detect_qr(annotated)
        if qr_data:
            result['qr_detected'] = True
            result['qr_data']     = qr_data
            result['qr_parsed']   = self._parse_qr(qr_data)
            self._draw_qr(annotated, qr_bbox, qr_data)

        # 2. Detectar cajas
        boxes = self._detect_boxes(frame, annotated)
        if boxes:
            result['box_detected'] = True
            result['boxes']        = boxes
            pkg_entry = self._log_detection()   # devuelve la entrada guardada o None
            if pkg_entry:
                result['qr_detected'] = True
                result['qr_data']     = pkg_entry['qr_raw']
                result['qr_parsed']   = pkg_entry['qr_parsed']

        return result, annotated

    # ── Log de detecciones ────────────────────────────────────────────────────
    def _load_log(self) -> list:
        """
        Carga el fichero JSON de detecciones.

        Returns:
            Lista de entradas existentes, o lista vacía si no existe el fichero.
        """
        if not os.path.exists(self.LOG_FILE):
            return []
        try:
            with open(self.LOG_FILE, 'r', encoding='utf-8') as fh:
                data = json.load(fh)
                return data if isinstance(data, list) else []
        except (json.JSONDecodeError, OSError):
            return []

    def _next_free_id(self, log: list) -> str:
        """
        Calcula el siguiente id libre que no exista en el log.

        Recorre PKG-001, PKG-002, PKG-003... hasta encontrar uno ausente.

        Args:
            log: Lista de entradas ya guardadas.

        Returns:
            String con el id libre, p.ej. 'PKG-003'.
        """
        ids_en_log = {
            e.get('qr_parsed', {}).get('id')
            for e in log
            if isinstance(e.get('qr_parsed'), dict)
        }
        n = 1
        while True:
            candidate = f'{n:3d}'
            if candidate not in ids_en_log:
                return candidate
            n += 1

    def _log_detection(self) -> None:
        """
        Añade una entrada al log con el siguiente id libre.

        Respeta el cooldown de LOG_COOLDOWN segundos para no registrar
        el mismo frame varias veces mientras la caja permanece en pantalla.
        """
        now = time.time()
        if now - self._last_log_time < self.LOG_COOLDOWN:
            return None

        self._last_log_time = now

        # Leer log y calcular siguiente id libre
        log        = self._load_log()
        new_id     = self._next_free_id(log)
        package    = {"id": new_id, **self.PACKAGE_BASE}
        qr_raw     = json.dumps(package, ensure_ascii=False, separators=(',', ':'))

        entry = {
            "timestamp": datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
            "qr_raw":    qr_raw,
            "qr_parsed": package,
        }

        log.append(entry)

        try:
            os.makedirs(os.path.dirname(self.LOG_FILE), exist_ok=True)
            with open(self.LOG_FILE, 'w', encoding='utf-8') as fh:
                json.dump(log, fh, ensure_ascii=False, indent=2)
            self.get_logger().info(
                f'[LOG] {new_id} guardado → {self.LOG_FILE}  (total: {len(log)})')
            return entry
        
        except OSError as exc:
            self.get_logger().warn(f'No se pudo escribir el log: {exc}')
            return None

    # ── Detección QR ─────────────────────────────────────────────────────────
    def _detect_qr(self, frame: np.ndarray) -> tuple[str | None, np.ndarray | None]:
        for img in self._build_candidates(frame):
            data, bbox, _ = self.qr_detector.detectAndDecode(img)
            if data and bbox is not None:
                return data, bbox
        return None, None

    def _build_candidates(self, frame: np.ndarray) -> list[np.ndarray]:
        results = [frame]

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        results.append(cv2.cvtColor(otsu, cv2.COLOR_GRAY2BGR))

        upscaled = cv2.resize(frame, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
        results.append(upscaled)

        gray_up = cv2.cvtColor(upscaled, cv2.COLOR_BGR2GRAY)
        _, otsu_up = cv2.threshold(gray_up, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        results.append(cv2.cvtColor(otsu_up, cv2.COLOR_GRAY2BGR))

        return results

    def _draw_qr(self, frame: np.ndarray, bbox: np.ndarray, data: str) -> None:
        pts   = bbox[0].astype(int)
        label = data if len(data) <= 35 else data[:32] + '...'
        cv2.polylines(frame, [pts], True, self.COLOR_QR, 2)
        cv2.putText(
            frame, f'QR: {label}',
            (pts[0][0], max(pts[0][1] - 8, 20)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, self.COLOR_QR, 2,
        )

    @staticmethod
    def _parse_qr(data: str) -> dict | None:
        try:
            return json.loads(data)
        except (json.JSONDecodeError, ValueError):
            return None

    # ── Detección de cajas ────────────────────────────────────────────────────
    def _detect_boxes(self, original: np.ndarray, annotated: np.ndarray) -> list[dict]:
        gray    = cv2.cvtColor(original, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges   = cv2.Canny(blurred, self.CANNY_LOW, self.CANNY_HIGH)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        edges  = cv2.dilate(edges, kernel, iterations=1)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        boxes: list[dict] = []

        for cnt in sorted(contours, key=cv2.contourArea, reverse=True):
            if len(boxes) >= self.MAX_BOXES:
                break

            area = cv2.contourArea(cnt)
            if area < self.MIN_BOX_AREA:
                break

            peri   = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, self.APPROX_EPSILON * peri, True)

            if len(approx) != 4:
                continue

            x, y, w, h = cv2.boundingRect(approx)
            aspect = w / h if h > 0 else 0.0

            if not (self.ASPECT_MIN < aspect < self.ASPECT_MAX):
                continue

            box_id = len(boxes) + 1
            boxes.append({'x': int(x), 'y': int(y), 'w': int(w), 'h': int(h)})

            cv2.rectangle(annotated, (x, y), (x + w, y + h), self.COLOR_BOX, 2)
            cv2.putText(
                annotated,
                f'Paquete #{box_id}',
                (x, max(y - 8, 20)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, self.COLOR_BOX, 2,
            )

        return boxes


# ── Punto de entrada ──────────────────────────────────────────────────────────
def main(args=None) -> None:
    rclpy.init(args=args)
    node = PackageDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()