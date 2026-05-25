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
    ros2 run carrybot_web_bridge package_detector

Formato JSON de /package/detection:
    {
        "box_detected": true,
        "boxes": [{"x": 120, "y": 80, "w": 200, "h": 150}],
        "qr_detected": true,
        "qr_data": '{"id":"PKG-001","dest":"Estanteria1","weight":"2.5kg"}',
        "qr_parsed": {"id": "PKG-001", "dest": "Estanteria1", "weight": "2.5kg"}
    }

Formato QR recomendado (JSON):
    {"id": "PKG-001", "dest": "Estanteria1", "weight": "2.5kg"}
"""

import json

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

    La detección de cajas se basa en búsqueda de contornos rectangulares con
    Canny + approxPolyDP. El QR se decodifica con cv2.QRCodeDetector, incluido
    en OpenCV sin dependencias adicionales.
    """

    # ── Parámetros de detección ───────────────────────────────────────────────
    MIN_BOX_AREA: int   = 3000   # px² mínimos para aceptar un contorno como caja
    MAX_BOXES: int      = 5      # máximo de cajas reportadas por frame
    ASPECT_MIN: float   = 0.4    # ratio w/h mínimo (evita líneas y palos)
    ASPECT_MAX: float   = 3.0    # ratio w/h máximo (evita franjas horizontales)
    CANNY_LOW: int      = 40     # umbral bajo de Canny
    CANNY_HIGH: int     = 130    # umbral alto de Canny
    APPROX_EPSILON: float = 0.03 # tolerancia approxPolyDP (% del perímetro)

    # ── Colores BGR ───────────────────────────────────────────────────────────
    COLOR_BOX: tuple = (0, 200, 50)    # verde — caja detectada
    COLOR_QR: tuple  = (0, 255, 0)     # verde claro — QR detectado

    def __init__(self) -> None:
        super().__init__('package_detector')
        self.bridge       = CvBridge()
        self.qr_detector  = cv2.QRCodeDetector()

        self.sub_image = self.create_subscription(
            Image,
            '/camera/image_raw',
            self._image_callback,
            10,
        )
        self.pub_detection = self.create_publisher(String, '/package/detection', 10)
        self.pub_processed = self.create_publisher(Image,  '/camera/processed',  10)

        self.get_logger().info(
            'PackageDetector iniciado — escuchando /camera/image_raw'
        )

    # ── Callback principal ────────────────────────────────────────────────────
    def _image_callback(self, msg: Image) -> None:
        """
        Recibe cada frame, ejecuta detección y publica resultados.

        Args:
            msg: Mensaje ROS2 Image desde /camera/image_raw.
        """
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:                            # noqa: BLE001
            self.get_logger().warn(f'cv_bridge error: {exc}')
            return

        detection, annotated = self._process_frame(frame)

        # Publicar JSON de detección
        det_msg      = String()
        det_msg.data = json.dumps(detection)
        self.pub_detection.publish(det_msg)

        # Publicar imagen anotada
        try:
            proc_msg        = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            proc_msg.header = msg.header
            self.pub_processed.publish(proc_msg)
        except Exception as exc:                            # noqa: BLE001
            self.get_logger().warn(f'Error publicando imagen procesada: {exc}')

    # ── Procesado de frame ────────────────────────────────────────────────────
    def _process_frame(
        self, frame: np.ndarray
    ) -> tuple[dict, np.ndarray]:
        """
        Detecta cajas y QR en el frame y anota el resultado.

        Args:
            frame: Imagen BGR de OpenCV.

        Returns:
            Tuple (dict con resultados de detección, imagen anotada BGR).

        Raises:
            PackageDetectorError: Si el frame tiene un formato inesperado.
        """
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

        # 1. Detectar QR primero (más prioritario visualmente)
        qr_data, qr_bbox = self._detect_qr(annotated)
        if qr_data:
            result['qr_detected'] = True
            result['qr_data']     = qr_data
            result['qr_parsed']   = self._parse_qr(qr_data)
            self._draw_qr(annotated, qr_bbox, qr_data)

        # 2. Detectar cajas por contornos
        boxes = self._detect_boxes(frame, annotated)
        if boxes:
            result['box_detected'] = True
            result['boxes']        = boxes

        return result, annotated

    # ── Detección QR ─────────────────────────────────────────────────────────
    def _detect_qr(
        self, frame: np.ndarray
    ) -> tuple[str | None, np.ndarray | None]:
        """
        Intenta leer el QR con múltiples estrategias de preprocesado.
        Orden: imagen original → escala de grises umbralizada → upscale x2.
        """
        candidates = self._build_candidates(frame)
        for img in candidates:
            data, bbox, _ = self.qr_detector.detectAndDecode(img)
            if data and bbox is not None:
                return data, bbox
        return None, None


    def _build_candidates(self, frame: np.ndarray) -> list[np.ndarray]:
        """
        Genera variantes del frame para maximizar la probabilidad de lectura.

        Returns:
            Lista de imágenes BGR en orden de preferencia.
        """
        results = []

        # 1. Frame original
        results.append(frame)

        # 2. Escala de grises con threshold de Otsu (limpia ruido de textura)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, otsu = cv2.threshold(
            gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )
        results.append(cv2.cvtColor(otsu, cv2.COLOR_GRAY2BGR))

        # 3. Upscale x2 con interpolación cúbica (más píxeles para el detector)
        upscaled = cv2.resize(
            frame, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC
        )
        results.append(upscaled)

        # 4. Upscale x2 + Otsu (combinación más agresiva)
        gray_up = cv2.cvtColor(upscaled, cv2.COLOR_BGR2GRAY)
        _, otsu_up = cv2.threshold(
            gray_up, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )
        results.append(cv2.cvtColor(otsu_up, cv2.COLOR_GRAY2BGR))

        return results

    def _draw_qr(
        self, frame: np.ndarray, bbox: np.ndarray, data: str
    ) -> None:
        """
        Dibuja el contorno del QR y la etiqueta sobre el frame.

        Args:
            frame: Imagen sobre la que se dibuja (modificada in-place).
            bbox:  Coordenadas devueltas por QRCodeDetector.
            data:  Texto decodificado del QR.
        """
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
        """
        Intenta parsear el contenido del QR como JSON de paquete.

        El formato esperado es:
            {"id": "PKG-001", "dest": "Estanteria1", "weight": "2.5kg"}

        Args:
            data: Cadena decodificada del QR.

        Returns:
            Dict con los campos del paquete, o None si no es JSON válido.
        """
        try:
            return json.loads(data)
        except (json.JSONDecodeError, ValueError):
            return None

    # ── Detección de cajas ────────────────────────────────────────────────────
    def _detect_boxes(
        self, original: np.ndarray, annotated: np.ndarray
    ) -> list[dict]:
        """
        Detecta contornos rectangulares (cajas) mediante Canny + approxPolyDP.

        Algoritmo:
            1. Convertir a escala de grises y aplicar Gaussian Blur.
            2. Detectar bordes con Canny.
            3. Dilatar ligeramente para cerrar contornos abiertos.
            4. Filtrar contornos con 4 vértices, área mínima y ratio de aspecto.

        Args:
            original:  Frame sin anotar (usado sólo para lectura).
            annotated: Frame sobre el que se dibujan las anotaciones.

        Returns:
            Lista de dicts {'x', 'y', 'w', 'h'} por cada caja detectada.
        """
        gray    = cv2.cvtColor(original, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges   = cv2.Canny(blurred, self.CANNY_LOW, self.CANNY_HIGH)

        # Cerrar pequeñas brechas en los bordes
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        edges  = cv2.dilate(edges, kernel, iterations=1)

        contours, _ = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        boxes: list[dict] = []

        # Ordenar de mayor a menor para detectar primero las cajas grandes
        for cnt in sorted(contours, key=cv2.contourArea, reverse=True):
            if len(boxes) >= self.MAX_BOXES:
                break

            area = cv2.contourArea(cnt)
            if area < self.MIN_BOX_AREA:
                break  # Ordenado por área desc → ya no hay contornos más grandes

            peri  = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, self.APPROX_EPSILON * peri, True)

            if len(approx) != 4:
                continue  # No es rectangular

            x, y, w, h = cv2.boundingRect(approx)
            aspect = w / h if h > 0 else 0.0

            if not (self.ASPECT_MIN < aspect < self.ASPECT_MAX):
                continue  # Forma demasiado alargada o demasiado cuadrada extrema

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
    """Inicializa ROS2 y lanza el nodo PackageDetector."""
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