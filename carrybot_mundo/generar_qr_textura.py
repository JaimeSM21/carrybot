#!/usr/bin/env python3
"""
generar_qr_textura.py
=====================
Coge un QR ya existente de la carpeta 'qr_fuente/' y lo prepara como
textura PNG para el modelo Gazebo de la caja.

Uso:
    1. Pon tu imagen QR en:  carrybot_mundo/qr_fuente/
       (cualquier nombre, cualquier formato: PNG, JPG, JPEG, BMP, WEBP)
    2. Ejecuta:
           cd ~/turtlebot3_ws/src/carrybot/carrybot_mundo
           python3 generar_qr_textura.py

El fichero resultante se guarda en:
    models/caja_qr/materials/textures/qr_pkg001.png

Dependencias:
    pip install Pillow
"""

import os
import sys

# ── Rutas ─────────────────────────────────────────────────────────────────────

BASE_DIR    = os.path.dirname(os.path.abspath(__file__))

# Carpeta donde el usuario deja su imagen QR
SOURCE_DIR  = os.path.join(BASE_DIR, "qr_fuente")

# Destino final que leerá Gazebo
OUTPUT_DIR  = os.path.join(BASE_DIR, "models", "caja_qr", "materials", "textures")
OUTPUT_FILE = os.path.join(OUTPUT_DIR, "QR-prueba1.png")

# Tamaño final de la textura (px). 512 es suficiente para Gazebo.
TEXTURE_SIZE = 512

# Formatos de imagen aceptados
VALID_EXTENSIONS = {".png", ".jpg", ".jpeg", ".bmp", ".webp"}


# ── Helpers ───────────────────────────────────────────────────────────────────

def find_source_image() -> str:
    """
    Busca la primera imagen válida dentro de SOURCE_DIR.

    Returns:
        Ruta absoluta al fichero encontrado.

    Raises:
        SystemExit: Si la carpeta no existe o no contiene ninguna imagen válida.
    """
    if not os.path.isdir(SOURCE_DIR):
        print(f"[ERROR] No existe la carpeta de origen: {SOURCE_DIR}")
        print( "        Créala y pon dentro tu imagen QR.")
        sys.exit(1)

    candidates = [
        f for f in sorted(os.listdir(SOURCE_DIR))
        if os.path.splitext(f)[1].lower() in VALID_EXTENSIONS
    ]

    if not candidates:
        print(f"[ERROR] No se encontró ninguna imagen en: {SOURCE_DIR}")
        print(f"        Formatos aceptados: {', '.join(VALID_EXTENSIONS)}")
        sys.exit(1)

    if len(candidates) > 1:
        print(f"[AVISO] Hay {len(candidates)} imágenes en la carpeta.")
        print(f"        Se usará la primera por orden alfabético: {candidates[0]}")
        print(f"        Las demás se ignoran: {', '.join(candidates[1:])}\n")

    return os.path.join(SOURCE_DIR, candidates[0])


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    try:
        from PIL import Image
    except ImportError:
        print("[ERROR] Falta Pillow. Instálalo con:")
        print("        pip install Pillow")
        sys.exit(1)

    source_path = find_source_image()
    print(f"[INFO] Imagen origen: {source_path}")

    # Abrir y convertir a RGB (por si viene con canal alpha o en escala de grises)
    try:
        qr_img = Image.open(source_path).convert("RGB")
    except Exception as exc:
        print(f"[ERROR] No se pudo abrir la imagen: {exc}")
        sys.exit(1)

    original_size = qr_img.size
    print(f"[INFO] Tamaño original: {original_size[0]}x{original_size[1]} px")

    # Fondo marrón cartón con el QR centrado
    background = Image.new("RGB", (TEXTURE_SIZE, TEXTURE_SIZE), (180, 120, 60))
    qr_size    = int(TEXTURE_SIZE * 0.80)

    # NEAREST para QR (evita suavizado que difumina los módulos del código)
    qr_resized = qr_img.resize((qr_size, qr_size), Image.NEAREST)

    offset = (TEXTURE_SIZE - qr_size) // 2
    background.paste(qr_resized, (offset, offset))

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    background.save(OUTPUT_FILE, format="PNG")

    print(f"[OK] Textura guardada en:\n     {OUTPUT_FILE}")
    print("\nSIGUIENTE PASO — Compila el workspace:")
    print("    cd ~/turtlebot3_ws")
    print("    colcon build --packages-select carrybot_mundo")
    print("    source install/setup.bash")


if __name__ == "__main__":
    main()