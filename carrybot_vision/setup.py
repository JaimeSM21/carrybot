from setuptools import setup

package_name = 'carrybot_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Registro del paquete en ament (obligatorio)
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Metadatos del paquete
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Emilio Sanchez Granado',
    maintainer_email='tu_email@ejemplo.com',
    description='Visión artificial de CarryBot: detección de cajas y QR.',
    license='MIT',
    entry_points={
        'console_scripts': [
            # ros2 run carrybot_vision package_detector
            'package_detector = carrybot_vision.package_detector:main',
        ],
    },
)