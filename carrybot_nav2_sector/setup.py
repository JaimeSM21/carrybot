import os
from glob import glob
from setuptools import setup

package_name = 'carrybot_nav2_sector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carrybot',
    maintainer_email='carrybot@todo.todo',
    description='Navegación por sectores del warehouse: ir al sector e inspeccionarlo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sector_navigator = carrybot_nav2_sector.sector_navigator:main',
        ],
    },
)
