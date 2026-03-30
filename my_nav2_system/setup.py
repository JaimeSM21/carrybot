import os
from glob import glob
from setuptools import setup

package_name = 'my_nav2_system'

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
        (os.path.join('share', package_name, 'param'),
            glob('param/*.yaml')),
        (os.path.join('share', package_name, 'map'),
            glob('map/*.pgm')),
        (os.path.join('share', package_name, 'map'),
            glob('map/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emilio',
    maintainer_email='emilio@todo.todo',
    description='Sistema de navegación Nav2 para el warehouse',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'initial_pose_pub = my_nav2_system.initial_pose_pub:main',
        ],
    },
)
