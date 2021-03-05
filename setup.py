import os
from glob import glob

from setuptools import setup

package_name = 'reachy_controllers'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'zoom_kurokesu>=1',
        'reachy_pyluos_hal',
    ],
    zip_safe=True,
    maintainer='Pollen-Robotics',
    maintainer_email='contact@pollen-robotics.com',
    description='ROS2 Foxy controllers for Reachy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_controller = reachy_controllers.joint_state_controller:main',
            'camera_publisher = reachy_controllers.camera_publisher:main',
            'camera_zoom_service = reachy_controllers.camera_zoom_service:main',
        ],
    },
)
