from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'assem6_hardware'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='todo@email.com',
    description='Hardware interface for Assem6 barista robot on Raspberry Pi 4',
    license='BSD',
    entry_points={
        'console_scripts': [
            'servo_bridge = assem6_hardware.servo_bridge_node:main',
            'servo_test = assem6_hardware.servo_test:main',
            'calibrate_servos = assem6_hardware.calibrate_servos:main',
        ],
    },
)
