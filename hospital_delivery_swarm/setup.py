import os
from glob import glob
from setuptools import setup

package_name = 'hospital_swarm'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'models', 'diff_drive_robot'), glob('models/diff_drive_robot/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pratyush',
    maintainer_email='you@example.com',
    description='Smart Hospital Medicine Delivery Swarm',
    license='MIT',
    scripts=[
        'scripts/hospital_navigator.py',
        'scripts/generate_robot_sdf.py',
    ],
)
