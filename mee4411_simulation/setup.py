import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mee4411_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'param'), glob('param/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools', 'collision'],
    zip_safe=True,
    maintainer='Philip Dames',
    maintainer_email='pdames@temple.edu',
    description='A package that holds the main configuration and launch files for this project',
    license='LGPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'collision_detector = mee4411_simulation.collision_detection:main',
            'sensor_republisher = mee4411_simulation.repub_simulated_sensors:main',
            'test_frames = mee4411_simulation.test_frames:main',
        ],
    },
)
