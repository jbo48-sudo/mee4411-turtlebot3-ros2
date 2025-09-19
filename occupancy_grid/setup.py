import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'occupancy_grid'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools', 'pytest'],
    zip_safe=True,
    maintainer='Philip Dames',
    maintainer_email='pdames@temple.edu',
    description='A package to create occupancy grid maps from text file descriptions',
    license='LGPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'block_visualizer = occupancy_grid.block_visualizer:main',
            'occupancy_grid_node = occupancy_grid.occupancy_grid_node:main'
        ],
    },
)
