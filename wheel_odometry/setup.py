from setuptools import find_packages, setup

package_name = 'wheel_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Philip Dames',
    maintainer_email='pdames@temple.edu',
    description='A package to perform wheel odometry for a differential drive robot',
    license='LGPLv3',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
