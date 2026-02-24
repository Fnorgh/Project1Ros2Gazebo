from setuptools import setup
from glob import glob
import os

package_name = 'project_1'

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
        (os.path.join('share', package_name, 'worlds'),
	    glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gibs0113',
    maintainer_email='gibs0113@example.com',
    description='Project 1 Simulation Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'behavior_mux = project_1.behavior_mux:main',
            'bumper_stop  = project_1.behavior_bumper_stop:main',
            'reactive_controller = project_1.reactive_controller:main',
        ]
    },
)

