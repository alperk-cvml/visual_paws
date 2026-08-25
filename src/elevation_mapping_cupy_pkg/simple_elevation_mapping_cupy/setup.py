from setuptools import setup
import os
from glob import glob

package_name = 'simple_elevation_mapping_cupy'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Wrapper package for elevation_mapping_cupy to launch with grid_map_visualization',
    license='MIT',
    entry_points={
        'console_scripts': [
            'simple_elevation_mapping_node = simple_elevation_mapping_cupy.simple_elevation_mapping_node:main',
        ],
    },
)
