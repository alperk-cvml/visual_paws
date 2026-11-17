from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pointcloud_scale_correction'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alper Kucukkomurler',
    maintainer_email='kucukkomurler.alper@gmail.com',
    description='Corrects monocular depth point clouds using the '
                'ground-plane scale estimate; pluggable error models',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'scale_correction = '
            'pointcloud_scale_correction.scale_correction_node:main',
        ],
    },
)
