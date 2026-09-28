from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'monocular_depth'

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
    maintainer='you',
    maintainer_email='you@yourdomain.com',
    description='Monocular metric depth estimation using Metric3D V2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'depth_node = monocular_depth.depth_node:main',
        ],
    },
)
