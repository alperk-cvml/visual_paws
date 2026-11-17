from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'quadruped_gait_controller'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alper Kucukkomurler',
    maintainer_email='kucukkomurler.alper@gmail.com',
    description='Spot Micro Bezier-swing gait controller (trot / walk) with whole-body IK',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gait_controller = quadruped_gait_controller.gait_node:main',
        ],
    },
)
