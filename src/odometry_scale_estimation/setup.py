from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'odometry_scale_estimation'

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
    description='Estimates the monocular depth scale by comparing a metric '
                'odometry stream (leg kinematics + IMU) against a '
                'scale-bearing one (KISS-ICP on the monodepth cloud) over a '
                'sliding window; pluggable scalar/affine fit models',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'odom_scale = '
            'odometry_scale_estimation.odom_scale_node:main',
        ],
    },
)
