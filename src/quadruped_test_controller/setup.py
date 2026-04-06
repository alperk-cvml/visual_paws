from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'quadruped_test_controller'

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
    description='Dummy W/A/S/D keyboard controller for the quadruped robot (test only)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'keyboard_controller = quadruped_test_controller.keyboard_controller_node:main',
        ],
    },
)
