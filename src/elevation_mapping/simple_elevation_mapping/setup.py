from setuptools import setup

package_name = 'simple_elevation_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Lightweight CPU-based elevation mapping node using grid_map.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'elevation_mapping_node = simple_elevation_mapping.elevation_mapping_node:main',
        ],
    },
)
