from setuptools import find_packages, setup

package_name = 'waypoint_collector'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FYP Team',
    maintainer_email='Kevinlasnh@outlook.com',
    description='RViz 交互式航点收集器，支持多航点导航',
    license='MIT',
    entry_points={
        'console_scripts': [
            'waypoint_node = waypoint_collector.waypoint_node:main',
        ],
    },
)
