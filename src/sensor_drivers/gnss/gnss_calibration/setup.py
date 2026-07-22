from setuptools import find_packages, setup

package_name = 'gnss_calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gnss_calibration_launch.py']),
        ('share/' + package_name + '/config', ['config/calibration_points.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mup10',
    maintainer_email='mup10@todo.todo',
    description='GNSS Calibration Package',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'gnss_calibration_node = gnss_calibration.gnss_calibration_node:main',
            'gps_anchor_localizer_node = gnss_calibration.gps_anchor_localizer_node:main',
        ],
    },
)
