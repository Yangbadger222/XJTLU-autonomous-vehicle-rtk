from setuptools import find_packages, setup

package_name = 'frc_nodes'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FYP Team',
    maintainer_email='Kevinlasnh@outlook.com',
    description='FRC 双锚风险记忆在线节点',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frc_health_aggregator = frc_nodes.health_aggregator_node:main',
            'frc_event_marker = frc_nodes.event_marker_node:main',
            'frc_risk_pipeline = frc_nodes.risk_pipeline_node:main',
            'frc_memory_manager = frc_nodes.memory_manager_node:main',
            'frc_trial_runner = frc_nodes.trial_runner_node:main',
        ],
    },
)
