from setuptools import find_packages, setup

package_name = 'frc_bev'

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
    description='FRC BEV 特征构建共用库（train/runtime 唯一真源）',
    license='MIT',
)
