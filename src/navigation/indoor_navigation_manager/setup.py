from setuptools import find_packages, setup


package_name = "indoor_navigation_manager"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="FYP Team",
    maintainer_email="Kevinlasnh@outlook.com",
    description="Named-destination action manager for indoor Nav2 navigation",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "indoor_navigation_manager_node = indoor_navigation_manager.manager_node:main",
            "foxglove_navigation_adapter_node = indoor_navigation_manager.foxglove_adapter_node:main",
        ],
    },
)
