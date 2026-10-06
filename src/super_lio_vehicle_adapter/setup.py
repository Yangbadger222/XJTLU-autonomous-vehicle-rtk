from setuptools import setup
setup(name="super_lio_vehicle_adapter", version="0.1.0",
      packages=["super_lio_vehicle_adapter"],
      data_files=[("share/ament_index/resource_index/packages", ["resource/super_lio_vehicle_adapter"]),
                  ("share/super_lio_vehicle_adapter", ["package.xml"])],
      install_requires=["setuptools"],
      entry_points={"console_scripts": ["super_lio_vehicle_adapter = super_lio_vehicle_adapter.adapter_node:main"]})
