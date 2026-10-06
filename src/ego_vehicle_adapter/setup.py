from setuptools import setup
setup(name="ego_vehicle_adapter", version="0.1.0", packages=["ego_vehicle_adapter"],
      data_files=[("share/ament_index/resource_index/packages", ["resource/ego_vehicle_adapter"]),
                  ("share/ego_vehicle_adapter", ["package.xml"])],
      install_requires=["setuptools"])
