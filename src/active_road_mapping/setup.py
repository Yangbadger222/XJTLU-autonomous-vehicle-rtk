from setuptools import setup
setup(name="active_road_mapping", version="0.1.0", packages=["active_road_mapping"],
      data_files=[("share/ament_index/resource_index/packages", ["resource/active_road_mapping"]),
                  ("share/active_road_mapping", ["package.xml"])], install_requires=["setuptools"],
      entry_points={"console_scripts": [
          "active_road_map = active_road_mapping.map_node:main",
          "active_road_evidence = active_road_mapping.evidence_node:main",
          "active_observation = active_road_mapping.observation_node:main",
      ]})
