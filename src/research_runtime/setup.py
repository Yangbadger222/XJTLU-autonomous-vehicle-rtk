from setuptools import find_packages, setup
from glob import glob

setup(
    name="research_runtime",
    version="0.1.0",
    packages=find_packages(),
    data_files=[("share/ament_index/resource_index/packages", ["resource/research_runtime"]), ("share/research_runtime", ["package.xml"]),
                ("share/research_runtime/web", glob("web/*"))],
    install_requires=["setuptools"],
    entry_points={"console_scripts": [
        "research_replay = research_runtime.replay_sim:main",
        "research_safety_bridge = research_runtime.safety_bridge:main",
        "research_local_obstacle_grid = research_runtime.local_obstacle_grid_node:main",
        "research_operator_console = research_runtime.console_node:main",
    ]},
)
