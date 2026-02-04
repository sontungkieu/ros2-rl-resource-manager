from glob import glob
import os

from setuptools import setup

package_name = "tb3_nav2_dynamic"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="student",
    maintainer_email="student@example.com",
    description="Nav2 bringup helpers and dynamic obstacle spawner/mover for TurtleBot3 Gazebo Sim.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dynamic_obstacles = tb3_nav2_dynamic.dynamic_obstacles_node:main",
            "patrol_robot = tb3_nav2_dynamic.patrol_robot:main",
            "initial_pose_publisher = tb3_nav2_dynamic.initial_pose_publisher_node:main",
        ],
    },
)
