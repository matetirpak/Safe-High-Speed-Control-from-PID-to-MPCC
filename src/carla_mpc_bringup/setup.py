"""ament_python package setup for carla_mpc_bringup.

Declare the package metadata, console-script entry points for the control
loop, visualization, and offline tooling, and install the config, launch,
and Foxglove layout assets into the package share directory.
"""

import os
from glob import glob

from setuptools import find_packages, setup

package_name = "carla_mpc_bringup"


def _data_files():
    """Install configs, launch files and Foxglove layouts into the package share."""
    files = [
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ]
    for top in ("config", "launch", "foxglove"):
        for path in glob(f"{top}/**/*", recursive=True):
            if os.path.isfile(path):
                dest = os.path.join("share", package_name, os.path.dirname(path))
                files.append((dest, [path]))
    return files


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=_data_files(),
    install_requires=["setuptools==58.2.0", "numpy", "pyyaml"],
    zip_safe=True,
    maintainer="mate",
    maintainer_email="mate.tirpak@gmail.com",
    description="Carla 0.9.16 + ROS2 Humble high-speed PID/PID++/MPC/MPCC trajectory-tracking testbed.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "control_node = carla_mpc_bringup.sim.control_node:main",
            "gt_pose_publisher = carla_mpc_bringup.sim.gt_pose_publisher:main",
            "ego_model_publisher = carla_mpc_bringup.viz.ego_model_publisher:main",
            "predicted_path_overlay = carla_mpc_bringup.viz.predicted_path_overlay:main",
            "record_run = carla_mpc_bringup.tools.record_run:main",
            "review_run = carla_mpc_bringup.tools.review_run:main",
            "run_benchmark = carla_mpc_bringup.tools.run_benchmark:main",
            "benchmark_report = carla_mpc_bringup.tools.benchmark_report:main",
            "identify_grip = carla_mpc_bringup.tools.identify_grip:main",
            "map_atlas = carla_mpc_bringup.tools.map_atlas:main",
            "route_lab = carla_mpc_bringup.tools.route_lab:main",
            "build_routes = carla_mpc_bringup.tools.build_routes:main",
        ],
    },
)
