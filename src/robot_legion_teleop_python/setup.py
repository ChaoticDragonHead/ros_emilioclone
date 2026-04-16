from setuptools import find_packages, setup

package_name = "robot_legion_teleop_python"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Install config files to the package share directory so ament can find them
        ("share/" + package_name + "/config", ["config/robot_profiles.yaml"]),
        # Install launch files to the package share directory
        ("share/" + package_name + "/launch", [
            "launch/robot_bringup.launch.py",
            "launch/robot_minimal.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vitruvian Systems",
    maintainer_email="",
    description="Robot legion teleop + FPV + stand-in autonomy executors.",
    license="LicenseRef-Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Existing nodes you have in the package tree
            "teleop_legion_key = robot_legion_teleop_python.teleop_legion_key:main",
            "terminal_orchestrator = robot_legion_teleop_python.terminal_orchestrator:main",
            "motor_driver_node = robot_legion_teleop_python.motor_driver_node:main",
            "usb_camera_node = robot_legion_teleop_python.usb_camera_node:main",
            "fpv_control_arbiter = robot_legion_teleop_python.fpv_control_arbiter:main",
            "fpv_camera_mux = robot_legion_teleop_python.fpv_camera_mux:main",

            # Unified stand-in autonomy executor
            "unit_executor_action_server = robot_legion_teleop_python.unit_executor_action_server:main",
            
            # Heartbeat and control lock manager
            "heartbeat_node = robot_legion_teleop_python.heartbeat_node:main",
            "control_lock_manager = robot_legion_teleop_python.control_lock_manager:main",        ],
    },
)
