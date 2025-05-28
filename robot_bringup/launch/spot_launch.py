import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def create_spot_driver_launch(ld: LaunchDescription, pkg_dir: str) -> None:
    spot_driver_pkg_dir = get_package_share_directory("spot_driver")

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(spot_driver_pkg_dir, "launch/spot_driver.launch.py")
            ),
            launch_arguments={"config_file": os.path.join(pkg_dir, "config/spot_driver_params.yaml")}.items()
        )
    )


def create_robot_bt_launch(ld: LaunchDescription) -> None:
    ld.add_action(Node(package="robot_bt", executable="bt_server", output="screen"))


def create_tello_control_station_launch(ld: LaunchDescription) -> None:
    ld.add_action(
        Node(
            package="tello_control_station",
            executable="control_station",
            output="screen",
        )
    )


def create_hand_tracker_plugin_launch(ld: LaunchDescription) -> None:
    pkg_dir = get_package_share_directory("robot_bringup")
    params_file = os.path.join(pkg_dir, "config", "params.yaml")
    hand_tracker_pck_dir = get_package_share_directory("hand_gestures")
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hand_tracker_pck_dir, "launch/hand_gestures_launch.py")
            ),
            launch_arguments={
                "params_file": params_file,
                "run_annotator": "true",
            }.items(),
        )
    )


def generate_launch_description():
    ld = LaunchDescription()
    pkg_dir = get_package_share_directory("robot_bringup")
    

    create_spot_driver_launch(ld, pkg_dir)
    create_robot_bt_launch(ld)
    # te_tello_control_station_launch(ld)

    # ------------------
    # -    Plugins     -
    # ------------------

    create_hand_tracker_plugin_launch(ld)

    return ld
