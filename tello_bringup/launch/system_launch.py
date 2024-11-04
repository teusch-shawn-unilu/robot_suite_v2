import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def create_hand_tracker_plugin_launch() -> IncludeLaunchDescription:
    pkg_dir = get_package_share_directory("tello_bringup")
    params_file = os.path.join(pkg_dir, "config", "params.yaml")
    hand_tracker_pck_dir = get_package_share_directory("hand_gestures")
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hand_tracker_pck_dir, "launch/hand_gestures_launch.py")
        ),
        launch_arguments={
            "params_file": params_file,
            "run_annotator": "true",
        }.items(),
    )


def create_tello_sign_interpreter() -> Node:
    return Node(
        package="tello_sign_interpreter",
        executable="tello_sign_interpreter_node",
        output="screen",
    )


def create_face_tracker_plugin_launch() -> IncludeLaunchDescription:
    pkg_dir = get_package_share_directory("tello_bringup")
    params_file = os.path.join(pkg_dir, "config", "params.yaml")
    hand_tracker_pck_dir = get_package_share_directory("face_tracker_plugin")
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hand_tracker_pck_dir, "launch/face_tracker_plugin_launch.py")
        ),
        launch_arguments={"params_file": params_file}.items(),
    )


def create_tello_driver_launch() -> IncludeLaunchDescription:
    pkg_dir = get_package_share_directory("tello_bringup")
    params_file = os.path.join(pkg_dir, "config", "params.yaml")
    tello_driver_pkg_dir = get_package_share_directory("tello_driver")
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tello_driver_pkg_dir, "launch/tello_driver.launch.py")
        ),
    )


def create_tello_bt_launch() -> Node:
    pkg_dir = get_package_share_directory("tello_bringup")
    params_file = os.path.join(pkg_dir, "config", "params.yaml")
    return Node(package="tello_bt", executable="bt_server", output="screen")


def create_tello_control_station_launch() -> Node:
    pkg_dir = get_package_share_directory("tello_bringup")
    params_file = os.path.join(pkg_dir, "config", "params.yaml")
    # tello_driver_pkg_dir = get_package_share_directory("tello_driver")
    return Node(
        package="tello_control_station",
        executable="control_station",
        output="screen",
    )


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(create_hand_tracker_plugin_launch())
    ld.add_action(create_tello_sign_interpreter())
    # ld.add_action(create_face_tracker_plugin_launch())
    ld.add_action(create_tello_driver_launch())
    ld.add_action(create_tello_bt_launch())
    ld.add_action(create_tello_control_station_launch())

    return ld
