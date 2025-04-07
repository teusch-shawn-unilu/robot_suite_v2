from setuptools import find_packages, setup
from glob import glob
import os

package_name = "hand_gestures"

setup(
    name=package_name,
    version="0.7.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pedros235",
    maintainer_email="pmbs.123@gmail.com",
    description="A ROS package to detect hand landmarks and signs",
    license="GPL-3.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "landmark_detector_node = hand_gestures.landmark_detector_node:main",
            "landmark_annotator_node = hand_gestures.landmark_annotator_node:main",
        ],
    },
)
