from setuptools import find_packages, setup

package_name = "tello_sign_interpreter"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pedro-t14",
    maintainer_email="pmbs.123@gmail.com",
    description="A gestures interpreter for tello based on hand_gestures_plugin",
    license="BSD-3.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tello_sign_interpreter_node = tello_sign_interpreter.tello_sign_interpreter_node:main"
        ],
    },
)
