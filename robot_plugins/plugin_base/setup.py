from setuptools import setup
from glob import glob
import os

package_name = "plugin_base"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Pedro Soares",
    maintainer_email="pmbs.123@gmail.com",
    description="A plugin base class which is used for communication with a BT.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["test_plugin = plugin_base.plugin_base:main"],
    },
)
