"""Build recipe for the boat ament_python package."""

import os
from glob import glob

from setuptools import find_packages, setup

package_name = "boat"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="sorgiovanni60@gmail.com",
    description="Boat control and autonomy stack",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "heartbeat = boat.heartbeat:main",
        ],
    },
)
