from setuptools import find_packages, setup
import os

package_name = "uav_constants"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="bwally",
    maintainer_email="bwaldropw@aol.com",
    description="Shared constants for UAV packages",
    license="TODO",
    extras_require={
        "test": [
            "pytest",
        ],
    },
)
