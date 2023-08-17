"""Installation script for the 'isaacgymenvs' python package."""

from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

from setuptools import setup, find_packages

import os
import compileall
compileall.compile_dir('swervesim')

# Minimum dependencies required prior to installation
INSTALL_REQUIRES = [
    "protobuf==3.20.2",
    "omegaconf==2.1.1",
    "hydra-core==1.1.1",
    "redis==3.5.3", # needed by Ray on Windows
    "rl-games==1.6.0",
    "shapely",
    "squaternion",
    "robotpy==2022.4.8",
    "wpilib==2022.4.1.6"
]

# Installation operation
setup(
    name="swervesim",
    author="NVIDIA",
    version="1.2.0",
    description="RL environments for robot learning in NVIDIA Isaac Sim.",
    keywords=["robotics", "rl"],
    include_package_data=True,
    install_requires=INSTALL_REQUIRES,
    packages=find_packages("."),
    classifiers=["Natural Language :: English", "Programming Language :: Python :: 3.7, 3.8"],
    zip_safe=False,
)

# EOF
