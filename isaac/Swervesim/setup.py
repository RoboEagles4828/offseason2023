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
    "protobuf==3.20.1",
    "omegaconf==2.1.1",
    "hydra-core==1.1.1",
    "redis==3.5.3", # needed by Ray on Windows
    "rl-games==1.5.2",
    "shapely"
]

# Installation operation
setup(
    name="swervesim",
    author="NVIDIA",
    version="1.1.0",
    description="RL environments for robot learning in NVIDIA Isaac Sim.",
    keywords=["robotics", "rl"],
    include_package_data=True,
    install_requires=INSTALL_REQUIRES,
    packages=find_packages("."),
    classifiers=["Natural Language :: English", "Programming Language :: Python :: 3.7, 3.8"],
    zip_safe=False,
)

# EOF
