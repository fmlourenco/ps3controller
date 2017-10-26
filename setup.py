#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['ps3controller', 'ps3controller_ros'],
 package_dir={'ps3controller': 'common/src/ps3controller', 'ps3controller_ros': 'ros/src/ps3controller_ros'}
)

setup(**d)
