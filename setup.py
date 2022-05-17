#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['coordinates_waypoint_manager_ros'],
 package_dir={'coordinates_waypoint_manager_ros': 'src/coordinates_waypoint_manager_ros'}
)

setup(**d)
