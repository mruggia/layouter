cmake_minimum_required(VERSION 3.0.2)
project(marker)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_install()

cs_install_scripts(scripts/drawer.py)
cs_install_scripts(scripts/wheels.py)
cs_install_scripts(scripts/planner.py)
cs_install_scripts(scripts/camera.py)

cs_export()
