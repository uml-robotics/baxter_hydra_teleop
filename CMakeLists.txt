cmake_minimum_required(VERSION 2.8.3)
project(baxter_hydra_teleop)

find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

install(PROGRAMS scripts/teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
