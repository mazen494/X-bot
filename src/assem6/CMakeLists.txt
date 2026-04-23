cmake_minimum_required(VERSION 3.5)
project(assem6)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  meshes
  launch
  config
  scripts
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/joint_controller.py
  scripts/barista_gui.py
  scripts/position_finder.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
