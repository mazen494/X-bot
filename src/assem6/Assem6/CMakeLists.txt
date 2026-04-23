cmake_minimum_required(VERSION 3.5)
project(assem6)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/path_planner_node.py
  scripts/path_planning_demo.py
  scripts/ik_solver.py
  scripts/barista_robot.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  urdf
  meshes
  launch
  config
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
