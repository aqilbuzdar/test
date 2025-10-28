cmake_minimum_required(VERSION 3.8)
project(dodo_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# --------------------------
# Install resources
# --------------------------

# Launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# URDF files
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Mesh files
install(
  DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Config files (only dodo.rviz + yaml configs)
install(
  FILES
    config/dodo.rviz
    config/slam.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

# Python scripts (nodes)
install(PROGRAMS scripts/serial_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/mpu6050_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mpu6050_node
)

# --------------------------
ament_package()
