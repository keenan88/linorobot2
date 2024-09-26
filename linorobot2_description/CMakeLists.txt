cmake_minimum_required(VERSION 3.5)
project(linorobot2_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(
  DIRECTORY launch urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY mecanum_wheel_left/urdf/
  DESTINATION share/${PROJECT_NAME}/mecanum_wheel_left/urdf/
  FILES_MATCHING PATTERN "*.urdf"
)

install(DIRECTORY mecanum_wheel_left/meshes
  DESTINATION share/${PROJECT_NAME}/mecanum_wheel_left/
  FILES_MATCHING PATTERN "*.STL"
)

ament_package()
