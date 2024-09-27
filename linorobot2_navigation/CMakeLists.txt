cmake_minimum_required(VERSION 3.5)
project(linorobot2_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

install(
  DIRECTORY launch rviz config maps
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    linorobot2_navigation/navigate_to_named_pose.py  # Path to your Python script
  DESTINATION lib/${PROJECT_NAME}  # Install location
  RENAME navigate_to_named_pose  # Remove .py from the installed executable name
)

ament_package()
