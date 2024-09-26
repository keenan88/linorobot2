cmake_minimum_required(VERSION 3.8)
project(linorobot2_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

file(GLOB msg_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "msg/*.msg")
file(GLOB srv_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "srv/*.srv")
file(GLOB action_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "action/*.action")

set(rosidl_files ${msg_files} ${action_files} ${srv_files})

rosidl_generate_interfaces(${PROJECT_NAME}
  ${rosidl_files}
)


ament_package()
