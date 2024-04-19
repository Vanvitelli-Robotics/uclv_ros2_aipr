cmake_minimum_required(VERSION 3.8)
project(uclv_ros2_srv_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

########## FIND dependencies ############
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(uclv_ros2_demo_interfaces REQUIRED)
###################################

# this is needed to add the include directory of this package to the include search path
include_directories(include)

########## ADD executable add2ints_srv_noclass ############
add_executable(add2ints_srv_noclass src/add2ints_srv_noclass_node.cpp)
ament_target_dependencies(add2ints_srv_noclass rclcpp uclv_ros2_demo_interfaces) # <-- add dependencies to the executable
###################################

########## ADD executable add2ints_srv ############
add_executable(add2ints_srv src/add2ints_srv_class_node.cpp)
ament_target_dependencies(add2ints_srv rclcpp uclv_ros2_demo_interfaces) # <-- add dependencies to the executable
###################################

########## ADD executable add2ints_client ############
add_executable(add2ints_client_noclass src/add2ints_client_noclass_node.cpp)
ament_target_dependencies(add2ints_client_noclass rclcpp uclv_ros2_demo_interfaces) # <-- add dependencies to the executable
###################################

########## ADD executable add2ints_client ############
add_executable(add2ints_client src/add2ints_client_class_node.cpp)
ament_target_dependencies(add2ints_client rclcpp uclv_ros2_demo_interfaces) # <-- add dependencies to the executable
###################################

########## ADD executable add2ints_sync_client ############
add_executable(add2ints_sync_client src/add2ints_sync_client_class_node.cpp)
ament_target_dependencies(add2ints_sync_client rclcpp uclv_ros2_demo_interfaces) # <-- add dependencies to the executable
###################################

########## INSTALL the executable ############
install(TARGETS
  add2ints_srv_noclass
  add2ints_srv
  add2ints_client_noclass
  add2ints_client
  add2ints_sync_client
  DESTINATION lib/${PROJECT_NAME})
###################################

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

ament_package()
