cmake_minimum_required(VERSION 3.8)
project(uclv_ros2_hello)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

########## FIND rclcpp ############
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED) # <-- needed by hello_pub
###################################

########## ADD executable hello_class ############
add_executable(hello_class src/hello_class_node.cpp)
ament_target_dependencies(hello_class rclcpp) # <-- add dependencies to the executable
###################################

########## ADD executable hello_noclass ############
add_executable(hello_noclass src/hello_noclass_node.cpp)
ament_target_dependencies(hello_noclass rclcpp) # <-- add dependencies to the executable
###################################

########## ADD executable hello_pub ############
add_executable(hello_pub src/hello_pub_node.cpp)
# !NOTE! that we need the new dependency std_msgs!
ament_target_dependencies(hello_pub rclcpp std_msgs) # <-- add dependencies to the executable
###################################

########## INSTALL the executable ############
install(TARGETS
  hello_class
  hello_noclass
  hello_pub
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
