cmake_minimum_required(VERSION 3.8)
project(uclv_ros2_demo_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

########## FIND dependencies ############
# rosidl_default_generators is required for generating ros2 interfaces (msg, srv, action)
find_package(rosidl_default_generators REQUIRED)
# the following are the dependencies of our package
find_package(geometry_msgs REQUIRED)
###################################

##### GENERATE THE ROS2 INTERFACES (msg, srv, action) ####
rosidl_generate_interfaces(${PROJECT_NAME}
  # list all the .msg, .srv, and .action files to be generated...
  "msg/Num.msg"
  "msg/Sphere.msg"
  "srv/AddTwoInts.srv"
  "srv/AddThreeInts.srv"
  # Add packages that above messages depend on
  DEPENDENCIES geometry_msgs # in this case geometry_msgs for Sphere.msg
)
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
