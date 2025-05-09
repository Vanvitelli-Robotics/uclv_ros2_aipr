cmake_minimum_required(VERSION 3.8)
project(uclv_custom_action_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

########## FIND dependencies ############
find_package(ament_cmake REQUIRED)  
find_package(uclv_custom_action_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED) # <-- the ros2 action lib  
###################################

########## ADD executable fibonacci_action_server ############
add_executable(fibonacci_action_server src/fibonacci_action_server.cpp)
ament_target_dependencies(
  fibonacci_action_server 
  "uclv_custom_action_interfaces"
  "rclcpp"
  "rclcpp_action"
) # <-- add dependencies to the executable
###################################

########## ADD executable fibonacci_action_client ############
add_executable(fibonacci_action_client src/fibonacci_action_client.cpp)
ament_target_dependencies(
  fibonacci_action_client 
  "uclv_custom_action_interfaces"
  "rclcpp"
  "rclcpp_action"
) # <-- add dependencies to the executable
###################################

########## ADD executable fibonacci_action_client_sync ############
add_executable(fibonacci_action_client_sync src/fibonacci_action_client_sync.cpp)
ament_target_dependencies(
  fibonacci_action_client_sync 
  "uclv_custom_action_interfaces"
  "rclcpp"
  "rclcpp_action"
) # <-- add dependencies to the executable
###################################

########## ADD executable fibonacci_action_client_synccall_asyncspin ############
add_executable(fibonacci_action_client_synccall_asyncspin src/fibonacci_action_client_synccall_asyncspin.cpp)
ament_target_dependencies(
  fibonacci_action_client_synccall_asyncspin 
  "uclv_custom_action_interfaces"
  "rclcpp"
  "rclcpp_action"
) # <-- add dependencies to the executable
###################################

########## INSTALL the executable ############
install(TARGETS
  fibonacci_action_server
  fibonacci_action_client
  fibonacci_action_client_sync
  fibonacci_action_client_synccall_asyncspin
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
