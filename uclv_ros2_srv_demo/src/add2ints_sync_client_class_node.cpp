#include "uclv_ros2_srv_demo/add2ints_client_node.hpp"


int main(int argc, char* argv[])
{
  // this instruction is needed to initialize the ROS2 framework for this executable
  rclcpp::init(argc, argv);

  // Here we parse args to not add two hardcoded integers
  if (argc != 3)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
    return 1;
  }
  int64_t a = atoll(argv[1]);  // <-- atoll converts the argv input (it is a char vector) to an integer
  int64_t b = atoll(argv[2]);

  // Build a AddTwoIntsServiceNode object.
  // The following is a std::shared_ptr<uclv::AddTwoIntsServiceNode>
  auto add_two_ints_client_node = std::make_shared<uclv::AddTwoIntsClientNode>();

  // Triggers the node by calling the async add
  // This is a toy example, this call could be triggered by another ROS2 callback
  add_two_ints_client_node->sync_add(a,b);

  // In this case I don't need to spin. It is managed in he sync_add
  // [removed] rclcpp::spin(add_two_ints_client_node);

  // Cleanup the ROS2 framework for this process.
  rclcpp::shutdown();

  return 0;
}
