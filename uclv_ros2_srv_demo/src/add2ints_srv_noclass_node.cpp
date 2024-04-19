#include "rclcpp/rclcpp.hpp"
#include "uclv_ros2_demo_interfaces/srv/add_two_ints.hpp"

#include <memory>

// This is the service callback, it is executed every time a new service request arrives
// The first argument is a CONST shared ptr to the request
// The second argument is a shared_ptr to the response (to be modified by the callback)
void add(const std::shared_ptr<uclv_ros2_demo_interfaces::srv::AddTwoInts::Request> request,
         std::shared_ptr<uclv_ros2_demo_interfaces::srv::AddTwoInts::Response> response)
{
  // Here the response is modified
  response->sum = request->a + request->b;

  // Print info... (printf style)
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), // <-- to get a simple logger without a node object...
              "Incoming request\na: %ld"
              " b: %ld",
              request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // In this version we do not create any class. The node is created as a base rclcpp::Node
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

  // The rclcpp::Service object manages the service server. It is constructed by the
  // create_service method (of the node class).
  // Note the template!
  // The second argument is the callback. Note that we don't need std::bind here because the callback
  // is not a class member function!
  rclcpp::Service<uclv_ros2_demo_interfaces::srv::AddTwoInts>::SharedPtr service =
      node->create_service<uclv_ros2_demo_interfaces::srv::AddTwoInts>("add_two_ints", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  // Spin to execute the callback(s)
  rclcpp::spin(node);

  // Terminate
  rclcpp::shutdown();
}