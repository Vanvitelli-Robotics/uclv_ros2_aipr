#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

/* We do not recommend this style anymore, because composition of multiple
 * nodes in the same executable is not possible. Please see one of the subclass
 * examples for the "new" recommended styles. This example is only included
 * for completeness because it is similar to "classic" standalone ROS nodes. */

int main(int argc, char* argv[])
{
  // initialize ROS2
  rclcpp::init(argc, argv);

  // The Node object
  auto node = rclcpp::Node::make_shared("hello");

  // the variable
  int count = 0;

  // the rate
  rclcpp::WallRate loop_rate(std::chrono::milliseconds(1000));

  // the main loop
  while (rclcpp::ok())
  {
    // do stuff
    count++;
    RCLCPP_INFO_STREAM(node->get_logger(), "Hello World " << count);

    // spin is used to receive messages/service requests
    rclcpp::spin_some(node);

    // sleep on rate
    loop_rate.sleep();
  }

  // exit
  rclcpp::shutdown();
  return 0;
}