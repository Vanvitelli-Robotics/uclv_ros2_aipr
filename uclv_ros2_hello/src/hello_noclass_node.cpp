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
  // this instruction is needed to initialize the ROS2 framework for this executable
  rclcpp::init(argc, argv);

  // The Node object
  auto node = rclcpp::Node::make_shared("hello");

  // A toy variable
  int count = 0;

  // This object is responsable to keep the rate of a loop as close
  // as possible to a prescribed period (1 second in this case)
  rclcpp::WallRate loop_rate(std::chrono::milliseconds(1000));

  // the main loop
  // This is like a while(true)
  // rclcpp::ok() is false when the node is killed or when we press CTRL-C
  while (rclcpp::ok())
  {
    // do stuff
    count++;
    RCLCPP_INFO_STREAM(node->get_logger(), "Hello World " << count);

    // spin is used to receive messages/service requests
    // calls all the callback associated to the node.
    // It does not lock the execution, after the execution of the callback (if any)
    // the program proceeds to the next code line
    rclcpp::spin_some(node);
    // NOTE THAT: In this case we have no subscribers, thus NO callbaks.
    // So, spin_some is like a "no operation". But it is a good practice to put
    // a spin_some instructions in your loops.

    // sleep such that to keep the loop rate to the prescribed value.
    // This is not a simple sleep. It takes into account the compute time needed by the loop.
    loop_rate.sleep();
  }

  // Cleanup the ROS2 framework for this process.
  rclcpp::shutdown();
  return 0;
}