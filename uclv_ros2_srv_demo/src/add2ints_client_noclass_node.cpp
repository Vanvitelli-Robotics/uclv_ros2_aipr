#include "rclcpp/rclcpp.hpp"
#include "uclv_ros2_demo_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Here we parse args to not add two hardcoded integers
  if (argc != 3)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
    return 1;
  }

  // after the check we know that "a" is stored in argv[1] and "b" in argv[2]

  // Build the node with a "no class" approach...
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");

  // The client is managed by this rclcpp::Client object
  // Note: this is a shared_ptr, Note: there is a template
  // It is created with the create_client method
  // The argument is the service name
  rclcpp::Client<uclv_ros2_demo_interfaces::srv::AddTwoInts>::SharedPtr client =
      node->create_client<uclv_ros2_demo_interfaces::srv::AddTwoInts>("add_two_ints");

  // In order to send a request we need to build the Request message
  auto request = std::make_shared<uclv_ros2_demo_interfaces::srv::AddTwoInts::Request>();
  request->a = atoll(argv[1]); // <-- atoll converts the argv input (it is a char vector) to an integer
  request->b = atoll(argv[2]);

  // Here we wait for the server to be available (1s is 1 second, it is defined in std::chrono::literals)
  while (!client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      // In these kind of wait cycle, we need to check if the node should be killed!
      // For instance, if the user send a CTRL+C, we should stop the program...
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
  // The wait for service cycle ends here...

  // The request is sent with async_send_request on the client object
  // See [https://docs.ros.org/en/humble/p/rclcpp/generated/classrclcpp_1_1Client.html#_CPPv4N6rclcpp6Client18async_send_requestE13SharedRequest]
  // The return is a FutureAndRequestId (it is a struct that contains the requestID and a std::future)
  // what is a std::future? see: [https://en.cppreference.com/w/cpp/thread/future]
  auto future_result = client->async_send_request(request);
  // The call above is async, it sends the request but does NOT wait for the response
  // The program could do other stuff here...


  // I can wait for the response to be actually received by waiting on the future
  // In ROS2 i can use the utility function spin_until_future_complete
  // To simplify, here is similar to call rclcpp::spin_some(node) until the response is received.
  // This way we do not block the node and we can still receive messages on the callbacks.
  if (rclcpp::spin_until_future_complete(node, future_result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    // If spin_until_future_complete returns SUCCESS, we can get the value in the future with future_result.get()
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", future_result.get()->sum);
  }
  else
  {
    // Else some error...
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}
