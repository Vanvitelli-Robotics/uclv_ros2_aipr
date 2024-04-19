#include "rclcpp/rclcpp.hpp"
#include "uclv_ros2_demo_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

// Note that I'm using a namespace (it is not necessary)
namespace uclv
{

// Our node class inherits the rclcpp::Node class
class AddTwoIntsClientNode : public rclcpp::Node
{
private:
  // The client is managed by this rclcpp::Client object
  // Note: this is a shared_ptr, Note: there is a template
  // It is created with the create_client method
  rclcpp::Client<uclv_ros2_demo_interfaces::srv::AddTwoInts>::SharedPtr client_;

public:
  // Our node constructor.
  // Having a constructor that accepts an "const rclcpp::NodeOptions&" with default "rclcpp::NodeOptions()"
  // will be very useful if we want to exploit the ROS2 node composition paradigm [Advanced Concept:
  // https://docs.ros.org/en/foxy/Concepts/About-Composition.html]
  AddTwoIntsClientNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : rclcpp::Node("add_two_ints_client", options)
  {
    // The client is managed by this rclcpp::Client object
    // Note: this is a shared_ptr, Note: there is a template
    // It is created with the create_client method
    // The argument is the service name
    client_ = this->create_client<uclv_ros2_demo_interfaces::srv::AddTwoInts>("add_two_ints");
  }

  // A default destructor
  ~AddTwoIntsClientNode() = default;

  void wait_for_server()
  {
    using namespace std::chrono_literals;
    // Here we wait for the server to be available (1s is 1 second, it is defined in std::chrono::literals)
    while (!client_->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        // In these kind of wait cycle, we need to check if the node should be killed!
        // For instance, if the user send a CTRL+C, we should stop the program...
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        throw std::runtime_error("Unable to find the Server");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    // The wait for service cycle ends here...
  }

  void async_add(int64_t a, int64_t b)
  {
    using namespace std::placeholders;  // <-- to use _1

    wait_for_server(); // <--- Here whe choose to wait_for_server in the add function (It is not the only choice)

    // In order to send a request we need to build the Request message
    auto request = std::make_shared<uclv_ros2_demo_interfaces::srv::AddTwoInts::Request>();
    request->a = a;  // <-- atoll converts the argv input (it is a char vector) to an integer
    request->b = b;

    // The request is sent with async_send_request on the client object
    // See
    // [https://docs.ros.org/en/humble/p/rclcpp/generated/classrclcpp_1_1Client.html#_CPPv4N6rclcpp6Client18async_send_requestE13SharedRequest]
    // The return is a FutureAndRequestId (it is a struct that contains the requestID and a std::future)
    // what is a std::future? see: [https://en.cppreference.com/w/cpp/thread/future]
    // Note that this overload accepts a callback as second argument
    // [https://docs.ros.org/en/humble/p/rclcpp/generated/classrclcpp_1_1Client.html#_CPPv4I0_PNSt9enable_ifIN6rclcpp15function_traits14same_argumentsI9CallbackT12CallbackTypeE5valueEE4typeEEN6rclcpp6Client18async_send_requestE24SharedFutureAndRequestId13SharedRequestRR9CallbackT]
    // The callback is called when the response arrives
    auto future_result =
        client_->async_send_request(request, std::bind(&AddTwoIntsClientNode::response_callbk, this, _1));
    // The call above is async, it sends the request but does NOT wait for the response

    // Note that we do note use the future_result in this version

    // Print info... (cout style)
    RCLCPP_INFO_STREAM(this->get_logger(), "Requesting\na: " << request->a << " b: " << request->b);
  }

  // This is the response callback
  void
  response_callbk(rclcpp::Client<uclv_ros2_demo_interfaces::srv::AddTwoInts>::SharedFutureWithRequest future_request)
  {
    // in the future we will find a std::pair, it is a container of two elements
    // In this case the two elements are the request and the response
    auto request_response_pair = future_request.get();

    // The request is in the "first" field
    auto& request = request_response_pair.first;

    // The response is in the "second" field
    auto& response = request_response_pair.second;
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Requested\na: " << request->a << " b: " << request->b << "\nResponse: " << response->sum);
  }
};

}  // namespace uclv

// A main is needed to start the node
// EXTRA: this could be avoided using composition [Advanced Concept:
// https://docs.ros.org/en/foxy/Concepts/About-Composition.html]
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
  add_two_ints_client_node->async_add(a,b);

  // This is important.
  // Spin lock the execution to this line and calls all the callbacks associated to the node hello_node
  rclcpp::spin(add_two_ints_client_node);
  // We will exit from the spin function when an error arises, when the node is killed externally,
  // or when we press CTRL-C on the terminal

  // Cleanup the ROS2 framework for this process.
  rclcpp::shutdown();

  return 0;
}
