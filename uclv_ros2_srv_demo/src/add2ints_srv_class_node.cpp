#include "rclcpp/rclcpp.hpp"
#include "uclv_ros2_demo_interfaces/srv/add_two_ints.hpp"

#include <memory>

// Note that I'm using a namespace (it is not necessary)
namespace uclv
{

// Our node class inherits the rclcpp::Node class
class AddTwoIntsServiceNode : public rclcpp::Node
{
private:
  // The rclcpp::Service object manages the service server. It is constructed by the
  // create_service method (of the node class).
  // Note the template!
  rclcpp::Service<uclv_ros2_demo_interfaces::srv::AddTwoInts>::SharedPtr service_;

public:
  // Our node constructor.
  // Having a constructor that accepts an "const rclcpp::NodeOptions&" with default "rclcpp::NodeOptions()"
  // will be very useful if we want to exploit the ROS2 node composition paradigm [Advanced Concept:
  // https://docs.ros.org/en/foxy/Concepts/About-Composition.html]
  AddTwoIntsServiceNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : rclcpp::Node("add_two_ints_server", options)
  {
    using namespace std::placeholders;  // <-- to use _1 and _2

    // The rclcpp::Service is constructed by the create_service method (of the node class).
    // Note the template!
    // The second argument is the callback. Note that we DO need std::bind here because the callback
    // is a class member function!
    service_ = this->create_service<uclv_ros2_demo_interfaces::srv::AddTwoInts>(
        "add_two_ints", std::bind(&AddTwoIntsServiceNode::add, this, _1, _2));

    // std::bind [https://en.cppreference.com/w/cpp/utility/functional/bind]

    RCLCPP_INFO_STREAM(this->get_logger(), "Ready to add two ints.");
  }

  // A default destructor
  ~AddTwoIntsServiceNode() = default;

  // This is the service callback, it is executed every time a new service request arrives
  // The first argument is a CONST shared ptr to the request
  // The second argument is a shared_ptr to the response (to be modified by the callback)
  void add(const std::shared_ptr<uclv_ros2_demo_interfaces::srv::AddTwoInts::Request> request,
           std::shared_ptr<uclv_ros2_demo_interfaces::srv::AddTwoInts::Response> response)
  {
    // Here the response is modified
    response->sum = request->a + request->b;

    // Print info... (cout style)
    RCLCPP_INFO_STREAM(this->get_logger(), "Incoming request\na: " << request->a << " b: " << request->b);
    RCLCPP_INFO_STREAM(this->get_logger(), "sending back response: [" << (long int)response->sum << "]");
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

  // Build a AddTwoIntsServiceNode object.
  // The following is a std::shared_ptr<uclv::AddTwoIntsServiceNode>
  auto add_two_ints_node = std::make_shared<uclv::AddTwoIntsServiceNode>();

  // This is important.
  // Spin lock the execution to this line and calls all the callback associated to the node hello_node
  rclcpp::spin(add_two_ints_node);
  // We will exit from the spin function when an error arises, when the node is killed externally,
  // or when we press CTRL-C on the terminal

  // Cleanup the ROS2 framework for this process.
  rclcpp::shutdown();

  return 0;
}
