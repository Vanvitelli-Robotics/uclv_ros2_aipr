#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

// Note that I'm using a namespace (it is not necessary)
namespace uclv
{

// Our node class inherits the rclcpp::Node class
class HelloNode : public rclcpp::Node
{
private:

  // This is a timer object. It is responsable to call a callback function periodically
  // Note that the type is a std::shared_ptr<rclcpp::TimerBase>, it is a pointer!
  rclcpp::TimerBase::SharedPtr timer_;

  // An interal toy variable
  int count_;

public:

  // Our node constructor.
  // Having a constructor that accepts an "const rclcpp::NodeOptions&" with default "rclcpp::NodeOptions()"
  // will be very useful if we want to exploit the ROS2 node composition paradigm [Advanced Concept: https://docs.ros.org/en/foxy/Concepts/About-Composition.html]
  HelloNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : rclcpp::Node("hello", options), count_(0)
  {

    // The timer object can be created via this method of the base rclcpp::Node object.
    // Note that this method returns a shared pointer to a rclcpp::TimerBase implementation.
    // The first input is the timer period as a std::chrono::duration object [https://en.cppreference.com/w/cpp/chrono]
    // The second input is the callback. It should be a function that accepts no arguments.
    // Note that std::bind is used to transform the HelloNode::timer_callback to a function with no arguments (See below).
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&HelloNode::timer_callback, this));

    // Note on std::bind [https://en.cppreference.com/w/cpp/utility/functional/bind]
    // HelloNode::timer_callback is a function that (implicitly) accepts one argument.
    // The actual signature is:
    // void HelloNode::timer_callback(*HelloNode)
    // It accepts a pointer to HelloNode as argument!
    // std::bind used above transforms HelloNode::timer_callback to a new function that accepts no arguments.
    // Let's call this function F
    // void F() =====  std::bind(&HelloNode::timer_callback, this) ==== void HelloNode::timer_callback(this)
    // where this is the pointer to this object.

    // //* alternative using operator""ms *//
    // using namespace std::chrono_literals;  // needed to access operator(suffix) ms
    // timer_ = this->create_wall_timer(500ms, std::bind(&HelloNode::timer_callback, this));
  }

  // A default destructor
  ~HelloNode() = default;

  // The timer callback
  void timer_callback()
  {
    // do stuff
    count_++;
    RCLCPP_INFO_STREAM(this->get_logger(), "Hello World " << count_);
  }
};

}  // namespace uclv

// A main is needed to start the node
// EXTRA: this could be avoided using composition [Advanced Concept: https://docs.ros.org/en/foxy/Concepts/About-Composition.html]
int main(int argc, char* argv[])
{

  // this instruction is needed to initialize the ROS2 framework for this executable
  rclcpp::init(argc, argv);

  // Build a HelloNode object. 
  // The following is a std::shared_ptr<uclv::HelloNode>
  auto hello_node = std::make_shared<uclv::HelloNode>();

  // This is important.
  // Spin lock the execution to this line and calls all the callback associated to the node hello_node
  rclcpp::spin(hello_node);
  // We will exit from the spin function when an error arises, when the node is killed externally,
  // or when we press CTRL-C on the terminal

  // Cleanup the ROS2 framework for this process.
  rclcpp::shutdown();

  return 0;
}