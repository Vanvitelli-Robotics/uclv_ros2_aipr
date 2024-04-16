#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

// Include the hpp file of the message that we want to use
#include <std_msgs/msg/string.hpp>

// For more comments see hello_class_node.cpp first!

namespace uclv
{

class HelloPubNode : public rclcpp::Node
{
private:
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;

  // The publisher object is used to publish on a topic
  // Note that it is defined with a template
  // Note that it is a shared ptr, the actual type is:
  // std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>>
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

public:
  HelloPubNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : rclcpp::Node("hello_pub", options), count_(0)
  {
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&HelloPubNode::timer_callback, this));

    // alternative using operator""ms
    // using namespace std::chrono_literals;  // needed to access operator(suffix) ms
    // timer_ = this->create_wall_timer(500ms, std::bind(&HelloPubNode::timer_callback, this));

    // The publisher object is creeted with this method of the rclcpp::Node class.
    // It returns a shared_pointer !
    // The first argument is the topic
    // The second argument is the queue size [https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html#qos-policies]
    publisher_ = this->create_publisher<std_msgs::msg::String>("hello", 10);
  }

  ~HelloPubNode() = default;

  void timer_callback()
  {
    // We publish a message with a given period using a timer

    // We build the message object
    std_msgs::msg::String msg;
    count_++;
    msg.data = "Hello World " + std::to_string(count_);

    RCLCPP_INFO_STREAM(this->get_logger(), msg.data);

    // The publish is done with the publish method on the publisher object
    publisher_->publish(msg);
  }
};

}  // namespace uclv

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto hello_node = std::make_shared<uclv::HelloPubNode>();

  rclcpp::spin(hello_node);

  rclcpp::shutdown();

  return 0;
}