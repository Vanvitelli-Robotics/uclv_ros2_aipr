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

class HelloSubNode : public rclcpp::Node
{
private:

  // The subscription object is used to subscribe to a topic
  // Note that it is defined with a template
  // Note that it is a shared ptr, the actual type is:
  // std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>>
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

  // This is a const pointer to a String message.
  // This is useful to use the message outside the callback
  // The actual type is:
  // std::shared_ptr<const std_msgs::msg::String>
  std_msgs::msg::String::ConstSharedPtr last_msg_;

public:
  HelloSubNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : rclcpp::Node("hello_sub", options)
  {
    // In the constructor we create the subscripton object

    // using namespace is to avoid to write std::placeholders::_1 in the bind function below
    using namespace std::placeholders;

    // create_subscription is a method of the rclcpp::Node object.
    // It creates a rclcpp::Subscription and returns a shared pointer to it
    // The first parameter is the topic name
    // The second parameter is the queue size [https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html#qos-policies]
    // The last parameter is the callback to be called every time a new message arrives
    // The callback should be a function that returns void and accepts only one parameter (the message)
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "hello_sub", 10, std::bind(&HelloSubNode::subscription_callbk, this, _1));

    // std::bind (above) is used to transform the subscription_callbk to a function that accepts a message
    // Note that HelloSubNode::subscription_callbk (implicitly) accepts 2 parameters!
    // void HelloSubNode::subscription_callbk(*HelloSubNode,const std_msgs::msg::String::ConstSharedPtr&)

  }

  ~HelloSubNode() = default;

  // The callback. It runs every time a new message arrives
  // There are various ways to write a callback, the most efficient one is to use a ConstSharedPtr
  void subscription_callbk(const std_msgs::msg::String::ConstSharedPtr& msg)
  {
    // In this example we copy the message pointer
    // This way we can use the message later in other methods
    last_msg_ = msg;

    // Then we choose to call the method that processes the message directly in the callback
    // Note: This is not the only way. We could have a separate thread that processes the messages 
    process_msg();
  }

  // This is a toy example
  void process_msg()
  {
    if (last_msg_)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "I heard: " + last_msg_->data);
    }
    else
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "No message received!");
    }
  }
};

}  // namespace uclv

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto hello_node = std::make_shared<uclv::HelloSubNode>();

  rclcpp::spin(hello_node);

  rclcpp::shutdown();

  return 0;
}