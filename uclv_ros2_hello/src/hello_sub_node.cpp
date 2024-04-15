#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

namespace uclv
{

class HelloSubNode : public rclcpp::Node
{
private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

public:
  HelloSubNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : rclcpp::Node("hello_sub", options)
  {
    using namespace std::placeholders;
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "hello", 10, std::bind(&HelloSubNode::subscription_callbk, this, _1));
  }

  ~HelloSubNode() = default;

  void subscription_callbk(const std_msgs::msg::String& msg) const
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: " + msg.data);
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