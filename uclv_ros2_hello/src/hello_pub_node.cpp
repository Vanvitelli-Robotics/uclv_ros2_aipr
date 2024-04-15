#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

namespace uclv
{

class HelloPubNode : public rclcpp::Node
{
private:
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

public:
  HelloPubNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : rclcpp::Node("hello_pub", options), count_(0)
  {
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&HelloPubNode::timer_callback, this));

    // alternative using operator""ms
    // using namespace std::chrono_literals;  // needed to access operator(suffix) ms
    // timer_ = this->create_wall_timer(500ms, std::bind(&HelloPubNode::timer_callback, this));

    publisher_ = this->create_publisher<std_msgs::msg::String>("hello",10);
  }

  ~HelloPubNode() = default;

  void timer_callback()
  {
    std_msgs::msg::String msg;
    count_++;
    msg.data = "Hello World " + std::to_string(count_);
    
    RCLCPP_INFO_STREAM(this->get_logger(), msg.data);

    publisher_->publish(msg);
  }
};

}  // namespace uclv

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto hello_node = std::make_shared<uclv::HelloPubNode>();

  rclcpp::spin(hello_node);

  rclcpp::shutdown();

  return 0;
}