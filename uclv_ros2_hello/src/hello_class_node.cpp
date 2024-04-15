#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace uclv
{

class HelloNode : public rclcpp::Node
{
private:
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;

public:
  HelloNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : rclcpp::Node("hello", options), count_(0)
  {
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&HelloNode::timer_callback, this));

    // //* alternative using operator""ms *//
    // using namespace std::chrono_literals;  // needed to access operator(suffix) ms
    // timer_ = this->create_wall_timer(500ms, std::bind(&HelloNode::timer_callback, this));
  }

  ~HelloNode() = default;

  void timer_callback()
  {
    count_++;
    RCLCPP_INFO_STREAM(this->get_logger(), "Hello World " << count_);
  }
};

}  // namespace uclv

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto hello_node = std::make_shared<uclv::HelloNode>();

  rclcpp::spin(hello_node);

  rclcpp::shutdown();

  return 0;
}