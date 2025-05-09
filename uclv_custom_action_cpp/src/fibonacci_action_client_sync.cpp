#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "uclv_custom_action_interfaces/action/fibonacci.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// This "using" is to avoid writing long types
using Fibonacci = uclv_custom_action_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

int main(int argc, char const* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("fibonacci_action_client");

  // creation of the action client
  rclcpp_action::Client<Fibonacci>::SharedPtr client = rclcpp_action::create_client<Fibonacci>(node, "fibonacci");

  // wait for the action server to be available
  if (!client->wait_for_action_server())
  {
    RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
    return -1;
  }

  auto goal_msg = Fibonacci::Goal();
  goal_msg.order = 10;

  RCLCPP_INFO(node->get_logger(), "Sending goal");

  // the async_send_goal returns a std::shared_future<std::shared_ptr<GoalHandleFibonacci>>
  auto future_goal_handle = client->async_send_goal(goal_msg);

  // wait on the future while spinning on the node
  if (rclcpp::spin_until_future_complete(node, future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Wait goal handle error!");
    rclcpp::shutdown();
    return -1;
  }

  // get the goal handle, it is a std::shared_ptr<GoalHandleFibonacci> 
  // = std::shared_ptr<rclcpp_action::ClientGoalHandle<uclv_custom_action_interfaces::action::Fibonacci>>
  auto goal_handle = future_goal_handle.get();
  // check if the goal was accepted, it the pointer is a nullprt the goal was rejected
  if (!goal_handle)
  {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
    rclcpp::shutdown();
    return -1;
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "Goal accepted by server, waiting for result");
  }


  // async_get_result returns a std::shared_future<GoalHandleFibonacci::WrappedResult>
  // = std::shared_future<rclcpp_action::ClientGoalHandle<uclv_custom_action_interfaces::action::Fibonacci>::WrappedResult>
  // where the WrappedResult is an object containing the result code and the actual result
  auto future_result = client->async_get_result(goal_handle);

  // wait on the future while spinning on the node
  if (rclcpp::spin_until_future_complete(node, future_result) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Waiting goal result...");
    rclcpp::shutdown();
    return -1;
  }

  // finally get the result
  auto result = future_result.get();

  // check the rclcpp_action::ResultCode to see the terminal state of the action
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
      return -1;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node->get_logger(), "Goal was canceled");
      return -1;
    default:
      RCLCPP_ERROR(node->get_logger(), "Unknown result code");
      return -1;
  }

  // use the actual result: result.result is a std::shared_ptr<uclv_custom_action_interfaces::action::Fibonacci_Result>
  // in this toy example we print the result
  std::stringstream ss;
  ss << "Result received: ";
  for (auto number : result.result->sequence)
  {
    ss << number << " ";
  }
  RCLCPP_INFO(node->get_logger(), ss.str().c_str());

  rclcpp::shutdown();

  return 0;
}
