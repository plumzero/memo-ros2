
#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/add_three_ints.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 4) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Usage: add_three_ints_client X Y Z");
    return -1;
  }

  auto node = rclcpp::Node::make_shared("add_three_ints_client");
  auto client = node->create_client<tutorial_interfaces::srv::AddThreeInts>("add_three_ints");

  auto request = std::make_shared<tutorial_interfaces::srv::AddThreeInts::Request>();
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);
  request->c = atoll(argv[3]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return -1;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
  }

  auto result_future = client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result: %ld", result_future.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_three_ints");
    return -1;
  }

  rclcpp::shutdown();
  
  return 0;
}