
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/two_ints.hpp>
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  if (argc != 3) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: two_ints_client X Y");
    return -1;
  }

  // 创建一个名为"add_two_ints_client"的节点
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
  // 创建一个客户端，连接到"add_two_ints"服务
  rclcpp::Client<example_interfaces::srv::TwoInts>::SharedPtr client =
      node->create_client<example_interfaces::srv::TwoInts>("add_two_ints");

  auto request = std::make_shared<example_interfaces::srv::TwoInts::Request>();
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result_future = client->async_send_request(request);

  // 等待结果
  if (rclcpp::spin_until_future_complete(node, result_future) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result_future.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  
  return 0;
}