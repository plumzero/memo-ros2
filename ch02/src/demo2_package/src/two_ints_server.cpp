
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/two_ints.hpp>
#include <memory>

void add(const std::shared_ptr<example_interfaces::srv::TwoInts::Request> request,
         std::shared_ptr<example_interfaces::srv::TwoInts::Response> response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld b: %ld", request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  // 创建一个名为"add_two_ints_server"的节点
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");
  // 创建一个名为"add_two_ints"的服务，使用add函数处理请求
  rclcpp::Service<example_interfaces::srv::TwoInts>::SharedPtr service = node->create_service<example_interfaces::srv::TwoInts>("add_two_ints", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}