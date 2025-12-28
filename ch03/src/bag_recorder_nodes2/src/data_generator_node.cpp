#include <chrono>

#include <example_interfaces/msg/int32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

using namespace std::chrono_literals;

class DataGenerator : public rclcpp::Node
{
public:
  DataGenerator()
  : Node("data_generator")
  {
    data.data = 0;
    const rosbag2_cpp::StorageOptions storage_options({"timed_synthetic_bag", "sqlite3"});
    const rosbag2_cpp::ConverterOptions converter_options(
      {rmw_get_serialization_format(),
       rmw_get_serialization_format()});
    writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();

    writer_->open(storage_options, converter_options);

    writer_->create_topic(
      {"synthetic",
       "example_interfaces/msg/Int32",
       rmw_get_serialization_format(),
       ""});

    // 该节点并非订阅某个主题，而是使用一个定时器。定时器每隔一秒触发一次，并在触发时调用指定的成员函数。
    timer_ = create_wall_timer(1s, std::bind(&DataGenerator::timer_callback, this));
  }

private:
  // 在定时器回调函数中，会生成(或以其他方式获取，例如从连接到某个硬件的串口读取)要存储在包中的数据。
  //  与之前的示例相比，主要区别在于数据尚未序列化。由于包写入器需要序列化数据，因此我们必须先对其进行序列化。
  //  这可以通过使用相应 rclcpp::Serialization 的类来实现。
  void timer_callback()
  {
    auto serializer = rclcpp::Serialization<example_interfaces::msg::Int32>();
    auto serialized_message = rclcpp::SerializedMessage();
    serializer.serialize_message(&data, &serialized_message);

    auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

    bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      new rcutils_uint8_array_t,
      [this](rcutils_uint8_array_t *msg) {
        auto fini_return = rcutils_uint8_array_fini(msg);
        delete msg;
        if (fini_return != RCUTILS_RET_OK) {
          RCLCPP_ERROR(get_logger(),
            "Failed to destroy serialized message %s", rcutils_get_error_string().str);
        }
      });
    *bag_message->serialized_data = serialized_message.release_rcl_serialized_message();

    bag_message->topic_name = "synthetic";
    if (rcutils_system_time_now(&bag_message->time_stamp) != RCUTILS_RET_OK) {
      RCLCPP_ERROR(get_logger(), "Error getting current time: %s",
        rcutils_get_error_string().str);
    }

    writer_->write(bag_message);
    ++data.data;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
  example_interfaces::msg::Int32 data;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataGenerator>());
  rclcpp::shutdown();
  return 0;
}