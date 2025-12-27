#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <example_interfaces/msg/int32.hpp>

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

int main(int, char**)
{
  example_interfaces::msg::Int32 data;
  data.data = 0;
  const rosbag2_cpp::StorageOptions storage_options({"big_synthetic_bag", "sqlite3"});
  const rosbag2_cpp::ConverterOptions converter_options(
    {rmw_get_serialization_format(),
     rmw_get_serialization_format()});
  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_ =
    std::make_unique<rosbag2_cpp::writers::SequentialWriter>();

  writer_->open(storage_options, converter_options);

  writer_->create_topic(
    {"synthetic",
     "example_interfaces/msg/Int32",
     rmw_get_serialization_format(),
     ""});

  // 这里会为每个样本生成时间戳，而不是依赖当前的系统时间
  // 数据将以这些时间戳指定的速率播放，因此这是一种控制样本默认播放速度的有效方法
  //  虽然每个样本之间的时间间隔为一秒，但此可执行文件实际上无需在每个样本之间等待一秒。这使我们
  //  能够在远少于播放所需的时间内生成涵盖较大时间跨度的大量数据
  rcutils_time_point_value_t time_stamp;
  if (rcutils_system_time_now(&time_stamp) != RCUTILS_RET_OK) {
    std::cerr << "Error getting current time: " <<
      rcutils_get_error_string().str;
    return 1;
  }
  // 使用 for 循环来驱动数据生成
  for (int32_t ii = 0; ii < 100; ++ii) {
    auto serializer = rclcpp::Serialization<example_interfaces::msg::Int32>();
    auto serialized_message = rclcpp::SerializedMessage();
    serializer.serialize_message(&data, &serialized_message);

    auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

    bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      new rcutils_uint8_array_t,
      [](rcutils_uint8_array_t *msg) {
        auto fini_return = rcutils_uint8_array_fini(msg);
        delete msg;
        if (fini_return != RCUTILS_RET_OK) {
          std::cerr << "Failed to destroy serialized message " <<
            rcutils_get_error_string().str;
        }
      });
    *bag_message->serialized_data = serialized_message.release_rcl_serialized_message();

    bag_message->topic_name = "synthetic";
    bag_message->time_stamp = time_stamp;

    writer_->write(bag_message);
    ++data.data;
    time_stamp += 1000000000;
  }

  return 0;
}