#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

using std::placeholders::_1;

class SimpleBagRecorder : public rclcpp::Node
{
public:
  SimpleBagRecorder()
  : Node("simple_bag_recorder")
  {
    const rosbag2_cpp::StorageOptions storage_options({"my_bag", "sqlite3"}); // 指定包名与存储格式
    const rosbag2_cpp::ConverterOptions converter_options(
      {rmw_get_serialization_format(),
       rmw_get_serialization_format()});
    writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(); // 创建一个简单的写入器对象

    writer_->open(storage_options, converter_options);

    writer_->create_topic(
      {"chatter", // 主题名称
       "std_msgs/msg/String", // 数据类型
       rmw_get_serialization_format(),
       ""});

    // 创建一个订阅并为其指定一个回调函数，在回调函数中将数据写入包中
    subscription_ = create_subscription<std_msgs::msg::String>(
      "chatter", 10, std::bind(&SimpleBagRecorder::topic_callback, this, _1));
  }

private:
  // 回调函数接收的不是主题数据类型的实例，而是一个对象rclcpp::SerializedMessage。这样做有两个原因。
  // 1.消息数据需要先序列化才能写入包中，因此，与其在接收数据时将其反序列化，然后再重新序列化，不如直接要求 ROS 提供序列化后的消息。
  // 2.写入器 API 需要序列化消息，因此通过向 ROS 请求序列化消息，我们可以省去自己序列化数据的麻烦。
  void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
  {
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
    *bag_message->serialized_data = msg->release_rcl_serialized_message();

    bag_message->topic_name = "chatter";
    // 消息的时间戳也必须设置在 time_stamp 对象成员中。
    //  时间戳可以是任何适合数据的时间，但两个常用值是数据生成时间(如果已知)和接收时间。这里使用的是接收时间。
    if (rcutils_system_time_now(&bag_message->time_stamp) != RCUTILS_RET_OK) {
      RCLCPP_ERROR(get_logger(), "Error getting current time: %s",
        rcutils_get_error_string().str);
    }

    writer_->write(bag_message);
  }

  rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr subscription_;
  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleBagRecorder>());
  rclcpp::shutdown();
  return 0;
}