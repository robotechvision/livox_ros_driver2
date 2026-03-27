// Accumulates a fixed number of PointCloud2 messages and publishes a merged cloud.
// Output rate = input_hz / frames.
//
// Parameters:
//   input_topic  (string, default: "cloud_in")
//   output_topic (string, default: "cloud_out")
//   frames       (int,    default: 2)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>

class LidarAccumulator : public rclcpp::Node
{
public:
  explicit LidarAccumulator(const rclcpp::NodeOptions & options)
  : Node("lidar_accumulator", options)
  {
    declare_parameter("input_topic",  "cloud_in");
    declare_parameter("output_topic", "cloud_out");
    declare_parameter("frames",       2);

    const auto input  = get_parameter("input_topic").as_string();
    const auto output = get_parameter("output_topic").as_string();
    frames_ = get_parameter("frames").as_int();

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output, 10);
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { cloud_callback(msg); });
  }

private:
  void cloud_callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    buffer_.push_back(msg);
    if (static_cast<int64_t>(buffer_.size()) < frames_)
      return;

    publish_merged();
    buffer_.clear();
  }

  void publish_merged()
  {
    uint32_t total_points = 0;
    for (const auto & m : buffer_) { total_points += m->width * m->height; }

    const auto & first = *buffer_.front();
    sensor_msgs::msg::PointCloud2 out;
    out.header       = first.header;
    out.height       = 1;
    out.width        = total_points;
    out.fields       = first.fields;
    out.is_bigendian = first.is_bigendian;
    out.point_step   = first.point_step;
    out.row_step     = total_points * first.point_step;
    out.is_dense     = first.is_dense;
    out.data.resize(static_cast<size_t>(total_points) * first.point_step);

    uint32_t time_field_offset = UINT32_MAX;
    for (const auto & field : first.fields) {
      if (field.name == "time") { time_field_offset = field.offset; break; }
    }

    const double t0 = stamp_ns(first.header.stamp) * 1e-9;
    size_t offset = 0;
    for (const auto & msg : buffer_) {
      const size_t n     = static_cast<size_t>(msg->width * msg->height);
      const size_t bytes = n * msg->point_step;
      std::memcpy(out.data.data() + offset, msg->data.data(), bytes);
      if (time_field_offset != UINT32_MAX) {
        const float dt = static_cast<float>(stamp_ns(msg->header.stamp) * 1e-9 - t0);
        for (size_t i = 0; i < n; ++i) {
          float * tp = reinterpret_cast<float *>(
            out.data.data() + offset + i * msg->point_step + time_field_offset);
          *tp += dt;
        }
      }
      offset += bytes;
    }

    pub_->publish(out);
  }

  static int64_t stamp_ns(const builtin_interfaces::msg::Time & t)
  {
    return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec;
  }

  int64_t frames_{2};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;
  std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr>    buffer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarAccumulator>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
