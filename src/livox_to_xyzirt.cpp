// Converts Livox PointCloud2 (x,y,z,intensity,tag,line,timestamp) to
// PointXYZIRT format (x,y,z,intensity,ring,time) expected by image_projection.
//
// Field mapping:
//   line      (uint8)  -> ring  (uint16)
//   timestamp (float64, absolute ns) -> time (float32, seconds relative to frame start)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <cstring>
#include <cstdint>

// Matches LivoxPointXyzrtlt in comm/comm.h (#pragma pack(1))
#pragma pack(1)
struct LivoxPoint {
  float    x;
  float    y;
  float    z;
  float    reflectivity;
  uint8_t  tag;
  uint8_t  line;
  double   timestamp;   // absolute nanoseconds
};
#pragma pack()

// Output layout matching PointXYZIRT registered in image_projection.cpp
#pragma pack(1)
struct PointXYZIRT {
  float    x;
  float    y;
  float    z;
  float    intensity;
  uint16_t ring;
  float    time;        // seconds relative to frame start
};
#pragma pack()

class LivoxToXyzirt : public rclcpp::Node
{
public:
  explicit LivoxToXyzirt(const rclcpp::NodeOptions & options)
  : Node("livox_to_xyzirt", options)
  {
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "livox/lidar", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { convert(msg); });

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("livox/lidar_xyzirt", 10);
  }

private:
  void convert(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in)
  {
    const uint64_t frame_ns =
      static_cast<uint64_t>(in->header.stamp.sec) * 1000000000ULL +
      in->header.stamp.nanosec;

    const size_t n = static_cast<size_t>(in->width) * in->height;
    constexpr size_t out_step = sizeof(PointXYZIRT);

    sensor_msgs::msg::PointCloud2 out;
    out.header      = in->header;
    out.height      = 1;
    out.width       = static_cast<uint32_t>(n);
    out.is_dense    = true;
    out.is_bigendian = false;
    out.point_step  = out_step;
    out.row_step    = static_cast<uint32_t>(n * out_step);

    out.fields.resize(6);
    auto set_field = [&](int i, const char * name, uint32_t offset, uint8_t dtype) {
      out.fields[i].name     = name;
      out.fields[i].offset   = offset;
      out.fields[i].datatype = dtype;
      out.fields[i].count    = 1;
    };
    using PF = sensor_msgs::msg::PointField;
    set_field(0, "x",         0,  PF::FLOAT32);
    set_field(1, "y",         4,  PF::FLOAT32);
    set_field(2, "z",         8,  PF::FLOAT32);
    set_field(3, "intensity", 12, PF::FLOAT32);
    set_field(4, "ring",      16, PF::UINT16);
    set_field(5, "time",      18, PF::FLOAT32);

    out.data.resize(n * out_step);

    const uint8_t * src = in->data.data();
    uint8_t       * dst = out.data.data();

    for (size_t i = 0; i < n; ++i) {
      LivoxPoint p;
      memcpy(&p, src + i * in->point_step, sizeof(LivoxPoint));

      PointXYZIRT q;
      q.x         = p.x;
      q.y         = p.y;
      q.z         = p.z;
      q.intensity = p.reflectivity;
      q.ring      = static_cast<uint16_t>(p.line);
      q.time      = static_cast<float>((p.timestamp - static_cast<double>(frame_ns)) * 1e-9);

      memcpy(dst + i * out_step, &q, out_step);
    }

    pub_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxToXyzirt>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
