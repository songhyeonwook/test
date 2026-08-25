#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_point.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types_conversion.h>

// Livox PointXYZRTLT와 맞추기 위한 사용자 정의 PCL 포인트 타입
struct EIGEN_ALIGN16 PointXYZRTLT
{
  PCL_ADD_POINT4D;      // quad-word XYZ
  float intensity;
  std::uint8_t tag;
  std::uint8_t line;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZRTLT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (std::uint8_t, tag, tag)
  (std::uint8_t, line, line)
  (double, timestamp, timestamp)
)

class PointCloud2ToCustomMsg : public rclcpp::Node
{
public:
  PointCloud2ToCustomMsg()
  : Node("pointcloud2_to_custommsg")
  {
    // 파라미터 선언
    this->declare_parameter<std::string>("input_topic", "/livox/lidar");
    this->declare_parameter<std::string>("output_topic", "/livox/custom_points");

    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&PointCloud2ToCustomMsg::callbackPointCloud, this, std::placeholders::_1)
    );

    publisher_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>(
      output_topic_,
      rclcpp::QoS(rclcpp::KeepLast(10))
    );

    RCLCPP_INFO(this->get_logger(), "Subscribed input topic : %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing output topic: %s", output_topic_.c_str());
  }

private:
  void callbackPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<PointXYZRTLT> cloud;

    try {
      pcl::fromROSMsg(*msg, cloud);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to convert PointCloud2 to PCL cloud: %s", e.what());
      return;
    }

    if (cloud.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty point cloud received");
      return;
    }

    livox_ros_driver2::msg::CustomMsg custom_msg;
    custom_msg.header = msg->header;
    custom_msg.point_num = static_cast<std::uint32_t>(cloud.size());
    custom_msg.lidar_id = 0;
    custom_msg.rsvd = {0, 0, 0};

    // 첫 번째 포인트 timestamp를 기준 시간으로 사용
    const double first_timestamp = cloud.points.front().timestamp;

    // 주의:
    // timestamp의 단위가 원본 PointCloud2에서 무엇인지(초, ns, us) 반드시 확인해야 함.
    // 여기서는 "초(second)"라고 가정하고 ns로 변환해 timebase/offset_time을 채움.
    const std::uint64_t timebase_ns =
      static_cast<std::uint64_t>(first_timestamp * 1e9);

    custom_msg.timebase = timebase_ns;
    custom_msg.points.resize(cloud.size());

    for (std::size_t i = 0; i < cloud.size(); ++i) {
      const auto & pcl_point = cloud.points[i];
      auto & custom_point = custom_msg.points[i];

      custom_point.x = pcl_point.x;
      custom_point.y = pcl_point.y;
      custom_point.z = pcl_point.z;

      // intensity float -> uint8 범위로 clamp
      float intensity = pcl_point.intensity;
      if (std::isnan(intensity) || intensity < 0.0f) {
        intensity = 0.0f;
      }
      if (intensity > 255.0f) {
        intensity = 255.0f;
      }

      custom_point.reflectivity = static_cast<std::uint8_t>(intensity);
      custom_point.tag = pcl_point.tag;
      custom_point.line = pcl_point.line;

      // 첫 점 대비 상대 시간(ns)
      double dt_sec = pcl_point.timestamp - first_timestamp;
      if (dt_sec < 0.0) {
        dt_sec = 0.0;
      }

      const std::uint64_t offset_ns =
        static_cast<std::uint64_t>(dt_sec * 1e9);

      // CustomPoint.offset_time 은 uint32 이므로 saturate
      custom_point.offset_time =
        (offset_ns > std::numeric_limits<std::uint32_t>::max())
          ? std::numeric_limits<std::uint32_t>::max()
          : static_cast<std::uint32_t>(offset_ns);
    }

    publisher_->publish(custom_msg);
  }

  std::string input_topic_;
  std::string output_topic_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloud2ToCustomMsg>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
