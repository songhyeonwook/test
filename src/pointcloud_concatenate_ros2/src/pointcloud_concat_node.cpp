// 여러 대의 라이다 PointCloud2 를 TF 로 한 프레임에 모아 붙여 발행한다.
//
// 상류(lightinfection/pointcloud_concatenate_ros2)에서 고친 점:
//   1) header.stamp 를 rclcpp::Clock().now() (발행 시점의 시스템 시계) 대신
//      입력 클라우드의 스캔 시작 시각으로 쓴다. FAST-LIO 는 header.stamp 를
//      스캔 시작으로 보고 IMU 와 동기화하므로 이게 틀리면 디스큐가 한 프레임
//      어긋나고, 시스템 시계를 쓰면 use_sim_time (bag 재생)이 아예 깨진다.
//   2) sync_all: 설정한 입력이 전부 새로 들어왔을 때만 발행한다. 원본은 고정
//      주기로 마지막 메시지를 다시 써서 같은 프레임을 두 번 내거나 건너뛰었다.
//   3) 파라미터 이름에서 앞의 '/' 를 뗐다(YAML 파라미터 파일에서 쓰기 위해).
//   4) slice_enable: costmap 입력용 z 대역 + 차체 XY 크롭 출력을 하나 더 낸다.
//
// 점별 시간(livox xfer_format=0 의 timestamp 필드)은 절대시각이라 라이다마다
// 재기준화할 필요가 없다. transformPointCloud 가 x/y/z 만 건드리고 나머지
// 필드는 그대로 복사하므로 병합 후에도 그대로 살아 있다.

#include "pointcloud_concatenate_ros2/pointcloud_concat_node.hpp"

#include <cmath>
#include <cstring>
#include <limits>

namespace pointcloud_concatenate
{

PointCloudConcatNode::PointCloudConcatNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_concatenate", options)
{
  handleParams();
  tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfListener.reset(new tf2_ros::TransformListener(*tfBuffer));

  sensor_qos.keep_last(10);
  for (int i = 0; i < kMaxClouds; ++i) {
    const std::string topic = "cloud_in" + std::to_string(i + 1);
    sub_cloud_in[i] = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic, sensor_qos,
      [this, i](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        this->subCallbackCloudIn(msg, i);
      });
  }
  pub_cloud_out = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_out", sensor_qos);
  if (param_slice_enable_) {
    pub_cloud_out_sliced =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_out_sliced", sensor_qos);
  }

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / param_hz_)),
    std::bind(&PointCloudConcatNode::update, this));
}

PointCloudConcatNode::~PointCloudConcatNode()
{
  RCLCPP_INFO(this->get_logger(), "Destructing PointcloudConcatenate...");
}

void PointCloudConcatNode::subCallbackCloudIn(sensor_msgs::msg::PointCloud2::SharedPtr msg, int idx)
{
  cloud_in[idx] = *msg;
  cloud_received[idx] = true;
  cloud_fresh[idx] = true;
}

void PointCloudConcatNode::handleParams()
{
  RCLCPP_INFO(this->get_logger(), "Loading parameters...");

  param_frame_target_ = this->declare_parameter<std::string>("target_frame", "base_link");
  param_clouds_ = this->declare_parameter<int>("clouds", 2);
  param_hz_ = this->declare_parameter<double>("hz", 10.0);
  param_sync_all_ = this->declare_parameter<bool>("sync_all", true);
  param_stamp_from_input_ = this->declare_parameter<bool>("stamp_from_input", true);
  param_slice_enable_ = this->declare_parameter<bool>("slice_enable", false);
  param_slice_z_min_ = this->declare_parameter<double>("slice_z_min", -std::numeric_limits<double>::infinity());
  param_slice_z_max_ = this->declare_parameter<double>("slice_z_max", std::numeric_limits<double>::infinity());
  param_crop_half_x_ = this->declare_parameter<double>("crop_half_x", 0.0);
  param_crop_half_y_ = this->declare_parameter<double>("crop_half_y", 0.0);

  if (param_clouds_ < 1 || param_clouds_ > kMaxClouds) {
    RCLCPP_WARN(this->get_logger(), "clouds must be 1..%d (got %d). Resetting to 2.",
                kMaxClouds, param_clouds_);
    param_clouds_ = 2;
  }
  if (param_hz_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "hz must be > 0 (got %f). Resetting to 10.0", param_hz_);
    param_hz_ = 10.0;
  }

  RCLCPP_INFO(this->get_logger(), "Parameters loaded. target_frame=%s clouds=%d hz=%.1f sync_all=%d",
              param_frame_target_.c_str(), param_clouds_, param_hz_,
              static_cast<int>(param_sync_all_));
}

void PointCloudConcatNode::update()
{
  if (pub_cloud_out->get_subscription_count() == 0 &&
      (!pub_cloud_out_sliced || pub_cloud_out_sliced->get_subscription_count() == 0))
  {
    return;
  }

  for (int i = 0; i < param_clouds_; ++i) {
    if (param_sync_all_ ? !cloud_fresh[i] : !cloud_received[i]) {
      // sync_all 이면 전부 새로 들어올 때까지 기다린다. 라이다 한 대가 죽으면
      // 출력이 멈추는데, 그건 조용히 옛 프레임을 섞어 내는 것보다 낫다.
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Waiting for cloud_in%d ...", i + 1);
      return;
    }
  }

  sensor_msgs::msg::PointCloud2 cloud_out;
  sensor_msgs::msg::PointCloud2 cloud_tf;
  rclcpp::Time stamp_out;
  bool have_stamp = false;

  for (int i = 0; i < param_clouds_; ++i) {
    if (!pcl_ros::transformPointCloud(param_frame_target_, cloud_in[i], cloud_tf, *tfBuffer)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Transforming cloud %d from %s to %s failed!",
                           i + 1, cloud_in[i].header.frame_id.c_str(),
                           param_frame_target_.c_str());
      return;
    }

    // 스캔 시작 시각 = 입력들 중 가장 이른 header.stamp.
    // (livox 드라이버는 프레임의 base_time 을 header.stamp 로 찍는다)
    const rclcpp::Time stamp_i(cloud_in[i].header.stamp);
    if (!have_stamp || stamp_i < stamp_out) {
      stamp_out = stamp_i;
      have_stamp = true;
    }

    if (i == 0) {
      cloud_out = cloud_tf;
    } else if (!pcl::concatenatePointCloud(cloud_out, cloud_tf, cloud_out)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Concatenating cloud %d failed (field layout mismatch?)", i + 1);
      return;
    }
    cloud_fresh[i] = false;
  }

  if (!param_stamp_from_input_ || !have_stamp) {
    stamp_out = this->now();
  }
  publishPointcloud(cloud_out, stamp_out);
}

sensor_msgs::msg::PointCloud2
PointCloudConcatNode::sliceCloud(const sensor_msgs::msg::PointCloud2 & in) const
{
  sensor_msgs::msg::PointCloud2 out;
  out.header = in.header;
  out.height = 1;
  out.fields = in.fields;
  out.is_bigendian = in.is_bigendian;
  out.point_step = in.point_step;
  out.is_dense = in.is_dense;

  int off_x = -1, off_y = -1, off_z = -1;
  for (const auto & f : in.fields) {
    if (f.name == "x") {off_x = static_cast<int>(f.offset);}
    else if (f.name == "y") {off_y = static_cast<int>(f.offset);}
    else if (f.name == "z") {off_z = static_cast<int>(f.offset);}
  }
  if (off_x < 0 || off_y < 0 || off_z < 0 || in.point_step == 0) {
    return out;
  }

  const bool crop = param_crop_half_x_ > 0.0 && param_crop_half_y_ > 0.0;
  const size_t total = in.data.size() / in.point_step;
  out.data.resize(in.data.size());

  size_t kept = 0;
  for (size_t i = 0; i < total; ++i) {
    const uint8_t * p = in.data.data() + i * in.point_step;
    float x, y, z;
    std::memcpy(&x, p + off_x, sizeof(float));
    std::memcpy(&y, p + off_y, sizeof(float));
    std::memcpy(&z, p + off_z, sizeof(float));

    if (z < param_slice_z_min_ || z > param_slice_z_max_) {continue;}
    if (crop && std::fabs(x) <= param_crop_half_x_ && std::fabs(y) <= param_crop_half_y_) {continue;}

    std::memcpy(out.data.data() + kept * in.point_step, p, in.point_step);
    ++kept;
  }

  out.data.resize(kept * in.point_step);
  out.width = static_cast<uint32_t>(kept);
  out.row_step = static_cast<uint32_t>(out.data.size());
  return out;
}

void PointCloudConcatNode::publishPointcloud(
  sensor_msgs::msg::PointCloud2 & cloud, const rclcpp::Time & stamp)
{
  cloud.header.stamp = stamp;
  cloud.header.frame_id = param_frame_target_;
  // concatenatePointCloud 는 width 만 갱신하고 row_step 은 cloud1 것을 남긴다.
  // row_step 을 보고 점을 세는 구독자가 있으므로 여기서 맞춰준다.
  cloud.height = 1;
  cloud.row_step = static_cast<uint32_t>(cloud.data.size());
  pub_cloud_out->publish(cloud);

  if (pub_cloud_out_sliced) {
    auto sliced = sliceCloud(cloud);
    pub_cloud_out_sliced->publish(sliced);
  }
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_concatenate::PointCloudConcatNode)
