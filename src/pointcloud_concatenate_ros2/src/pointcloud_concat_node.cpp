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
//   5) 입력 구독을 SensorDataQoS(BEST_EFFORT) 대신 RELIABLE 로 받는다. 프레임이
//      0.4~0.7MB 라 BEST_EFFORT 로는 입력당 ~10% 가 유실됐고, sync_all 때문에
//      그게 곱해져 병합 출력이 절반으로 떨어졌다.
//
// 점별 시간(livox xfer_format=0 의 timestamp 필드)은 transformPointCloud 뒤에도
// 살아 있다. 단, 여러 MID-360 의 장치 시계가 동기화되지 않은 환경에서는
// align_timestamps 옵션으로 기준 라이다 시계에 맞춰 재기준화한다.

#include "pointcloud_concatenate_ros2/pointcloud_concat_node.hpp"

#include <cmath>
#include <cstring>
#include <limits>

namespace pointcloud_concatenate
{

// 이 시간 동안 한 번도 안 들어온 입력만 "끊겼다"고 경고한다. 라이다는 10Hz 이므로
// 프레임 몇 개 빠진 것으로는 찍히지 않는다.
constexpr double kInputTimeoutSec = 1.0;

PointCloudConcatNode::PointCloudConcatNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_concatenate", options)
{
  handleParams();
  tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfListener.reset(new tf2_ros::TransformListener(*tfBuffer));

  sensor_qos.keep_last(10);

  // 입력은 RELIABLE 로 받는다. MID-360 한 프레임이 0.4~0.7MB 라서 BEST_EFFORT
  // (SensorDataQoS) 로는 RTPS 프래그먼트 하나만 유실돼도 샘플 전체가 버려지고
  // 재전송이 없다. 실측으로 입력당 ~10% 가 사라졌고, sync_all 이 3대를 모두
  // 요구하므로 그 손실이 곱해져 병합 출력이 입력 ~9Hz 대비 ~5Hz 까지 떨어졌다.
  // livox_ros_driver2 의 발행자가 RELIABLE 이므로 그대로 맞물린다.
  const rclcpp::QoS input_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  for (int i = 0; i < kMaxClouds; ++i) {
    const std::string topic = "cloud_in" + std::to_string(i + 1);
    sub_cloud_in[i] = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic, input_qos,
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
  cloud_received_at[idx] = this->get_clock()->now();
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
  param_align_timestamps_ = this->declare_parameter<bool>("align_timestamps", false);
  param_timestamp_reference_cloud_ =
    this->declare_parameter<int>("timestamp_reference_cloud", 1);
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
  if (param_timestamp_reference_cloud_ < 1 ||
      param_timestamp_reference_cloud_ > param_clouds_)
  {
    RCLCPP_WARN(
      this->get_logger(),
      "timestamp_reference_cloud must be 1..%d (got %d). Resetting to 1.",
      param_clouds_, param_timestamp_reference_cloud_);
    param_timestamp_reference_cloud_ = 1;
  }

  RCLCPP_INFO(this->get_logger(),
              "Parameters loaded. target_frame=%s clouds=%d hz=%.1f sync_all=%d "
              "align_timestamps=%d reference_cloud=%d",
              param_frame_target_.c_str(), param_clouds_, param_hz_,
              static_cast<int>(param_sync_all_),
              static_cast<int>(param_align_timestamps_),
              param_timestamp_reference_cloud_);
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
      warnIfInputStale();
      return;
    }
  }

  sensor_msgs::msg::PointCloud2 cloud_out;
  sensor_msgs::msg::PointCloud2 cloud_tf;
  rclcpp::Time stamp_out;
  bool have_stamp = false;
  const int reference_idx = param_timestamp_reference_cloud_ - 1;
  const rclcpp::Time reference_stamp(cloud_in[reference_idx].header.stamp);
  const rclcpp::Time reference_received = cloud_received_at[reference_idx];

  for (int i = 0; i < param_clouds_; ++i) {
    if (!pcl_ros::transformPointCloud(param_frame_target_, cloud_in[i], cloud_tf, *tfBuffer)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Transforming cloud %d from %s to %s failed!",
                           i + 1, cloud_in[i].header.frame_id.c_str(),
                           param_frame_target_.c_str());
      return;
    }

    const rclcpp::Time stamp_i(cloud_in[i].header.stamp);
    rclcpp::Time aligned_stamp = stamp_i;
    if (param_align_timestamps_) {
      // 기준 라이다(현재 구성에서는 상단)의 장치 시계에 모든 입력을 맞춘다.
      // 단순히 세 스캔을 동시 시작으로 가정하지 않고, 콜백 수신시각 차이를
      // 더해 라이다별 스캔 위상 차이도 보존한다.
      const int64_t receive_delta_ns =
        (cloud_received_at[i] - reference_received).nanoseconds();
      const int64_t aligned_base_ns =
        reference_stamp.nanoseconds() + receive_delta_ns;
      aligned_stamp = rclcpp::Time(aligned_base_ns, reference_stamp.get_clock_type());
      if (!alignPointTimestamps(
          cloud_tf, stamp_i.nanoseconds(), aligned_base_ns))
      {
        return;
      }
    }

    // 병합 스캔의 시작은 정렬된 입력들 중 가장 이른 시각이다.
    if (!have_stamp || aligned_stamp < stamp_out) {
      stamp_out = aligned_stamp;
      have_stamp = true;
    }

    if (i == 0) {
      cloud_out = cloud_tf;
    } else if (!pcl::concatenatePointCloud(cloud_out, cloud_tf, cloud_out)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Concatenating cloud %d failed (field layout mismatch?)", i + 1);
      return;
    }
  }

  if (!param_stamp_from_input_ || !have_stamp) {
    stamp_out = this->now();
  }
  publishPointcloud(cloud_out, stamp_out);

  // 소비 표시는 발행에 성공한 뒤에만. 위 루프 안에서 지우면 transform/align 이
  // 중간에 실패해 빠져나갈 때 앞쪽 입력만 지워져서, 쓰지도 않은 프레임을 버리고
  // 그 입력의 다음 프레임을 처음부터 다시 기다리게 된다(한 프레임 통째로 손실).
  for (int i = 0; i < param_clouds_; ++i) {
    cloud_fresh[i] = false;
  }
}

void PointCloudConcatNode::warnIfInputStale()
{
  // sync_all 폴링(hz)이 라이다 발행률보다 빠르므로 "아직 다 안 들어왔다" 자체는
  // 정상이다. 그걸로 경고하면 hz=50 / 라이다 10Hz 에서 5초 throttle 이 항상 꽉
  // 차고, 위 루프가 늘 i=0 에서 먼저 멈추므로 멀쩡한데도 cloud_in1 만 지목한다.
  // 그래서 정말 끊긴 입력만 알린다.
  const rclcpp::Time now = this->get_clock()->now();
  std::string stale;
  for (int i = 0; i < param_clouds_; ++i) {
    if (!cloud_received[i] ||
        (now - cloud_received_at[i]).seconds() > kInputTimeoutSec)
    {
      if (!stale.empty()) {stale += ", ";}
      stale += "cloud_in" + std::to_string(i + 1);
    }
  }
  if (stale.empty()) {return;}
  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                       "No new data for >%.1fs on: %s", kInputTimeoutSec, stale.c_str());
}

bool PointCloudConcatNode::alignPointTimestamps(
  sensor_msgs::msg::PointCloud2 & cloud,
  int64_t input_base_ns,
  int64_t aligned_base_ns)
{
  if (cloud.is_bigendian || cloud.point_step == 0) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Cannot align timestamps: unsupported big-endian or zero point_step cloud.");
    return false;
  }

  int timestamp_offset = -1;
  for (const auto & field : cloud.fields) {
    if (field.name == "timestamp" &&
        field.datatype == sensor_msgs::msg::PointField::FLOAT64 &&
        field.count >= 1)
    {
      timestamp_offset = static_cast<int>(field.offset);
      break;
    }
  }
  if (timestamp_offset < 0 ||
      static_cast<uint32_t>(timestamp_offset + sizeof(double)) > cloud.point_step)
  {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Cannot align timestamps: FLOAT64 timestamp field is missing.");
    return false;
  }

  const double shift_ns =
    static_cast<double>(aligned_base_ns - input_base_ns);
  for (uint32_t row = 0; row < cloud.height; ++row) {
    for (uint32_t col = 0; col < cloud.width; ++col) {
      const size_t offset = static_cast<size_t>(row) * cloud.row_step +
        static_cast<size_t>(col) * cloud.point_step + timestamp_offset;
      if (offset + sizeof(double) > cloud.data.size()) {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Cannot align timestamps: malformed PointCloud2 buffer.");
        return false;
      }
      double point_timestamp;
      std::memcpy(&point_timestamp, cloud.data.data() + offset, sizeof(double));
      point_timestamp += shift_ns;
      std::memcpy(cloud.data.data() + offset, &point_timestamp, sizeof(double));
    }
  }
  return true;
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
