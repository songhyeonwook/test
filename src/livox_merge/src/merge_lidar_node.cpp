#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>
#include <sstream>
#include <limits>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define RESET "\033[0m"

struct PointOuster
{
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;
  uint16_t reflectivity;
  uint8_t ring;
  uint16_t ambient;
  uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointOuster,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint32_t, t, t)
                                  (uint16_t, reflectivity, reflectivity)
                                  (uint8_t, ring, ring)
                                  (uint16_t, ambient, ambient)
                                  (uint32_t, range, range))

// livox_ros_driver2 가 xfer_format=0 으로 낼 때의 PointCloud2 레이아웃.
// (bag 으로 기록된 것도 이 형식이다)
struct PointLivoxPc2
{
  PCL_ADD_POINT4D;
  float intensity;
  uint8_t tag;
  uint8_t line;
  double timestamp;      // 절대시각 [ns]
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

typedef pcl::PointCloud<PointOuster> CloudOuster;
typedef pcl::PointCloud<PointOuster>::Ptr CloudOusterPtr;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointLivoxPc2,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint8_t, tag, tag)
                                  (std::uint8_t, line, line)
                                  (double, timestamp, timestamp))

struct CloudPacket
{
  double start_time;
  double end_time;
  CloudOusterPtr cloud;

  CloudPacket() = default;
  CloudPacket(double start, double end, CloudOusterPtr c)
  : start_time(start), end_time(end), cloud(std::move(c)) {}
};

static Eigen::Matrix4d flatToMatrix4d(const std::vector<double> &flat)
{
  if (flat.size() != 16) {
    throw std::runtime_error("Extrinsic must contain exactly 16 values.");
  }

  Eigen::Matrix4d T;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      T(r, c) = flat[r * 4 + c];
    }
  }
  return T;
}

class MergeLidarNode : public rclcpp::Node
{
public:
  MergeLidarNode()
  : Node("merge_lidar"),
    lidar_ring_offset_set_(false),
    max_threads_(std::thread::hardware_concurrency()),
    n_lidar_(0),
    cutoff_time_(-1.0),
    cutoff_time_new_(-1.0),
    sync_frequency_(20.0),
    sync_period_(1.0 / 20.0),
    slice_z_min_(-2.0),
    slice_z_max_(2.0),
    running_(true)
  {
    initialize();
    sync_thread_ = std::thread(&MergeLidarNode::syncLidar, this);
  }

  ~MergeLidarNode() override
  {
    running_ = false;
    if (sync_thread_.joinable()) {
      sync_thread_.join();
    }
  }

private:
  std::vector<rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr> lidar_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> pc2_subs_;
  std::string input_type_;

  std::mutex lidar_buf_mtx_;
  std::deque<std::deque<CloudPacket>> lidar_buf_;
  std::deque<std::deque<CloudPacket>> lidar_leftover_buf_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_pc_pub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr merged_livox_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_pc_sliced_pub_;

  std::deque<Eigen::Matrix3d> R_B_L_;
  std::deque<Eigen::Vector3d> t_B_L_;

  std::vector<int> lidar_channels_;
  std::deque<int> lidar_ring_offset_;
  bool lidar_ring_offset_set_;
  std::mutex channel_mutex_;

  int max_threads_;
  int n_lidar_;

  double cutoff_time_;
  double cutoff_time_new_;
  double sync_frequency_;
  double sync_period_;

  double slice_z_min_;
  double slice_z_max_;
  double slice_crop_half_x_;
  double slice_crop_half_y_;

  std::thread sync_thread_;
  bool running_;

  void initialize()
  {
    this->declare_parameter<std::vector<std::string>>("lidars.topics", {"/livox/lidar"});
    this->declare_parameter<double>("sync_frequency", 20.0);
    this->declare_parameter<double>("slice_z_min", -2.0);
    this->declare_parameter<double>("slice_z_max", 2.0);
    // sliced 출력에서 |x|<=half_x 이고 |y|<=half_y 인 점(차체 셀프 반사)을
    // 버린다. 라이다가 차체 모서리에 있어 몸체를 보기 때문에, 이 크롭이
    // 없으면 costmap 이 차량 위치를 장애물로 찍는다. 0 이면 비활성.
    this->declare_parameter<double>("slice_crop_half_x", 0.0);
    this->declare_parameter<double>("slice_crop_half_y", 0.0);
    this->declare_parameter<std::string>("output.pointcloud2", "/livox_merge/merged_pointcloud");
    this->declare_parameter<std::string>("output.livox_custom", "/livox_merge/merged_livox");
    this->declare_parameter<std::string>("output.pointcloud2_sliced", "/livox_merge/merged_pointcloud_sliced");
    this->declare_parameter<std::string>("output.frame_id", "body");
    // "custom"      : livox_ros_driver2/CustomMsg  (xfer_format=1)
    // "pointcloud2" : sensor_msgs/PointCloud2      (xfer_format=0, bag 기록물)
    this->declare_parameter<std::string>("input_type", "custom");

    input_type_ = this->get_parameter("input_type").as_string();
    const auto lidar_topics = this->get_parameter("lidars.topics").as_string_array();
    n_lidar_ = static_cast<int>(lidar_topics.size());

    sync_frequency_ = this->get_parameter("sync_frequency").as_double();
    if (sync_frequency_ <= 0.0) {
      sync_frequency_ = 20.0;
    }
    sync_period_ = 1.0 / sync_frequency_;

    slice_z_min_ = this->get_parameter("slice_z_min").as_double();
    slice_z_max_ = this->get_parameter("slice_z_max").as_double();
    slice_crop_half_x_ = this->get_parameter("slice_crop_half_x").as_double();
    slice_crop_half_y_ = this->get_parameter("slice_crop_half_y").as_double();

    const auto out_pc = this->get_parameter("output.pointcloud2").as_string();
    const auto out_livox = this->get_parameter("output.livox_custom").as_string();
    const auto out_sliced = this->get_parameter("output.pointcloud2_sliced").as_string();

    merged_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(out_pc, rclcpp::SensorDataQoS());
    merged_livox_pub_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>(out_livox, rclcpp::SensorDataQoS());
    merged_pc_sliced_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(out_sliced, rclcpp::SensorDataQoS());

    for (int i = 0; i < n_lidar_; ++i) {
      const std::string param_name = "lidars.extrinsics.lidar_" + std::to_string(i);
      this->declare_parameter<std::vector<double>>(param_name, std::vector<double>{
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
      });

      const auto extr_flat = this->get_parameter(param_name).as_double_array();
      const auto T = flatToMatrix4d(extr_flat);
      R_B_L_.push_back(T.block<3, 3>(0, 0));
      t_B_L_.push_back(T.block<3, 1>(0, 3));

      lidar_buf_.push_back({});
      lidar_leftover_buf_.push_back({});

      if (input_type_ == "pointcloud2") {
        auto cb = [this, i](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          this->pcHandlerPointCloud2(msg, i);
        };
        pc2_subs_.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
          lidar_topics[i], rclcpp::SensorDataQoS(), cb));
      } else {
        auto cb = [this, i](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
          this->pcHandlerLivox(msg, i);
        };
        lidar_subs_.push_back(this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
          lidar_topics[i], rclcpp::SensorDataQoS(), cb));
      }

      RCLCPP_INFO(this->get_logger(), "Subscribed lidar %d: %s (%s)",
                  i, lidar_topics[i].c_str(), input_type_.c_str());
    }

    lidar_channels_ = std::vector<int>(n_lidar_, -1);
    lidar_ring_offset_ = std::deque<int>(n_lidar_, 0);

    RCLCPP_INFO(this->get_logger(), "Initialized MergeLidar without IMU, lidar count: %d", n_lidar_);
    RCLCPP_INFO(this->get_logger(), "Sync frequency: %.1f Hz", sync_frequency_);
  }

  bool checkLidarChannel(const CloudOusterPtr &cloud, int idx)
  {
    std::lock_guard<std::mutex> lock(channel_mutex_);

    if (idx < 0 || idx >= n_lidar_ || cloud->empty()) {
      return false;
    }

    uint8_t max_ring = 0;
    for (const auto &pt : cloud->points) {
      max_ring = std::max(max_ring, pt.ring);
    }

    lidar_channels_[idx] = static_cast<int>(max_ring) + 1;

    bool ready = true;
    for (int i = 0; i < n_lidar_; ++i) {
      if (lidar_channels_[i] <= 0) {
        ready = false;
        break;
      }
    }

    if (!ready) {
      return false;
    }

    lidar_ring_offset_[0] = 0;
    for (int i = 1; i < n_lidar_; ++i) {
      lidar_ring_offset_[i] = lidar_ring_offset_[i - 1] + lidar_channels_[i - 1];
    }

    RCLCPP_INFO(this->get_logger(), "Lidar ring offsets initialized.");
    return true;
  }

  void pcHandlerLivox(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg, int idx)
  {
    if (msg->points.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty cloud from lidar %d", idx);
      return;
    }

    double end_time = rclcpp::Time(msg->header.stamp).seconds();
    double start_time = end_time - 0.1;

    CloudOusterPtr cloud_in_l(new CloudOuster());
    const int points_total = static_cast<int>(msg->points.size());
    cloud_in_l->resize(points_total);

    for (int i = 0; i < points_total; ++i) {
      const auto &src = msg->points[i];
      auto &dst = cloud_in_l->points[i];
      dst.x = src.x;
      dst.y = src.y;
      dst.z = src.z;
      dst.intensity = static_cast<float>(src.reflectivity);
      dst.reflectivity = src.reflectivity;
      dst.t = src.offset_time;
      dst.ring = src.line;
      dst.ambient = 0;
      dst.range = static_cast<uint32_t>(std::sqrt(src.x * src.x + src.y * src.y + src.z * src.z) * 1000.0);
    }

    if (!lidar_ring_offset_set_) {
      lidar_ring_offset_set_ = checkLidarChannel(cloud_in_l, idx);
    }
    if (!lidar_ring_offset_set_) {
      return;
    }

    CloudOusterPtr cloud_in_b(new CloudOuster());
    cloud_in_b->resize(points_total);

    for (int i = 0; i < points_total; ++i) {
      const auto &point_in_l = cloud_in_l->points[i];
      Eigen::Vector3d p_l(point_in_l.x, point_in_l.y, point_in_l.z);
      Eigen::Vector3d p_b = R_B_L_[idx] * p_l + t_B_L_[idx];

      auto point_in_b = point_in_l;
      point_in_b.x = static_cast<float>(p_b.x());
      point_in_b.y = static_cast<float>(p_b.y());
      point_in_b.z = static_cast<float>(p_b.z());
      point_in_b.ring = static_cast<uint8_t>(point_in_b.ring + lidar_ring_offset_[idx]);
      cloud_in_b->points[i] = point_in_b;
    }

    {
      std::lock_guard<std::mutex> lock(lidar_buf_mtx_);
      lidar_buf_[idx].push_back(CloudPacket(start_time, end_time, cloud_in_b));
      constexpr std::size_t max_buffer_size = 50;
      if (lidar_buf_[idx].size() > max_buffer_size) {
        lidar_buf_[idx].pop_front();
      }
    }
  }

  // xfer_format=0 으로 나온 PointCloud2 를 받는 경로.
  // CustomMsg 경로(pcHandlerLivox)와 같은 PointOuster 버퍼로 정규화한다.
  void pcHandlerPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int idx)
  {
    pcl::PointCloud<PointLivoxPc2> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    const int points_total = static_cast<int>(pl_orig.points.size());
    if (points_total == 0) {
      RCLCPP_WARN(this->get_logger(), "Received empty cloud from lidar %d", idx);
      return;
    }

    // 포인트별 timestamp 는 절대시각[ns] 이다. 프레임 시작을 기준으로 offset 을 만든다.
    // timestamp 가 0 인 형식이면 header.stamp 로 떨어뜨린다.
    double ts_min = std::numeric_limits<double>::max();
    double ts_max = std::numeric_limits<double>::lowest();
    for (const auto &pt : pl_orig.points) {
      if (pt.timestamp <= 0.0) continue;
      ts_min = std::min(ts_min, pt.timestamp);
      ts_max = std::max(ts_max, pt.timestamp);
    }
    const bool has_ts = ts_min <= ts_max;
    const double header_time = rclcpp::Time(msg->header.stamp).seconds();
    const double start_time = has_ts ? ts_min * 1e-9 : header_time;
    const double end_time = has_ts ? ts_max * 1e-9 : header_time + 0.1;

    CloudOusterPtr cloud_in_l(new CloudOuster());
    cloud_in_l->resize(points_total);
    for (int i = 0; i < points_total; ++i) {
      const auto &src = pl_orig.points[i];
      auto &dst = cloud_in_l->points[i];
      dst.x = src.x;
      dst.y = src.y;
      dst.z = src.z;
      dst.intensity = src.intensity;
      dst.reflectivity = static_cast<uint16_t>(src.intensity);
      dst.t = has_ts ? static_cast<uint32_t>(std::max(0.0, src.timestamp - ts_min)) : 0u;
      dst.ring = src.line;
      dst.ambient = 0;
      dst.range = static_cast<uint32_t>(std::sqrt(src.x * src.x + src.y * src.y + src.z * src.z) * 1000.0);
    }

    if (!lidar_ring_offset_set_) {
      lidar_ring_offset_set_ = checkLidarChannel(cloud_in_l, idx);
    }
    if (!lidar_ring_offset_set_) {
      return;
    }

    CloudOusterPtr cloud_in_b(new CloudOuster());
    cloud_in_b->resize(points_total);
    for (int i = 0; i < points_total; ++i) {
      const auto &point_in_l = cloud_in_l->points[i];
      Eigen::Vector3d p_l(point_in_l.x, point_in_l.y, point_in_l.z);
      Eigen::Vector3d p_b = R_B_L_[idx] * p_l + t_B_L_[idx];

      auto point_in_b = point_in_l;
      point_in_b.x = static_cast<float>(p_b.x());
      point_in_b.y = static_cast<float>(p_b.y());
      point_in_b.z = static_cast<float>(p_b.z());
      point_in_b.ring = static_cast<uint8_t>(point_in_b.ring + lidar_ring_offset_[idx]);
      cloud_in_b->points[i] = point_in_b;
    }

    {
      std::lock_guard<std::mutex> lock(lidar_buf_mtx_);
      lidar_buf_[idx].push_back(CloudPacket(start_time, end_time, cloud_in_b));
      constexpr std::size_t max_buffer_size = 50;
      if (lidar_buf_[idx].size() > max_buffer_size) {
        lidar_buf_[idx].pop_front();
      }
    }
  }

  void publishPointCloud2(const CloudOuster &cloud, const rclcpp::Time &stamp, const std::string &frame)
  {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = frame;
    merged_pc_pub_->publish(msg);
  }

  void publishSlicedPointCloud2(const CloudOuster &cloud, const rclcpp::Time &stamp, const std::string &frame)
  {
    const bool crop_enabled = slice_crop_half_x_ > 0.0 && slice_crop_half_y_ > 0.0;
    CloudOuster sliced;
    for (const auto &pt : cloud.points) {
      if (pt.z < slice_z_min_ || pt.z > slice_z_max_) {
        continue;
      }
      if (crop_enabled &&
          std::abs(pt.x) <= slice_crop_half_x_ &&
          std::abs(pt.y) <= slice_crop_half_y_) {
        continue;
      }
      sliced.push_back(pt);
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(sliced, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = frame;
    merged_pc_sliced_pub_->publish(msg);
  }

  void publishLivoxCloud(const CloudOuster &cloud, const rclcpp::Time &stamp, const std::string &frame)
  {
    livox_ros_driver2::msg::CustomMsg out;
    out.header.stamp = stamp;
    out.header.frame_id = frame;
    out.timebase = stamp.nanoseconds();
    out.point_num = static_cast<uint32_t>(cloud.size());
    out.lidar_id = 255;
    out.points.resize(cloud.size());

    for (std::size_t i = 0; i < cloud.size(); ++i) {
      const auto &src = cloud.points[i];
      auto &dst = out.points[i];
      dst.x = src.x;
      dst.y = src.y;
      dst.z = src.z;
      dst.reflectivity = src.reflectivity;
      dst.offset_time = src.t;
      dst.line = src.ring;
      dst.tag = 0;
    }

    merged_livox_pub_->publish(out);
  }

  void syncLidar()
  {
    rclcpp::WallRate rate(sync_frequency_);
    const std::string frame_id = this->get_parameter("output.frame_id").as_string();

    while (rclcpp::ok() && running_) {
      std::vector<CloudPacket> packets;
      bool ready = true;
      double window_end = std::numeric_limits<double>::max();

      {
        std::lock_guard<std::mutex> lock(lidar_buf_mtx_);
        if (static_cast<int>(lidar_buf_.size()) != n_lidar_) {
          ready = false;
        } else {
          for (int i = 0; i < n_lidar_; ++i) {
            if (lidar_buf_[i].empty()) {
              ready = false;
              break;
            }
            window_end = std::min(window_end, lidar_buf_[i].front().end_time);
          }
        }

        if (ready) {
          packets.resize(n_lidar_);
          for (int i = 0; i < n_lidar_; ++i) {
            packets[i] = lidar_buf_[i].front();
            lidar_buf_[i].pop_front();
          }
        }
      }

      if (!ready) {
        rate.sleep();
        continue;
      }

      CloudOuster merged;
      for (const auto &pkt : packets) {
        merged += *(pkt.cloud);
      }

      const rclcpp::Time stamp(static_cast<int64_t>(window_end * 1e9));
      publishPointCloud2(merged, stamp, frame_id);
      publishLivoxCloud(merged, stamp, frame_id);
      publishSlicedPointCloud2(merged, stamp, frame_id);

      rate.sleep();
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MergeLidarNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
