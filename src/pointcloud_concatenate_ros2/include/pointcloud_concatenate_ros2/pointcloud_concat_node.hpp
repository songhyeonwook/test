#pragma once
#ifndef POINTCLOUD_CONCAT__POINTCLOUD_CONCAT_HPP_
#define POINTCLOUD_CONCAT__POINTCLOUD_CONCAT_HPP_

#include <array>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "pcl_ros/transforms.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace pointcloud_concatenate
{

constexpr int kMaxClouds = 4;

class PointCloudConcatNode : public rclcpp::Node {
private:

  void subCallbackCloudIn(sensor_msgs::msg::PointCloud2::SharedPtr msg, int idx);
  void publishPointcloud(sensor_msgs::msg::PointCloud2 & cloud, const rclcpp::Time & stamp);
  sensor_msgs::msg::PointCloud2 sliceCloud(const sensor_msgs::msg::PointCloud2 & in) const;

  std::string param_frame_target_;
  int param_clouds_;
  double param_hz_;
  bool param_sync_all_;
  bool param_stamp_from_input_;
  bool param_slice_enable_;
  double param_slice_z_min_, param_slice_z_max_;
  double param_crop_half_x_, param_crop_half_y_;

  rclcpp::SensorDataQoS sensor_qos;

  std::array<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr, kMaxClouds> sub_cloud_in;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_out;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_out_sliced;
  rclcpp::TimerBase::SharedPtr timer_;

  std::array<sensor_msgs::msg::PointCloud2, kMaxClouds> cloud_in;
  // 마지막으로 발행한 뒤 새 메시지가 들어왔는지. sync_all 이면 이게 전부 true 일
  // 때만 발행한다(같은 프레임을 두 번 쓰거나 건너뛰지 않게).
  std::array<bool, kMaxClouds> cloud_fresh {};
  std::array<bool, kMaxClouds> cloud_received {};

  std::unique_ptr<tf2_ros::Buffer> tfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> tfListener;

public:
  explicit PointCloudConcatNode(const rclcpp::NodeOptions & options);
  ~PointCloudConcatNode() override;

  void handleParams();
  void update();
};

}

#endif
