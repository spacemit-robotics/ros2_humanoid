// Copyright 2026 chenzhaoqi
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

struct PlanarPose
{
  double x;
  double y;
  double yaw;
};

using PlanarVelocity = std::array<double, 3>;

double quaternion_to_yaw(double x, double y, double z, double w)
{
  const double sin_yaw = 2.0 * (w * z + x * y);
  const double cos_yaw = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(sin_yaw, cos_yaw);
}

double normalize_angle(double angle)
{
  // Using atan2 keeps the result in [-pi, pi], matching the Python node.
  return std::atan2(std::sin(angle), std::cos(angle));
}

PlanarVelocity calculate_planar_velocity(
  const PlanarPose & previous_pose,
  const PlanarPose & current_pose,
  double dt)
{
  const double delta_x = current_pose.x - previous_pose.x;
  const double delta_y = current_pose.y - previous_pose.y;
  const double delta_yaw = normalize_angle(current_pose.yaw - previous_pose.yaw);

  const double cos_yaw = std::cos(previous_pose.yaw);
  const double sin_yaw = std::sin(previous_pose.yaw);
  const double translation_x = cos_yaw * delta_x + sin_yaw * delta_y;
  const double translation_y = -sin_yaw * delta_x + cos_yaw * delta_y;

  double coefficient_a;
  double coefficient_b;
  if (std::abs(delta_yaw) < 1e-6) {
    coefficient_a = 1.0 - delta_yaw * delta_yaw / 6.0;
    coefficient_b = delta_yaw / 2.0;
  } else {
    coefficient_a = std::sin(delta_yaw) / delta_yaw;
    coefficient_b = (1.0 - std::cos(delta_yaw)) / delta_yaw;
  }

  const double denominator =
    coefficient_a * coefficient_a + coefficient_b * coefficient_b;
  return {
    (coefficient_a * translation_x + coefficient_b * translation_y) / denominator / dt,
    (-coefficient_b * translation_x + coefficient_a * translation_y) / denominator / dt,
    delta_yaw / dt};
}

}  // namespace

class TfToOdomNode : public rclcpp::Node
{
public:
  TfToOdomNode()
  : Node("tf_to_odom_node")
  {
    parent_frame_ = declare_parameter<std::string>("parent_frame", "odom");
    child_frame_ = declare_parameter<std::string>("child_frame", "base_footprint");
    const double publish_rate = declare_parameter<double>("publish_rate", 50.0);
    velocity_filter_alpha_ = declare_parameter<double>("velocity_filter_alpha", 0.35);
    max_sample_interval_ = declare_parameter<double>("max_sample_interval", 1.0);
    max_linear_velocity_ = declare_parameter<double>("max_linear_velocity", 3.0);
    max_angular_velocity_ = declare_parameter<double>("max_angular_velocity", 6.0);
    declare_parameter<std::vector<double>>(
      "pose_covariance_diagonal", {0.0025, 0.0025, 1e6, 1e6, 1e6, 0.0012});
    declare_parameter<std::vector<double>>(
      "twist_covariance_diagonal", {0.01, 0.01, 1e6, 1e6, 1e6, 0.0076});

    require_positive_finite("publish_rate", publish_rate);
    if (!std::isfinite(velocity_filter_alpha_) ||
      velocity_filter_alpha_ <= 0.0 || velocity_filter_alpha_ > 1.0)
    {
      throw std::invalid_argument("velocity_filter_alpha must be in (0, 1]");
    }
    require_positive_finite("max_sample_interval", max_sample_interval_);
    require_positive_finite("max_linear_velocity", max_linear_velocity_);
    require_positive_finite("max_angular_velocity", max_angular_velocity_);

    pose_covariance_ = make_covariance("pose_covariance_diagonal");
    twist_covariance_ = make_covariance("twist_covariance_diagonal");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      std::bind(&TfToOdomNode::publish_odom, this));

    RCLCPP_INFO(
      get_logger(), "Publishing /odom from TF %s -> %s",
      parent_frame_.c_str(), child_frame_.c_str());
  }

private:
  static void require_positive_finite(const std::string & name, double value)
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(name + " must be greater than zero");
    }
  }

  std::array<double, 36> make_covariance(const std::string & parameter_name)
  {
    const auto diagonal = get_parameter(parameter_name).as_double_array();
    if (diagonal.size() != 6) {
      throw std::invalid_argument(parameter_name + " must contain 6 non-negative values");
    }

    std::array<double, 36> covariance{};
    constexpr std::array<std::size_t, 6> diagonal_indices{0, 7, 14, 21, 28, 35};
    for (std::size_t i = 0; i < diagonal.size(); ++i) {
      if (!std::isfinite(diagonal[i]) || diagonal[i] < 0.0) {
        throw std::invalid_argument(parameter_name + " must contain 6 non-negative values");
      }
      covariance[diagonal_indices[i]] = diagonal[i];
    }
    return covariance;
  }

  void set_previous_sample(
    std::int64_t stamp_ns, double x, double y, double yaw, bool reset_filter = false)
  {
    previous_stamp_ns_ = stamp_ns;
    previous_pose_ = {x, y, yaw};
    has_previous_sample_ = true;
    if (reset_filter) {
      has_filtered_velocity_ = false;
    }
  }

  void warn_throttled(const std::string & message)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", message.c_str());
  }

  void publish_odom()
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(parent_frame_, child_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & error) {
      warn_throttled(
        "Waiting for TF " + parent_frame_ + " -> " + child_frame_ + ": " + error.what());
      return;
    }

    const std::int64_t stamp_ns = rclcpp::Time(transform.header.stamp).nanoseconds();
    if (has_previous_sample_ && stamp_ns == previous_stamp_ns_) {
      return;
    }

    const auto & translation = transform.transform.translation;
    const auto & rotation = transform.transform.rotation;
    const double yaw = quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w);

    if (!has_previous_sample_) {
      set_previous_sample(stamp_ns, translation.x, translation.y, yaw, true);
      return;
    }

    const double dt = static_cast<double>(stamp_ns - previous_stamp_ns_) / 1e9;
    if (dt <= 0.0 || dt > max_sample_interval_) {
      warn_throttled(
        "Resetting velocity estimate after invalid TF interval: " +
        std::to_string(dt) + " s");
      set_previous_sample(stamp_ns, translation.x, translation.y, yaw, true);
      return;
    }

    const PlanarPose current_pose{translation.x, translation.y, yaw};
    const PlanarVelocity raw_velocity =
      calculate_planar_velocity(previous_pose_, current_pose, dt);
    const double linear_speed = std::hypot(raw_velocity[0], raw_velocity[1]);
    const bool velocity_is_finite =
      std::isfinite(raw_velocity[0]) && std::isfinite(raw_velocity[1]) &&
      std::isfinite(raw_velocity[2]);
    if (!velocity_is_finite || linear_speed > max_linear_velocity_ ||
      std::abs(raw_velocity[2]) > max_angular_velocity_)
    {
      warn_throttled(
        "Rejecting implausible TF velocity sample: linear=" +
        std::to_string(linear_speed) + " m/s, angular=" +
        std::to_string(raw_velocity[2]) + " rad/s");
      set_previous_sample(stamp_ns, translation.x, translation.y, yaw, true);
      return;
    }

    if (!has_filtered_velocity_) {
      filtered_velocity_ = raw_velocity;
      has_filtered_velocity_ = true;
    } else {
      for (std::size_t i = 0; i < filtered_velocity_.size(); ++i) {
        filtered_velocity_[i] = velocity_filter_alpha_ * raw_velocity[i] +
          (1.0 - velocity_filter_alpha_) * filtered_velocity_[i];
      }
    }

    nav_msgs::msg::Odometry odom;
    odom.header = transform.header;
    odom.header.frame_id = parent_frame_;
    odom.child_frame_id = child_frame_;
    odom.pose.pose.position.x = translation.x;
    odom.pose.pose.position.y = translation.y;
    odom.pose.pose.position.z = translation.z;
    odom.pose.pose.orientation = rotation;
    odom.pose.covariance = pose_covariance_;
    odom.twist.twist.linear.x = filtered_velocity_[0];
    odom.twist.twist.linear.y = filtered_velocity_[1];
    odom.twist.twist.angular.z = filtered_velocity_[2];
    odom.twist.covariance = twist_covariance_;

    odom_publisher_->publish(odom);
    set_previous_sample(stamp_ns, translation.x, translation.y, yaw);
  }

  std::string parent_frame_;
  std::string child_frame_;
  double velocity_filter_alpha_{0.35};
  double max_sample_interval_{1.0};
  double max_linear_velocity_{3.0};
  double max_angular_velocity_{6.0};
  std::array<double, 36> pose_covariance_{};
  std::array<double, 36> twist_covariance_{};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool has_previous_sample_{false};
  std::int64_t previous_stamp_ns_{0};
  PlanarPose previous_pose_{0.0, 0.0, 0.0};
  bool has_filtered_velocity_{false};
  PlanarVelocity filtered_velocity_{0.0, 0.0, 0.0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TfToOdomNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("tf_to_odom_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
