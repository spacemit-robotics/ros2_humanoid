/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file humanoid_nav_rl_bridge_node.cpp
 * @brief Bridge ROS2 cmd_vel to the humanoid operator service.
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "operator_client.h"
#include "operator_client_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
using Clock = std::chrono::steady_clock;

double Clamp(double value, double limit)
{
  return std::clamp(value, -std::abs(limit), std::abs(limit));
}

double Slew(double current, double target, double maximum_delta)
{
  return current + std::clamp(
    target - current, -maximum_delta, maximum_delta);
}
}  // namespace

class HumanoidNavRlBridge : public rclcpp::Node
{
public:
  HumanoidNavRlBridge()
  : Node("humanoid_nav_rl_bridge")
  {
    robot_config_path_ = humanoid_operator::ResolvePath(
      declare_parameter<std::string>("robot_config_path", ""));
    if (robot_config_path_.empty()) {
      throw std::invalid_argument("robot_config_path must not be empty");
    }
    cmd_vel_topic_ =
      declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    cmd_vel_timeout_s_ =
      declare_parameter<double>("accepted_cmd_vel_timeout_s", 0.5);
    publish_rate_hz_ = std::max(
      1.0, declare_parameter<double>("publish_rate_hz", 20.0));
    auto_fsm_ = declare_parameter<bool>("auto_fsm", false);
    auto_policy_ =
      declare_parameter<std::string>("auto_policy", "walk_mjlab");
    auto_request_period_s_ =
      declare_parameter<double>("auto_request_period_s", 0.5);
    require_zero_ready_ =
      declare_parameter<bool>("require_zero_ready", true);
    send_poweroff_on_shutdown_ =
      declare_parameter<bool>("send_poweroff_on_shutdown", false);
    velocity_ttl_ms_ = static_cast<int>(std::clamp<int64_t>(
        declare_parameter<int64_t>("velocity_ttl_ms", 300), 50, 1000));

    max_vx_ = declare_parameter<double>("max_vx", 0.3);
    max_vy_ = declare_parameter<double>("max_vy", 0.25);
    max_wz_ = declare_parameter<double>("max_wz", 0.5);
    max_accel_vx_ = declare_parameter<double>("max_accel_vx", 0.4);
    max_accel_vy_ = declare_parameter<double>("max_accel_vy", 0.3);
    max_accel_wz_ = declare_parameter<double>("max_accel_wz", 0.8);

    const int socket_timeout_ms = static_cast<int>(std::clamp<int64_t>(
        declare_parameter<int64_t>("socket_timeout_ms", 1500),
        100, 10000));
    connection_file_ = humanoid_operator::ConnectionFile(
      robot_config_path_,
      declare_parameter<std::string>("connection_file", ""));
    humanoid_operator::Connect(
      &client_, connection_file_, "ros_nav", socket_timeout_ms);
    const auto acquire = client_.AcquireControl();
    if (!acquire.ok) {
      throw std::runtime_error(
              "cannot acquire operator control: " +
              acquire.code + ": " + acquire.message);
    }

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::Twist::SharedPtr message)
      {
        target_.vx = Clamp(message->linear.x, max_vx_);
        target_.vy = Clamp(message->linear.y, max_vy_);
        target_.wz = Clamp(message->angular.z, max_wz_);
        last_cmd_vel_ = Clock::now();
        have_cmd_vel_ = true;
      });
    status_pub_ = create_publisher<std_msgs::msg::String>("~/status", 10);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_)),
      [this]() {Tick();});
    RCLCPP_INFO(
      get_logger(),
      "Nav bridge connected to hmi_runtime via %s; cmd_vel=%s "
      "auto_fsm=%s policy=%s", connection_file_.c_str(),
      cmd_vel_topic_.c_str(), auto_fsm_ ? "true" : "false",
      auto_policy_.c_str());
  }

  ~HumanoidNavRlBridge() override
  {
    if (!client_.Connected()) {
      return;
    }
    (void)client_.SetVelocity({}, velocity_ttl_ms_);
    if (send_poweroff_on_shutdown_) {
      (void)client_.RequestState("POWER_OFF");
    } else {
      (void)client_.ReleaseControl();
    }
  }

private:
  void Tick()
  {
    const auto now = Clock::now();
    const auto status = client_.LatestStatus();
    if (client_.Connected() && status.owns_control &&
      now - last_renew_ >= std::chrono::milliseconds(250))
    {
      const auto reply = client_.RenewControl();
      if (!reply.ok) {
        RCLCPP_WARN(
          get_logger(), "control renewal failed: %s",
          reply.message.c_str());
      }
      last_renew_ = now;
    }
    if (auto_fsm_) {
      AdvanceFsm(status, now);
    }

    const double dt = last_tick_.time_since_epoch().count() == 0 ?
      1.0 / publish_rate_hz_ :
      std::chrono::duration<double>(now - last_tick_).count();
    last_tick_ = now;
    const bool fresh = have_cmd_vel_ &&
      std::chrono::duration<double>(now - last_cmd_vel_).count() <=
      cmd_vel_timeout_s_;
    const bool gate_open = status.online && status.owns_control &&
      status.state == "RL" &&
      (auto_policy_.empty() || status.policy == auto_policy_);
    const operator_service::Velocity desired = fresh && gate_open ?
      target_ : operator_service::Velocity{};
    current_.vx = Slew(current_.vx, desired.vx, max_accel_vx_ * dt);
    current_.vy = Slew(current_.vy, desired.vy, max_accel_vy_ * dt);
    current_.wz = Slew(current_.wz, desired.wz, max_accel_wz_ * dt);
    operator_service::Reply velocity_reply;
    if (status.online && status.owns_control && status.state == "RL") {
      velocity_reply = client_.SetVelocity(current_, velocity_ttl_ms_);
    }
    if (!velocity_reply.ok && gate_open) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "velocity rejected: %s", velocity_reply.message.c_str());
    }
    PublishStatus(status, fresh, gate_open);
  }

  void AdvanceFsm(
    const operator_service::Status & status,
    const Clock::time_point & now)
  {
    if (!status.online || !status.owns_control || status.fault.latched ||
      humanoid_operator::RequestPending(status) ||
      now - last_auto_request_ <
      std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(auto_request_period_s_)))
    {
      return;
    }
    operator_service::Reply reply;
    bool requested = true;
    if (!auto_policy_.empty() && status.policy != auto_policy_ &&
      status.state != "POWER_OFF" && status.state != "DAMP")
    {
      reply = client_.RequestState("DAMP");
    } else if (!auto_policy_.empty() && status.policy != auto_policy_) {
      reply = client_.SelectPolicy(auto_policy_);
    } else if (status.state == "POWER_OFF") {
      reply = client_.RequestState("DAMP");
    } else if (status.state == "DAMP") {
      reply = client_.RequestState("HOME");
    } else if (status.state == "HOME") {
      reply = client_.RequestState("ZERO");
    } else if (status.state == "ZERO" &&
      (!require_zero_ready_ || status.zero_ready))
    {
      reply = client_.RequestState("RL");
    } else {
      requested = false;
    }
    if (requested) {
      last_auto_request_ = now;
      if (!reply.ok && reply.code != "busy") {
        RCLCPP_WARN(
          get_logger(), "automatic transition failed: %s",
          reply.message.c_str());
      }
    }
  }

  void PublishStatus(
    const operator_service::Status & status,
    bool command_fresh, bool gate_open)
  {
    std_msgs::msg::String message;
    message.data = std::string("control=") +
      (status.online ? "online" : "offline") +
      " mode=" + status.state + " policy=" + status.policy +
      " owner=" + (status.owns_control ? "self" : status.owner) +
      " cmd_vel=" + (command_fresh ? "fresh" : "stale") +
      " velocity_gate=" + (gate_open ? "open" : "closed") +
      " sent=(" + std::to_string(current_.vx) + "," +
      std::to_string(current_.vy) + "," +
      std::to_string(current_.wz) + ")";
    status_pub_->publish(message);
  }

  operator_service::Client client_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string robot_config_path_;
  std::string connection_file_;
  std::string cmd_vel_topic_;
  std::string auto_policy_;
  operator_service::Velocity target_;
  operator_service::Velocity current_;
  bool auto_fsm_ = false;
  bool require_zero_ready_ = true;
  bool send_poweroff_on_shutdown_ = false;
  bool have_cmd_vel_ = false;
  int velocity_ttl_ms_ = 300;
  double cmd_vel_timeout_s_ = 0.5;
  double publish_rate_hz_ = 20.0;
  double auto_request_period_s_ = 0.5;
  double max_vx_ = 0.3;
  double max_vy_ = 0.25;
  double max_wz_ = 0.5;
  double max_accel_vx_ = 0.4;
  double max_accel_vy_ = 0.3;
  double max_accel_wz_ = 0.8;
  Clock::time_point last_cmd_vel_{};
  Clock::time_point last_tick_{};
  Clock::time_point last_renew_{};
  Clock::time_point last_auto_request_{};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<HumanoidNavRlBridge>());
  } catch (const std::exception & error) {
    std::fprintf(stderr, "[humanoid_nav_rl_bridge] %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
