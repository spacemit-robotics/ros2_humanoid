/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file humanoid_cmd_vel_hmi_node.cpp
 * @brief ROS Twist and legacy ZMQ adapter for the operator service.
 */
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>
#include <zmq.h>

#include "geometry_msgs/msg/twist.hpp"
#include "operator_client.h"
#include "operator_client_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
using Clock = std::chrono::steady_clock;

class CommandSocket
{
public:
  explicit CommandSocket(const std::string & endpoint)
  {
    context_ = zmq_ctx_new();
    if (!context_) {
      throw std::runtime_error(zmq_strerror(errno));
    }
    socket_ = zmq_socket(context_, ZMQ_REP);
    if (socket_) {
      const int linger = 0;
      (void)zmq_setsockopt(socket_, ZMQ_LINGER, &linger, sizeof(linger));
    }
    if (!socket_ || zmq_bind(socket_, endpoint.c_str()) != 0) {
      const std::string error = zmq_strerror(errno);
      if (socket_) {
        zmq_close(socket_);
      }
      zmq_ctx_term(context_);
      throw std::runtime_error("ZMQ bind " + endpoint + ": " + error);
    }
  }

  ~CommandSocket()
  {
    if (socket_) {
      zmq_close(socket_);
    }
    if (context_) {
      zmq_ctx_term(context_);
    }
  }

  template<typename Handler>
  void Poll(Handler && handler)
  {
    char buffer[4096];
    const int size = zmq_recv(socket_, buffer, sizeof(buffer), ZMQ_DONTWAIT);
    if (size < 0) {
      return;
    }
    nlohmann::json reply;
    if (size >= static_cast<int>(sizeof(buffer))) {
      reply = {{"ok", false}, {"error", "request too large"}};
    } else {
      try {
        reply = handler(nlohmann::json::parse(buffer, buffer + size));
      } catch (const std::exception & error) {
        reply = {{"ok", false}, {"error", error.what()}};
      }
    }
    const std::string encoded = reply.dump();
    (void)zmq_send(socket_, encoded.data(), encoded.size(), 0);
  }

private:
  void * context_ = nullptr;
  void * socket_ = nullptr;
};

nlohmann::json ReplyJson(const operator_service::Reply & reply)
{
  return {{"ok", reply.ok}, {"code", reply.code},
    {"message", reply.message}};
}
}  // namespace

class HumanoidCmdVelClient : public rclcpp::Node
{
public:
  explicit HumanoidCmdVelClient(const std::string & robot_config_path)
  : Node("humanoid_cmd_vel_hmi")
  {
    const std::string cmd_vel_topic =
      declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    cmd_vel_timeout_s_ =
      std::max(0.1, declare_parameter<double>("cmd_vel_timeout_s", 0.5));
    bias_x_ = declare_parameter<double>("cmd_vel_bias_x", 0.0);
    bias_y_ = declare_parameter<double>("cmd_vel_bias_y", 0.0);
    bias_yaw_ = declare_parameter<double>("cmd_vel_bias_yaw", 0.0);
    velocity_ttl_ms_ = static_cast<int>(std::clamp<int64_t>(
        declare_parameter<int64_t>("velocity_ttl_ms", 300), 50, 1000));
    request_period_s_ = std::max(
      0.1,
      declare_parameter<double>("request_period_s", 0.5));
    const int socket_timeout_ms = static_cast<int>(std::clamp<int64_t>(
        declare_parameter<int64_t>("socket_timeout_ms", 1500),
        100, 10000));
    walk_policy_ =
      declare_parameter<std::string>("walk_policy", "walk_mjlab");
    stand_policy_ =
      declare_parameter<std::string>("stand_policy", "stand_mjlab");
    const std::string connection_file = humanoid_operator::ConnectionFile(
      robot_config_path,
      declare_parameter<std::string>("connection_file", ""));
    humanoid_operator::Connect(
      &client_, connection_file, "ros_cmd_vel", socket_timeout_ms);
    const auto acquire = client_.AcquireControl();
    if (!acquire.ok) {
      throw std::runtime_error(
              "cannot acquire operator control: " +
              acquire.code + ": " + acquire.message);
    }

    const std::string zmq_endpoint = declare_parameter<std::string>(
      "zmq_endpoint", "tcp://127.0.0.1:5565");
    command_socket_ = std::make_unique<CommandSocket>(zmq_endpoint);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::Twist::SharedPtr message)
      {
        if (Clock::now() < voice_override_until_) {
          return;
        }
        SetTarget(
          message->linear.x, message->linear.y,
          message->angular.z, cmd_vel_timeout_s_);
      });
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), [this]() {Tick();});
    RCLCPP_INFO(
      get_logger(),
      "Connected to hmi_runtime via %s; cmd_vel=%s ZMQ=%s",
      connection_file.c_str(), cmd_vel_topic.c_str(),
      zmq_endpoint.c_str());
  }

  ~HumanoidCmdVelClient() override
  {
    if (client_.Connected()) {
      (void)client_.Stop();
    }
  }

private:
  bool SetTarget(double vx, double vy, double wz, double timeout_s)
  {
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) {
      return false;
    }
    const bool moving = vx != 0.0 || vy != 0.0 || wz != 0.0;
    target_ = {vx + (moving ? bias_x_ : 0.0),
      vy + (moving ? bias_y_ : 0.0),
      wz + (moving ? bias_yaw_ : 0.0)};
    command_until_ = Clock::now() +
      std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(timeout_s));
    return true;
  }

  void Tick()
  {
    command_socket_->Poll(
      [this](const nlohmann::json & request) {return Handle(request);});
    const auto now = Clock::now();
    const auto status = client_.LatestStatus();
    if (!client_.Connected()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "operator service disconnected: %s",
        client_.LastError().c_str());
      return;
    }
    if (status.owns_control &&
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
    AdvancePolicy(status, now);
    if (now - last_velocity_send_ < std::chrono::milliseconds(100)) {
      return;
    }
    operator_service::Velocity command;
    const bool ready = status.online && status.owns_control &&
      status.state == "RL" && status.policy == walk_policy_;
    if (!status.online || !status.owns_control || status.state != "RL") {
      last_velocity_send_ = now;
      return;
    }
    if (ready && now <= command_until_) {
      command = target_;
    }
    const auto reply = client_.SetVelocity(command, velocity_ttl_ms_);
    if (!reply.ok) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "velocity rejected: %s", reply.message.c_str());
    }
    last_velocity_send_ = now;
  }

  void AdvancePolicy(
    const operator_service::Status & status,
    const Clock::time_point & now)
  {
    if ((!desired_trajectory_ && desired_policy_.empty()) || !status.online ||
      !status.owns_control || status.fault.latched ||
      humanoid_operator::RequestPending(status) ||
      now - last_request_ <
      std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(request_period_s_)))
    {
      return;
    }
    operator_service::Reply reply;
    bool requested = true;
    if (desired_trajectory_) {
      if (status.state == "POWER_OFF") {
        reply = client_.RequestState("DAMP");
      } else if (status.state == "DAMP") {
        reply = client_.RequestState("HOME");
      } else if (status.state == "HOME") {
        reply = client_.RequestState("ZERO");
      } else if (status.state == "ZERO" && status.zero_ready) {
        reply = client_.RequestState("TRAJECTORY");
      } else if (status.state == "TRAJECTORY") {
        desired_trajectory_ = false;
        requested = false;
      } else {
        reply = client_.RequestState("DAMP");
      }
    } else if (status.policy != desired_policy_ && status.state != "POWER_OFF" &&
      status.state != "DAMP")
    {
      reply = client_.RequestState("DAMP");
    } else if (status.policy != desired_policy_) {
      reply = client_.SelectPolicy(desired_policy_);
    } else if (status.state == "POWER_OFF") {
      reply = client_.RequestState("DAMP");
    } else if (status.state == "DAMP") {
      reply = client_.RequestState("HOME");
    } else if (status.state == "HOME") {
      reply = client_.RequestState("ZERO");
    } else if (status.state == "ZERO" && status.zero_ready) {
      reply = client_.RequestState("RL");
    } else if (status.state == "RL") {
      desired_policy_.clear();
      requested = false;
    } else {
      requested = false;
    }
    if (requested) {
      last_request_ = now;
      if (!reply.ok && reply.code != "busy") {
        RCLCPP_WARN(
          get_logger(), "automatic transition failed: %s",
          reply.message.c_str());
      }
    }
  }

  nlohmann::json Handle(const nlohmann::json & request)
  {
    if (!request.is_object()) {
      return {{"ok", false}, {"error", "request must be an object"}};
    }
    const std::string operation = request.value("op", std::string{});
    const auto status = client_.LatestStatus();
    if (operation == "status") {
      return {{"ok", true}, {"online", status.online},
        {"mode", status.state}, {"policy", status.policy},
        {"trajectory_enabled", status.trajectory_enabled},
        {"desired_policy", desired_policy_},
        {"desired_mode", status.trajectory_enabled ? "TRAJECTORY" : "RL"},
        {"switching", desired_trajectory_ || !desired_policy_.empty()},
        {"interaction_phase", status.interaction_phase},
        {"fault", status.fault.latched},
        {"owns_control", status.owns_control}};
    }
    if (operation == "walk" || operation == "stand" ||
      operation == "stop")
    {
      if (operation == "walk" && status.trajectory_enabled) {
        return {{"ok", false},
          {"error", "walking is unavailable in trajectory mode"}};
      }
      target_ = {};
      command_until_ = Clock::time_point{};
      desired_trajectory_ = status.trajectory_enabled;
      desired_policy_ = desired_trajectory_ ? "" :
        (operation == "walk" ? walk_policy_ : stand_policy_);
      return {{"ok", true}, {"desired_policy", desired_policy_},
        {"desired_mode", desired_trajectory_ ? "TRAJECTORY" : "RL"}};
    }
    if (operation == "velocity" || operation == "forward") {
      if (!status.online || !status.owns_control || status.state != "RL" ||
        status.policy != walk_policy_)
      {
        return {{"ok", false},
          {"error", "walking policy/RL is not ready"}};
      }
      const double duration = request.value("duration_s", 10.0);
      if (!std::isfinite(duration) || duration < 0.1 || duration > 30.0) {
        return {{"ok", false},
          {"error", "duration_s must be in [0.1, 30]"}};
      }
      const bool accepted = SetTarget(
        request.value(
          "vx",
          operation == "forward" ? 0.2 : 0.0),
        operation == "velocity" ? request.value("vy", 0.0) : 0.0,
        operation == "velocity" ? request.value("wz", 0.0) : 0.0,
        duration);
      if (accepted) {
        voice_override_until_ = command_until_;
      }
      return {{"ok", accepted}};
    }
    if (operation == "wave" || operation == "interaction") {
      const std::string action = operation == "wave" ? "wave_hello" :
        request.value("action", std::string{});
      return ReplyJson(client_.StartInteraction(action));
    }
    if (operation == "cancel") {
      return ReplyJson(client_.CancelInteraction());
    }
    return {{"ok", false}, {"error", "unknown op"}};
  }

  operator_service::Client client_;
  std::unique_ptr<CommandSocket> command_socket_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  operator_service::Velocity target_;
  std::string desired_policy_;
  std::string walk_policy_;
  std::string stand_policy_;
  bool desired_trajectory_ = false;
  double cmd_vel_timeout_s_ = 0.5;
  double request_period_s_ = 0.5;
  double bias_x_ = 0.0;
  double bias_y_ = 0.0;
  double bias_yaw_ = 0.0;
  int velocity_ttl_ms_ = 300;
  Clock::time_point command_until_{};
  Clock::time_point voice_override_until_{};
  Clock::time_point last_renew_{};
  Clock::time_point last_request_{};
  Clock::time_point last_velocity_send_{};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto arguments = rclcpp::remove_ros_arguments(argc, argv);
  if (arguments.size() < 2 || arguments[1] == "-h" ||
    arguments[1] == "--help")
  {
    std::fprintf(
      arguments.size() < 2 ? stderr : stdout,
      "Usage: %s CONFIG.yaml [--ros-args ...]\n", argv[0]);
    rclcpp::shutdown();
    return arguments.size() < 2 ? 1 : 0;
  }
  try {
    rclcpp::spin(
      std::make_shared<HumanoidCmdVelClient>(
        humanoid_operator::ResolvePath(arguments[1])));
  } catch (const std::exception & error) {
    std::fprintf(stderr, "[humanoid_cmd_vel_hmi] %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
