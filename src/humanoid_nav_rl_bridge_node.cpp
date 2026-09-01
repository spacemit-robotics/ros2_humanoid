/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file humanoid_nav_rl_bridge_node.cpp
 * @brief Bridge ROS2 /cmd_vel to humanoid RL command transport.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_base.h"
#include "std_msgs/msg/string.hpp"
#include "transport_executor.h"

namespace
{

using Clock = std::chrono::steady_clock;

const char* ModeName(robot_base::ControlMode mode)
{
    switch (mode)
    {
        case robot_base::ControlMode::POWER_OFF:
            return "POWER_OFF";
        case robot_base::ControlMode::DAMP:
            return "DAMP";
        case robot_base::ControlMode::ZERO:
            return "ZERO";
        case robot_base::ControlMode::RL:
            return "RL";
        case robot_base::ControlMode::SAFETY:
            return "SAFETY";
        case robot_base::ControlMode::HOME:
            return "HOME";
    }
    return "UNKNOWN";
}

float Clamp(float value, float limit)
{
    return std::clamp(value, -std::abs(limit), std::abs(limit));
}

float Slew(float current, float target, float max_delta)
{
    return current + std::clamp(target - current, -max_delta, max_delta);
}

std::string ResolvePath(const std::string& path)
{
    if (path.empty() || path.front() == '/')
    {
        return path;
    }
    const char* sdk_root = std::getenv("SDK_ROOT");
    if (sdk_root && sdk_root[0] != '\0')
    {
        return std::string(sdk_root) + "/" + path;
    }
    return std::string(std::getenv("PWD") ? std::getenv("PWD") : ".") + "/" + path;
}

}  // namespace

class HumanoidNavRlBridge : public rclcpp::Node
{
public:
    HumanoidNavRlBridge() : Node("humanoid_nav_rl_bridge")
    {
        robot_config_path_ = ResolvePath(declare_parameter<std::string>("robot_config_path", ""));
        if (robot_config_path_.empty())
        {
            throw std::invalid_argument("robot_config_path must not be empty");
        }
        cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        accepted_cmd_vel_timeout_s_ = declare_parameter<double>("accepted_cmd_vel_timeout_s", 0.5);
        publish_rate_hz_ = std::max(1.0, declare_parameter<double>("publish_rate_hz", 20.0));

        auto_fsm_ = declare_parameter<bool>("auto_fsm", false);
        auto_policy_ = declare_parameter<std::string>("auto_policy", "walk");
        auto_request_period_s_ = declare_parameter<double>("auto_request_period_s", 0.5);
        require_zero_ready_ = declare_parameter<bool>("require_zero_ready", true);
        send_poweroff_on_shutdown_ = declare_parameter<bool>("send_poweroff_on_shutdown", false);

        max_vx_ = static_cast<float>(declare_parameter<double>("max_vx", 0.3));
        max_vy_ = static_cast<float>(declare_parameter<double>("max_vy", 0.25));
        max_wz_ = static_cast<float>(declare_parameter<double>("max_wz", 0.5));
        max_accel_vx_ = static_cast<float>(declare_parameter<double>("max_accel_vx", 0.4));
        max_accel_vy_ = static_cast<float>(declare_parameter<double>("max_accel_vy", 0.3));
        max_accel_wz_ = static_cast<float>(declare_parameter<double>("max_accel_wz", 0.8));

        transport_ = transport::Create(robot_config_path_);
        if (!transport_->Init(robot_config_path_, transport::Role::HMI))
        {
            throw std::runtime_error("humanoid HMI transport init failed");
        }

        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(cmd_vel_topic_,
            rclcpp::SystemDefaultsQoS(),
            [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            {
                target_vx_ = Clamp(static_cast<float>(msg->linear.x), max_vx_);
                target_vy_ = Clamp(static_cast<float>(msg->linear.y), max_vy_);
                target_wz_ = Clamp(static_cast<float>(msg->angular.z), max_wz_);
                last_cmd_vel_time_ = Clock::now();
                have_cmd_vel_ = true;
            });
        status_pub_ = create_publisher<std_msgs::msg::String>("~/status", 10);

        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / publish_rate_hz_)),
            [this]() { Tick(); });

        RCLCPP_INFO(get_logger(),
            "Humanoid Nav RL bridge started: cmd_vel=%s config=%s auto_fsm=%s policy=%s",
            cmd_vel_topic_.c_str(),
            robot_config_path_.c_str(),
            auto_fsm_ ? "true" : "false",
            auto_policy_.c_str());
    }

    ~HumanoidNavRlBridge() override
    {
        if (transport_ && send_poweroff_on_shutdown_)
        {
            robot_base::Command cmd;
            cmd.key = -1;
            transport_->SendCommand(cmd);
        }
        else if (transport_)
        {
            robot_base::Command cmd;
            transport_->SendCommand(cmd);
        }
    }

private:
    void Tick()
    {
        DrainStatus();
        const auto now = Clock::now();
        const double dt = last_tick_time_.time_since_epoch().count() == 0
            ? 1.0 / publish_rate_hz_
            : std::chrono::duration<double>(now - last_tick_time_).count();
        last_tick_time_ = now;

        robot_base::Command cmd;
        cmd.key = auto_fsm_ ? NextAutoKey(now) : 0;
        cmd.switch_policy = NextAutoPolicy(now);

        const bool cmd_vel_fresh = have_cmd_vel_ &&
            std::chrono::duration<double>(now - last_cmd_vel_time_).count() <=
                accepted_cmd_vel_timeout_s_;
        const bool can_send_velocity = status_online_ &&
            latest_status_.mode == robot_base::ControlMode::RL &&
            (auto_policy_.empty() || latest_status_.active_policy == auto_policy_);

        const float target_vx = (cmd_vel_fresh && can_send_velocity) ? target_vx_ : 0.0f;
        const float target_vy = (cmd_vel_fresh && can_send_velocity) ? target_vy_ : 0.0f;
        const float target_wz = (cmd_vel_fresh && can_send_velocity) ? target_wz_ : 0.0f;
        cmd.vx = Slew(current_vx_, target_vx, max_accel_vx_ * static_cast<float>(dt));
        cmd.vy = Slew(current_vy_, target_vy, max_accel_vy_ * static_cast<float>(dt));
        cmd.wz = Slew(current_wz_, target_wz, max_accel_wz_ * static_cast<float>(dt));
        current_vx_ = cmd.vx;
        current_vy_ = cmd.vy;
        current_wz_ = cmd.wz;

        transport_->SendCommand(cmd);
        PublishStatus(cmd, cmd_vel_fresh, can_send_velocity);
    }

    void DrainStatus()
    {
        robot_base::ControlStatus status;
        bool received = false;
        while (transport_->RecvStatus(status))
        {
            latest_status_ = status;
            received = true;
        }
        if (received)
        {
            last_status_time_ = Clock::now();
            status_online_ = true;
        }
        else if (status_online_ &&
            std::chrono::duration<double>(Clock::now() - last_status_time_).count() > 1.0)
        {
            status_online_ = false;
        }
    }

    int NextAutoKey(const Clock::time_point& now)
    {
        if (!status_online_)
        {
            return 0;
        }
        if (now - last_auto_request_time_ <
            std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(auto_request_period_s_)))
        {
            return 0;
        }
        if (!latest_status_.active_policy.empty() && !auto_policy_.empty() &&
            latest_status_.active_policy != auto_policy_)
        {
            return 0;
        }

        switch (latest_status_.mode)
        {
            case robot_base::ControlMode::POWER_OFF:
                last_auto_request_time_ = now;
                return 1;
            case robot_base::ControlMode::DAMP:
                last_auto_request_time_ = now;
                return 4;
            case robot_base::ControlMode::HOME:
                last_auto_request_time_ = now;
                return 2;
            case robot_base::ControlMode::ZERO:
                if (!require_zero_ready_ || latest_status_.zero_ready)
                {
                    last_auto_request_time_ = now;
                    return 3;
                }
                return 0;
            case robot_base::ControlMode::RL:
            case robot_base::ControlMode::SAFETY:
                return 0;
        }
        return 0;
    }

    std::string NextAutoPolicy(const Clock::time_point& now)
    {
        if (!auto_fsm_ || auto_policy_.empty() || !status_online_)
        {
            return {};
        }
        if (latest_status_.active_policy == auto_policy_)
        {
            return {};
        }
        if (latest_status_.mode != robot_base::ControlMode::POWER_OFF &&
            latest_status_.mode != robot_base::ControlMode::DAMP)
        {
            return {};
        }
        if (now - last_policy_request_time_ <
            std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(auto_request_period_s_)))
        {
            return {};
        }
        last_policy_request_time_ = now;
        return auto_policy_;
    }

    void PublishStatus(const robot_base::Command& cmd, bool cmd_vel_fresh, bool can_send_velocity)
    {
        std_msgs::msg::String msg;
        msg.data = std::string("control=") + (status_online_ ? "online" : "offline") +
            " mode=" + ModeName(latest_status_.mode) + " policy=" + latest_status_.active_policy +
            " cmd_vel=" + (cmd_vel_fresh ? "fresh" : "stale") +
            " velocity_gate=" + (can_send_velocity ? "open" : "closed") + " sent=(" +
            std::to_string(cmd.vx) + "," + std::to_string(cmd.vy) + "," + std::to_string(cmd.wz) +
            ")" + " key=" + std::to_string(cmd.key) + " switch_policy=" + cmd.switch_policy;
        status_pub_->publish(msg);
    }

    std::string robot_config_path_;
    std::string cmd_vel_topic_;
    double accepted_cmd_vel_timeout_s_ = 0.5;
    double publish_rate_hz_ = 20.0;
    bool auto_fsm_ = false;
    std::string auto_policy_;
    double auto_request_period_s_ = 0.5;
    bool require_zero_ready_ = true;
    bool send_poweroff_on_shutdown_ = false;
    float max_vx_ = 0.3f;
    float max_vy_ = 0.25f;
    float max_wz_ = 0.5f;
    float max_accel_vx_ = 0.4f;
    float max_accel_vy_ = 0.3f;
    float max_accel_wz_ = 0.8f;

    std::unique_ptr<transport::TransportBase> transport_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    robot_base::ControlStatus latest_status_;
    bool status_online_ = false;
    Clock::time_point last_status_time_{};
    Clock::time_point last_auto_request_time_{};
    Clock::time_point last_policy_request_time_{};
    Clock::time_point last_tick_time_{};

    bool have_cmd_vel_ = false;
    Clock::time_point last_cmd_vel_time_{};
    float target_vx_ = 0.0f;
    float target_vy_ = 0.0f;
    float target_wz_ = 0.0f;
    float current_vx_ = 0.0f;
    float current_vy_ = 0.0f;
    float current_wz_ = 0.0f;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<HumanoidNavRlBridge>());
    }
    catch (const std::exception& e)
    {
        fprintf(stderr, "[humanoid_nav_rl_bridge] %s\n", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
