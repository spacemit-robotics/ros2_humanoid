/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @brief Forward ROS velocities through the same operator client as hmi_tui.
 */
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

namespace {
using Clock = std::chrono::steady_clock;

bool Ready(const operator_service::Status& status) {
    return status.online && status.hmi_connected && status.state == "RL" && !status.trajectory_enabled &&
        !status.fault.active && !status.fault.latched;
}

class G1CmdVelBridge : public rclcpp::Node {
public:
    G1CmdVelBridge() : Node("g1_cmd_vel_bridge") {
        const auto topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        timeout_s_ = declare_parameter<double>("cmd_vel_timeout_s", 0.2);
        release_timeout_s_ = declare_parameter<double>("control_release_timeout_s", 5.0);
        const auto rate = declare_parameter<double>("publish_rate_hz", 10.0);
        const auto ttl = declare_parameter<int64_t>("velocity_ttl_ms", 300);
        const auto socket_timeout = declare_parameter<int64_t>("socket_timeout_ms", 300);
        if (!std::isfinite(timeout_s_) || timeout_s_ <= 0.0 || !std::isfinite(rate) || rate < 1.0 || rate > 50.0 ||
            ttl < 50 || ttl > 1000 || 1000.0 / rate >= ttl || socket_timeout < 100 || socket_timeout > 10000) {
            throw std::invalid_argument("require finite cmd_vel_timeout_s > 0, publish_rate_hz in [1, 50], "
                                        "velocity_ttl_ms in [50, 1000] and greater than the send period, "
                                        "socket_timeout_ms in [100, 10000]");
        }
        if (!std::isfinite(release_timeout_s_) || release_timeout_s_ <= timeout_s_) {
            throw std::invalid_argument("control_release_timeout_s must be finite and greater than cmd_vel_timeout_s");
        }
        velocity_ttl_ms_ = static_cast<int>(ttl);
        socket_timeout_ms_ = static_cast<int>(socket_timeout);
        const auto config = humanoid_operator::ResolvePath(declare_parameter<std::string>("robot_config_path", ""));
        const auto connection = declare_parameter<std::string>("connection_file", "");
        if (!connection.empty()) {
            connection_file_ = humanoid_operator::ResolvePath(connection);
        } else if (!config.empty()) {
            connection_file_ = humanoid_operator::ConnectionFile(config, "");
        } else {
            connection_file_ = humanoid_operator::DefaultConnectionFile("g1");
        }

        // A best-effort reader accepts reliable and best-effort ROS publishers.
        // Keep only the latest volatile command, never a retained velocity.
        subscription_ =
            create_subscription<geometry_msgs::msg::Twist>(topic, rclcpp::QoS(1).best_effort().durability_volatile(),
                [this](geometry_msgs::msg::Twist::ConstSharedPtr message) { Receive(*message); });
        status_pub_ = create_publisher<std_msgs::msg::String>("~/status", 1);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate)),
            [this]() { Tick(); });
        RCLCPP_INFO(get_logger(), "cmd_vel=%s; operator connection=%s; waiting for RL and fresh commands",
            topic.c_str(), connection_file_.c_str());
    }

    ~G1CmdVelBridge() override { Idle(client_.LatestStatus()); }

private:
    void Receive(const geometry_msgs::msg::Twist& message) {
        const auto status = client_.LatestStatus();
        if (!Ready(status) || (!status.owns_control && !status.owner.empty()) || !std::isfinite(message.linear.x) ||
            !std::isfinite(message.linear.y) || !std::isfinite(message.angular.z)) {
            have_command_ = false;
            return;
        }
        target_ = {message.linear.x, message.linear.y, message.angular.z};
        last_command_ = Clock::now();
        have_command_ = true;
    }

    bool Fresh() const {
        return have_command_ && std::chrono::duration<double>(Clock::now() - last_command_).count() <= timeout_s_;
    }

    bool WithinReleaseTimeout() const {
        return have_command_ &&
            std::chrono::duration<double>(Clock::now() - last_command_).count() <= release_timeout_s_;
    }

    void Idle(const operator_service::Status& status) {
        have_command_ = false;
        target_ = {};
        if (client_.Connected() && (lease_acquired_ || status.owns_control)) {
            if (Ready(status) && status.owns_control) {
                (void)client_.SetVelocity({}, velocity_ttl_ms_);
            }
            // Release only this client's lease; never use Stop(), which affects other clients.
            (void)client_.ReleaseControl();
        }
        lease_acquired_ = false;
    }

    void Tick() {
        auto status = client_.LatestStatus();
        std_msgs::msg::String report;
        report.data = "state=" + status.state + " online=" + (status.online ? "true" : "false") +
            " owner=" + status.owner +
            " forwarding=" + (Ready(status) && status.owns_control && Fresh() ? "true" : "false");
        status_pub_->publish(report);

        if (!client_.Connected()) {
            Idle(status);
            const auto now = Clock::now();
            if (now < next_connect_) {
                return;
            }
            next_connect_ = now + std::chrono::seconds(1);
            try {
                humanoid_operator::Connect(&client_, connection_file_, "g1_ros_cmd_vel", socket_timeout_ms_);
                RCLCPP_INFO(get_logger(), "Connected to G1 operator service; waiting for a new cmd_vel");
            } catch (const std::exception& error) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
            }
            return;
        }
        if (!Ready(status) || !WithinReleaseTimeout()) {
            Idle(status);
            return;
        }
        const auto now = Clock::now();
        if (!status.owns_control) {
            if (!Fresh() && !lease_acquired_) {
                // Stale commands may retain our existing lease, but must never acquire one.
                Idle(status);
            } else if (!status.owner.empty()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "cmd_vel ignored: control belongs to %s; release it in the owning client", status.owner.c_str());
                Idle(status);
            } else if (lease_acquired_) {
                // Acquire replies can arrive before the next asynchronous status event.
                if (now - last_renew_ >= std::chrono::milliseconds(250)) {
                    Idle(status);
                }
            } else {
                const auto reply = client_.AcquireControl();
                // Even a timed-out request may have acquired a lease. Release it on failure.
                lease_acquired_ = true;
                last_renew_ = Clock::now();
                if (!reply.ok) {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 2000, "control acquisition rejected: %s", reply.message.c_str());
                    Idle(client_.LatestStatus());
                }
            }
            return;
        }
        lease_acquired_ = true;
        if (now - last_renew_ >= std::chrono::milliseconds(250)) {
            const auto reply = client_.RenewControl();
            last_renew_ = Clock::now();
            if (!reply.ok) {
                Idle(client_.LatestStatus());
                return;
            }
        }
        // Recheck after synchronous RPCs, so a timeout or state change cannot replay a command.
        status = client_.LatestStatus();
        if (!Ready(status) || !status.owns_control || !WithinReleaseTimeout()) {
            Idle(status);
            return;
        }
        // Keep renewing the lease and sending zero between the two timeouts.
        const auto command = Fresh() ? target_ : operator_service::Velocity{};
        const auto reply = client_.SetVelocity(command, velocity_ttl_ms_);
        if (!reply.ok) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "velocity rejected: %s", reply.message.c_str());
            Idle(client_.LatestStatus());
        }
    }

    operator_service::Client client_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string connection_file_;
    operator_service::Velocity target_;
    bool have_command_ = false;
    bool lease_acquired_ = false;
    double timeout_s_ = 0.2;
    double release_timeout_s_ = 5.0;
    int velocity_ttl_ms_ = 300;
    int socket_timeout_ms_ = 300;
    Clock::time_point last_command_{};
    Clock::time_point last_renew_{};
    Clock::time_point next_connect_{};
};
} // namespace

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        rclcpp::spin(std::make_shared<G1CmdVelBridge>());
    } catch (const std::exception& error) {
        std::fprintf(stderr, "[g1_cmd_vel_bridge] %s\n", error.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
