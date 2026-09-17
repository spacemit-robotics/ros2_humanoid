/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file humanoid_cmd_vel_hmi_node.cpp
 * @brief Native humanoid HMI with ROS2 Twist velocity input.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

// Reuse the native HMI's terminal rendering, key decoding, FSM validation,
// policy selection and status acknowledgement logic verbatim. Only main is
// replaced so ROS callbacks can be serviced in the same thread as the UI.
#define main humanoid_native_hmi_main
#include HUMANOID_HMI_RUNTIME_SOURCE
#undef main

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
    if (args.size() < 2 || args[1] == "-h" || args[1] == "--help") {
        fprintf(args.size() < 2 ? stderr : stdout,
            "用法: %s <config.yaml> [--ros-args -p cmd_vel_topic:=/cmd_vel]\n"
            "选项:\n"
            "  <config.yaml>  机器人配置文件路径\n"
            "  -h, --help     显示此帮助信息\n", argv[0]);
        rclcpp::shutdown();
        return args.size() < 2 ? 1 : 0;
    }
    const std::string yaml_path = args[1];

    UiConfig config;
    std::unique_ptr<runtime_logging::Session> logging_session;
    try {
        config = LoadUiConfig(yaml_path);
        const auto yaml_file = robot_base::YamlFile::Load(yaml_path);
        logging_session = std::make_unique<runtime_logging::Session>(
            yaml_file, yaml_path, "ros_hmi", false);
    } catch (const std::exception &e) {
        fprintf(stderr, "%s\n用法: %s <config.yaml>\n", e.what(), argv[0]);
        rclcpp::shutdown();
        return 1;
    }

    auto transport = transport::CreateV2(yaml_path);
    if (!transport->Init(yaml_path, transport::Role::HMI)) {
        runtime_logging::Log(runtime_logging::Level::kError,
            "ROS HMI transport initialization failed", false);
        fprintf(stderr, "[humanoid_cmd_vel_hmi] 传输初始化失败\n");
        rclcpp::shutdown();
        return 1;
    }

    // This node is deliberately receive-only on the ROS graph. Disable the
    // implicit rosout/parameter publishers and parameter service endpoints.
    rclcpp::NodeOptions node_options;
    node_options.enable_rosout(false)
        .start_parameter_event_publisher(false)
        .start_parameter_services(false);
    auto node = std::make_shared<rclcpp::Node>(
        "humanoid_cmd_vel_hmi", node_options);
    const std::string cmd_vel_topic =
        node->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const double cmd_vel_timeout_s = std::max(0.1,
        node->declare_parameter<double>("cmd_vel_timeout_s", 0.5));
    const double cmd_vel_bias_x =
        node->declare_parameter<double>("cmd_vel_bias_x", 0.0);
    const double cmd_vel_bias_y =
        node->declare_parameter<double>("cmd_vel_bias_y", 0.0);
    const double cmd_vel_bias_yaw =
        node->declare_parameter<double>("cmd_vel_bias_yaw", 0.0);

    runtime_logging::Log(runtime_logging::Level::kInfo,
        "ROS cmd_vel HMI started: topic=" + cmd_vel_topic, false);

    g_color_enabled = isatty(STDOUT_FILENO) && std::getenv("NO_COLOR") == nullptr;
    Terminal terminal;

    UiState state;
    state.policies = config.policies;
    state.command_limits = config.command_limits;
    state.active_policy_idx = config.default_policy_idx;
    state.policy_cursor_idx = config.default_policy_idx;

    bool have_ros_command = false;
    bool ros_command_pending = false;
    bool ros_dirty = false;
    Clock::time_point last_ros_command_at{};
    auto cmd_vel_sub = node->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic, rclcpp::SystemDefaultsQoS(),
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            const auto *limits = ActiveCommandLimits(state);
            if (!state.status_online || state.status.mode != ControlMode::RL || !limits) {
                ZeroVelocity(&state);
                state.last_action = !state.status_online
                    ? "已收到 ROS cmd_vel，但 Control 状态未连接"
                    : "已收到 ROS cmd_vel，但当前状态/策略不接受速度";
            } else {
                const bool has_velocity = msg->linear.x != 0.0 ||
                    msg->linear.y != 0.0 || msg->angular.z != 0.0;
                state.target_command.vx = std::clamp(
                    static_cast<float>(msg->linear.x +
                        (has_velocity ? cmd_vel_bias_x : 0.0)),
                    -limits->max_vx, limits->max_vx);
                state.target_command.vy = std::clamp(
                    static_cast<float>(msg->linear.y +
                        (has_velocity ? cmd_vel_bias_y : 0.0)),
                    -limits->max_vy, limits->max_vy);
                state.target_command.wz = std::clamp(
                    static_cast<float>(msg->angular.z +
                        (has_velocity ? cmd_vel_bias_yaw : 0.0)),
                    -limits->max_wz, limits->max_wz);
                state.last_action = "ROS cmd_vel 已更新速度目标";
            }
            have_ros_command = true;
            last_ros_command_at = Clock::now();
            ros_command_pending = true;
            ros_dirty = true;
        });
    (void)cmd_vel_sub;

    const double heartbeat_period = 1.0 / config.hmi.heartbeat_hz;
    auto last_command_at = Clock::now() -
        std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(heartbeat_period));
    bool dirty = true;
    std::string last_logged_action;

    while (g_running && rclcpp::ok()) {
        rclcpp::spin_some(node);
        const auto now = Clock::now();
        bool send_immediately = ros_command_pending;
        ros_command_pending = false;
        dirty = dirty || ros_dirty;
        ros_dirty = false;

        if (have_ros_command &&
            std::chrono::duration<double>(now - last_ros_command_at).count() >
                cmd_vel_timeout_s) {
            have_ros_command = false;
            if (std::abs(state.target_command.vx) > 1e-6f ||
                std::abs(state.target_command.vy) > 1e-6f ||
                std::abs(state.target_command.wz) > 1e-6f) {
                ZeroVelocity(&state);
                state.last_action = "ROS cmd_vel 超时，速度已清零";
                send_immediately = true;
                dirty = true;
            }
        }

        robot_base::ControlStatus latest_status;
        robot_base::FaultStatus latest_fault;
        bool received_status = false;
        while (transport->RecvStatusV2(latest_status, latest_fault))
            received_status = true;
        if (received_status) {
            const bool status_changed =
                StatusChanged(state, latest_status, latest_fault);
            ProcessStatus(
                &state, latest_status, latest_fault, now, &send_immediately);
            dirty = dirty || status_changed || send_immediately;
        }

        const bool waiting_for_ack = state.transition.active ||
            !state.pending_policy.empty();
        const double effective_status_timeout = waiting_for_ack
            ? std::max(config.hmi.status_timeout, config.hmi.request_timeout)
            : config.hmi.status_timeout;
        const bool online = state.has_status &&
            std::chrono::duration<double>(now - state.last_status_at).count()
                <= effective_status_timeout;
        if (online != state.status_online) {
            state.status_online = online;
            dirty = true;
            if (!online) {
                state.transition.active = false;
                state.pending_policy.clear();
                state.fault_ack_sequence = 0;
                ZeroVelocity(&state);
                if (state.page == HmiPage::VELOCITY) state.page = HmiPage::MAIN;
                state.last_action = "Control 状态回传超时，速度已清零";
                send_immediately = true;
            } else {
                state.last_action = "Control 状态通道已连接";
            }
        }

        if (state.transition.active &&
            std::chrono::duration<double>(now - state.transition.requested_at).count()
                > config.hmi.request_timeout) {
            state.transition.active = false;
            state.last_action = "FSM 请求超时，已停止重发";
            send_immediately = true;
            dirty = true;
        }
        if (!state.pending_policy.empty() &&
            std::chrono::duration<double>(now - state.policy_requested_at).count()
                > config.hmi.request_timeout) {
            state.pending_policy.clear();
            state.last_action = "策略切换未获 Control 确认，已停止重发";
            send_immediately = true;
            dirty = true;
        }

        if (state.highlighted_key >= 0 && now >= state.highlight_until) {
            state.highlighted_key = -1;
            dirty = true;
        }

        const int key = ReadUiKey();
        if (key >= 0) {
            dirty = true;
            if (key == 'f') {
                send_immediately = RequestShortcut(&state, key, now) || send_immediately;
            } else if (state.page == HmiPage::POLICY_SELECT) {
                if ((key == kKeyUp || key == 'k') && !state.policies.empty()) {
                    state.policy_cursor_idx = (state.policy_cursor_idx - 1 +
                        static_cast<int>(state.policies.size())) %
                        static_cast<int>(state.policies.size());
                } else if ((key == kKeyDown || key == 'j') && !state.policies.empty()) {
                    state.policy_cursor_idx = (state.policy_cursor_idx + 1) %
                        static_cast<int>(state.policies.size());
                } else if ((key == '\r' || key == '\n') && !state.policies.empty()) {
                    if (!state.status_online) {
                        state.last_action = "策略未切换：Control 状态已断开";
                    } else if (state.status.mode != ControlMode::POWER_OFF &&
                        state.status.mode != ControlMode::DAMP) {
                        state.last_action = "策略未切换：请先进入 POWER_OFF 或 DAMP";
                    } else if (state.policy_cursor_idx == state.active_policy_idx) {
                        state.last_action = "所选策略已经生效";
                        state.page = HmiPage::MAIN;
                    } else {
                        state.pending_policy = state.policies[state.policy_cursor_idx];
                        state.policy_source = state.status.active_policy;
                        state.policy_requested_at = now;
                        state.last_action = "请求切换策略 → " +
                            state.pending_policy + "，等待 Control 确认";
                        state.page = HmiPage::MAIN;
                        send_immediately = true;
                    }
                } else if (key == kKeyEscape || key == 'p') {
                    state.policy_cursor_idx = state.active_policy_idx;
                    state.page = HmiPage::MAIN;
                    state.last_action = "取消策略选择";
                }
            } else if (state.page == HmiPage::VELOCITY) {
                const auto *limits = ActiveCommandLimits(state);
                if (key == kKeyEscape || key == 'v') {
                    ZeroVelocity(&state);
                    have_ros_command = false;
                    state.page = HmiPage::MAIN;
                    state.last_action = "退出速度控制，速度清零";
                    send_immediately = true;
                } else if (!limits) {
                    ZeroVelocity(&state);
                    state.page = HmiPage::MAIN;
                    state.last_action = "当前策略未配置速度命令范围";
                    send_immediately = true;
                } else if (key == 'w') {
                    state.target_command.vx = std::clamp(state.target_command.vx +
                        config.hmi.step_vx, -limits->max_vx, limits->max_vx);
                    state.last_action = "W：增加前进速度";
                    send_immediately = true;
                } else if (key == 's') {
                    state.target_command.vx = std::clamp(state.target_command.vx -
                        config.hmi.step_vx, -limits->max_vx, limits->max_vx);
                    state.last_action = "S：增加后退速度";
                    send_immediately = true;
                } else if (key == 'a') {
                    state.target_command.vy = std::clamp(state.target_command.vy +
                        config.hmi.step_vy, -limits->max_vy, limits->max_vy);
                    state.last_action = "A：增加左移速度";
                    send_immediately = true;
                } else if (key == 'd') {
                    state.target_command.vy = std::clamp(state.target_command.vy -
                        config.hmi.step_vy, -limits->max_vy, limits->max_vy);
                    state.last_action = "D：增加右移速度";
                    send_immediately = true;
                } else if (key == 'q') {
                    state.target_command.wz = std::clamp(state.target_command.wz +
                        config.hmi.step_wz, -limits->max_wz, limits->max_wz);
                    state.last_action = "Q：增加左转角速度";
                    send_immediately = true;
                } else if (key == 'e') {
                    state.target_command.wz = std::clamp(state.target_command.wz -
                        config.hmi.step_wz, -limits->max_wz, limits->max_wz);
                    state.last_action = "E：增加右转角速度";
                    send_immediately = true;
                } else if (key == ' ') {
                    ZeroVelocity(&state);
                    have_ros_command = false;
                    state.last_action = "SPACE：速度清零";
                    send_immediately = true;
                }
                if (send_immediately && (key == 'w' || key == 's' || key == 'a' ||
                    key == 'd' || key == 'q' || key == 'e' || key == ' ')) {
                    state.highlighted_key = key;
                    state.highlight_until = now +
                        std::chrono::milliseconds(config.hmi.key_highlight_ms);
                }
            } else {
                if (key == kKeyLeft) {
                    send_immediately = RequestByArrow(&state, false, now) || send_immediately;
                } else if (key == kKeyRight) {
                    send_immediately = RequestByArrow(&state, true, now) || send_immediately;
                } else if (key == 'p') {
                    if (state.policies.empty()) {
                        state.last_action = "没有配置可选策略";
                    } else {
                        state.policy_cursor_idx = state.active_policy_idx;
                        state.page = HmiPage::POLICY_SELECT;
                        state.last_action = "选择策略";
                    }
                } else if (key == 'v' || key == '\r' || key == '\n') {
                    if (state.status_online && state.status.hmi_connected &&
                        state.status.mode == ControlMode::RL && ActiveCommandLimits(state)) {
                        state.page = HmiPage::VELOCITY;
                        state.last_action = "进入键盘/ROS 速度控制";
                    } else if (state.status_online && state.status.mode == ControlMode::RL) {
                        state.last_action =
                            "当前策略未配置 command.limits，不接受速度命令";
                    } else {
                        state.last_action = "速度页仅在真实 FSM=RL 且心跳正常时开放";
                    }
                } else if (key == 'o' || key == 'h' || key == 'z' || key == 'r') {
                    send_immediately = RequestShortcut(&state, key, now) || send_immediately;
                } else if (key == ' ') {
                    ZeroVelocity(&state);
                    have_ros_command = false;
                    state.last_action = "速度清零";
                    send_immediately = true;
                } else if (key == 'w' || key == 's' || key == 'a' ||
                    key == 'd' || key == 'q' || key == 'e') {
                    state.last_action = "请按 V 或 Enter 进入速度控制页";
                }
            }
        }

        if (!state.last_action.empty() && state.last_action != last_logged_action) {
            runtime_logging::Log(runtime_logging::Level::kInfo,
                state.last_action, false);
            last_logged_action = state.last_action;
        }

        if (send_immediately ||
            std::chrono::duration<double>(now - last_command_at).count() >=
                heartbeat_period) {
            if (SendCommand(transport.get(), &state)) {
                dirty = true;
                last_logged_action = state.last_action;
            }
            last_command_at = now;
        }

        if (dirty) {
            Render(state);
            dirty = false;
        }
        usleep(5000);
    }

    state.transition.active = true;
    state.transition.key = -1;
    state.pending_policy.clear();
    state.fault_ack_sequence = 0;
    ZeroVelocity(&state);
    (void)SendCommand(transport.get(), &state);
    runtime_logging::Log(
        state.command_send_failed ? runtime_logging::Level::kWarning
            : runtime_logging::Level::kInfo,
        state.command_send_failed
            ? "ROS HMI stopped; POWER_OFF request send failed, control timeout is fallback"
            : "ROS HMI stopped after requesting POWER_OFF",
        false);
    rclcpp::shutdown();
    return 0;
}
