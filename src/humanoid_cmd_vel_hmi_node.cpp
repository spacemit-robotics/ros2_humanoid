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
#include <cerrno>

#include <nlohmann/json.hpp>
#include <zmq.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

// Reuse the native HMI's terminal rendering, key decoding, FSM validation,
// policy selection and status acknowledgement logic verbatim. Only main is
// replaced so ROS callbacks can be serviced in the same thread as the UI.
#define main humanoid_native_hmi_main
#include HUMANOID_HMI_RUNTIME_SOURCE
#undef main

namespace {

class CommandSocket {
public:
    explicit CommandSocket(const std::string &endpoint) {
        context_ = zmq_ctx_new();
        if (!context_) throw std::runtime_error(zmq_strerror(errno));
        socket_ = zmq_socket(context_, ZMQ_REP);
        if (socket_) {
            const int linger = 0;
            (void)zmq_setsockopt(socket_, ZMQ_LINGER, &linger,
                sizeof(linger));
        }
        if (!socket_ || zmq_bind(socket_, endpoint.c_str()) != 0) {
            const std::string error = zmq_strerror(errno);
            if (socket_) zmq_close(socket_);
            zmq_ctx_term(context_);
            socket_ = nullptr;
            context_ = nullptr;
            throw std::runtime_error("ZMQ bind " + endpoint + ": " + error);
        }
    }

    ~CommandSocket() {
        if (socket_) zmq_close(socket_);
        if (context_) zmq_ctx_term(context_);
    }

    template <typename Handler>
    void Poll(Handler &&handler) {
        char buffer[4096];
        const int size = zmq_recv(socket_, buffer, sizeof(buffer), ZMQ_DONTWAIT);
        if (size < 0) return;
        nlohmann::json reply;
        if (size >= static_cast<int>(sizeof(buffer))) {
            reply = {{"ok", false}, {"error", "request too large"}};
        } else {
            try {
                reply = handler(nlohmann::json::parse(buffer, buffer + size));
            } catch (const std::exception &error) {
                reply = {{"ok", false}, {"error", error.what()}};
            }
        }
        const std::string encoded = reply.dump();
        (void)zmq_send(socket_, encoded.data(), encoded.size(), 0);
    }

private:
    void *context_ = nullptr;
    void *socket_ = nullptr;
};

}  // namespace

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
    const double zero_velocity_walk_hold_s =
        node->declare_parameter<double>("zero_velocity_walk_hold_s", 0.0);
    if (!std::isfinite(zero_velocity_walk_hold_s) ||
        zero_velocity_walk_hold_s < 0.0) {
        fprintf(stderr, "[humanoid_cmd_vel_hmi] zero_velocity_walk_hold_s "
            "必须为非负有限数\n");
        rclcpp::shutdown();
        return 1;
    }
    const double cmd_vel_bias_x =
        node->declare_parameter<double>("cmd_vel_bias_x", 0.0);
    const double cmd_vel_bias_y =
        node->declare_parameter<double>("cmd_vel_bias_y", 0.0);
    const double cmd_vel_bias_yaw =
        node->declare_parameter<double>("cmd_vel_bias_yaw", 0.0);
    const std::string zmq_endpoint = node->declare_parameter<std::string>(
        "zmq_endpoint", "tcp://127.0.0.1:5565");
    std::unique_ptr<CommandSocket> command_socket;
    try {
        command_socket = std::make_unique<CommandSocket>(zmq_endpoint);
    } catch (const std::exception &error) {
        fprintf(stderr, "[humanoid_cmd_vel_hmi] %s\n", error.what());
        rclcpp::shutdown();
        return 1;
    }

    runtime_logging::Log(runtime_logging::Level::kInfo,
        "ROS cmd_vel HMI started: topic=" + cmd_vel_topic, false);

    g_color_enabled = isatty(STDOUT_FILENO) && std::getenv("NO_COLOR") == nullptr;
    Terminal terminal;

    UiState state;
    state.policies = config.policies;
    state.manual_reference_policies = config.manual_reference_policies;
    state.interaction_actions = config.interaction_actions;
    state.command_limits = config.command_limits;
    state.active_policy_idx = config.default_policy_idx;
    state.policy_cursor_idx = config.default_policy_idx;

    const bool automatic_policy_switch =
        std::find(config.policies.begin(), config.policies.end(),
            "stand_mjlab") != config.policies.end() &&
        std::find(config.policies.begin(), config.policies.end(),
            "walk_mjlab") != config.policies.end();
    enum class SwitchStage { IDLE, TO_DAMP, SWITCH, TO_HOME, TO_ZERO, TO_RL };
    SwitchStage switch_stage = SwitchStage::IDLE;
    std::string desired_policy;
    std::string blocked_policy;
    std::string switch_target;
    Clock::time_point switch_started_at{};
    Clock::time_point zero_velocity_since{};
    bool zero_velocity_hold_armed = false;
    bool zero_velocity_hold_pending = false;
    std::string requested_interaction;
    double requested_vx = 0.0;
    double requested_vy = 0.0;
    double requested_wz = 0.0;

    bool have_ros_command = false;
    bool ros_command_pending = false;
    bool ros_dirty = false;
    Clock::time_point last_ros_command_at{};
    Clock::time_point voice_override_until{};
    double active_command_timeout_s = cmd_vel_timeout_s;
    const auto set_desired_policy = [&](const std::string &next_policy) {
        if (next_policy != desired_policy) blocked_policy.clear();
        desired_policy = next_policy;
    };
    const auto start_zero_velocity_hold = [&](const Clock::time_point &now) {
        zero_velocity_hold_armed = true;
        if (automatic_policy_switch && zero_velocity_walk_hold_s > 0.0 &&
            state.status_online && state.status.mode == ControlMode::RL &&
            state.status.active_policy == "walk_mjlab" &&
            switch_stage == SwitchStage::IDLE) {
            zero_velocity_hold_pending = true;
            zero_velocity_since = now;
        }
    };
    const auto apply_velocity = [&](double vx, double vy, double wz,
            const char *source, double timeout_s,
            bool allow_walk_hold = true) {
        if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) {
            return false;
        }
        requested_vx = vx;
        requested_vy = vy;
        requested_wz = wz;
        const bool moving = vx != 0.0 || vy != 0.0 || wz != 0.0;
        if (automatic_policy_switch) {
            if (moving) {
                zero_velocity_hold_armed = false;
                zero_velocity_hold_pending = false;
                set_desired_policy("walk_mjlab");
            } else {
                if (!allow_walk_hold) {
                    zero_velocity_hold_armed = true;
                    zero_velocity_hold_pending = false;
                } else if (!zero_velocity_hold_armed) {
                    start_zero_velocity_hold(Clock::now());
                }
                set_desired_policy(zero_velocity_hold_pending ?
                    "walk_mjlab" : "stand_mjlab");
            }
        }
        const auto *limits = ActiveCommandLimits(state);
        if (!state.status_online || state.status.mode != ControlMode::RL ||
            !limits || switch_stage != SwitchStage::IDLE ||
            (automatic_policy_switch &&
                state.status.active_policy != desired_policy)) {
            ZeroVelocity(&state);
            state.last_action = std::string(source) +
                " 速度已接收，等待 Control 状态/策略就绪";
        } else {
            state.target_command.vx = std::clamp(static_cast<float>(vx +
                (moving ? cmd_vel_bias_x : 0.0)),
                limits->min_vx, limits->max_vx);
            state.target_command.vy = std::clamp(static_cast<float>(vy +
                (moving ? cmd_vel_bias_y : 0.0)),
                limits->min_vy, limits->max_vy);
            state.target_command.wz = std::clamp(static_cast<float>(wz +
                (moving ? cmd_vel_bias_yaw : 0.0)),
                limits->min_wz, limits->max_wz);
            state.last_action = std::string(source) + " 已更新速度目标";
        }
        have_ros_command = true;
        last_ros_command_at = Clock::now();
        active_command_timeout_s = timeout_s;
        ros_command_pending = true;
        ros_dirty = true;
        return true;
    };
    auto cmd_vel_sub = node->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic, rclcpp::SystemDefaultsQoS(),
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            if (Clock::now() < voice_override_until) return;
            (void)apply_velocity(msg->linear.x, msg->linear.y,
                msg->angular.z, "ROS cmd_vel", cmd_vel_timeout_s);
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
        command_socket->Poll([&](const nlohmann::json &request) {
            if (!request.is_object()) {
                return nlohmann::json{{"ok", false},
                    {"error", "request must be a JSON object"}};
            }
            const std::string op = request.value("op", std::string{});
            if (op == "status") {
                return nlohmann::json{{"ok", true},
                    {"online", state.status_online},
                    {"mode", ModeName(state.status.mode)},
                    {"policy", state.status.active_policy},
                    {"desired_policy", desired_policy},
                    {"switching", switch_stage != SwitchStage::IDLE},
                    {"interaction_phase", InteractionPhaseName(
                        state.status.interaction.phase)},
                    {"fault", state.fault.latched}};
            }
            if (op == "velocity" || op == "forward" || op == "walk" ||
                op == "stand" || op == "stop") {
                if (op != "stand" && op != "stop" &&
                    (!state.status_online || state.fault.latched ||
                    state.status.mode == ControlMode::POWER_OFF ||
                    state.status.mode == ControlMode::SAFETY)) {
                    return nlohmann::json{{"ok", false},
                        {"error", "Control state does not allow motion"}};
                }
                const double vx = op == "forward" || op == "walk" ?
                    request.value("vx", 0.2) :
                    (op == "velocity" ? request.value("vx", 0.0) : 0.0);
                const double vy = op == "velocity" ?
                    request.value("vy", 0.0) : 0.0;
                const double wz = op == "velocity" ?
                    request.value("wz", 0.0) : 0.0;
                const double duration_s = request.value("duration_s", 10.0);
                if (!std::isfinite(duration_s) || duration_s < 0.1 ||
                    duration_s > 30.0) {
                    return nlohmann::json{{"ok", false},
                        {"error", "duration_s must be in [0.1, 30]"}};
                }
                if (op == "stand" || op == "stop") {
                    requested_interaction.clear();
                    if (InteractionIsBusy(state.status.interaction.phase)) {
                        ros_command_pending = RequestInteractionCancel(
                            &state, Clock::now()) || ros_command_pending;
                    }
                }
                const bool accepted = apply_velocity(
                    vx, vy, wz, "ZMQ", duration_s,
                    op != "stand" && op != "stop");
                if (accepted) {
                    voice_override_until = Clock::now() +
                        std::chrono::duration_cast<Clock::duration>(
                            std::chrono::duration<double>(duration_s));
                }
                return nlohmann::json{{"ok", accepted},
                    {"desired_policy", desired_policy}};
            }
            if (op == "wave" || op == "interaction") {
                const std::string action_key = op == "wave" ? "wave_hello" :
                    request.value("action", std::string{});
                const auto actions = config.interaction_actions.find(
                    "stand_mjlab");
                if (action_key.empty() ||
                    actions == config.interaction_actions.end() ||
                    std::none_of(actions->second.begin(), actions->second.end(),
                        [&](const InteractionAction &action) {
                            return action.key == action_key;
                        })) {
                    return nlohmann::json{{"ok", false},
                        {"error", "action is not available on stand_mjlab"}};
                }
                if (!state.status_online || state.fault.latched ||
                    state.status.mode == ControlMode::POWER_OFF ||
                    state.status.mode == ControlMode::SAFETY) {
                    return nlohmann::json{{"ok", false},
                        {"error", "Control is offline or faulted"}};
                }
                requested_interaction = action_key;
                (void)apply_velocity(0.0, 0.0, 0.0, "ZMQ", 10.0, false);
                voice_override_until = Clock::now() +
                    std::chrono::seconds(10);
                return nlohmann::json{{"ok", true},
                    {"action", requested_interaction}, {"queued", true}};
            }
            if (op == "cancel") {
                requested_interaction.clear();
                const bool accepted = RequestInteractionCancel(
                    &state, Clock::now());
                ros_command_pending = ros_command_pending || accepted;
                ros_dirty = true;
                return nlohmann::json{{"ok", accepted},
                    {"message", state.last_action}};
            }
            return nlohmann::json{{"ok", false},
                {"error", "unknown op"}};
        });
        const auto now = Clock::now();
        bool send_immediately = ros_command_pending;
        ros_command_pending = false;
        dirty = dirty || ros_dirty;
        ros_dirty = false;

        if (have_ros_command &&
            std::chrono::duration<double>(now - last_ros_command_at).count() >
                active_command_timeout_s) {
            have_ros_command = false;
            requested_vx = requested_vy = requested_wz = 0.0;
            if (automatic_policy_switch) {
                if (!zero_velocity_hold_armed) start_zero_velocity_hold(now);
                set_desired_policy(zero_velocity_hold_pending ?
                    "walk_mjlab" : "stand_mjlab");
            }
            if (std::abs(state.target_command.vx) > 1e-6f ||
                std::abs(state.target_command.vy) > 1e-6f ||
                std::abs(state.target_command.wz) > 1e-6f) {
                ZeroVelocity(&state);
                state.last_action = "速度命令超时，速度已清零";
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

        if (zero_velocity_hold_pending &&
            (state.status.active_policy != "walk_mjlab" ||
                std::chrono::duration<double>(now - zero_velocity_since).count()
                    >= zero_velocity_walk_hold_s)) {
            zero_velocity_hold_pending = false;
            set_desired_policy("stand_mjlab");
            state.last_action = "零速度驻留结束，请求站立策略";
            send_immediately = true;
            dirty = true;
        }

        const bool waiting_for_ack = state.transition.active ||
            !state.pending_policy.empty() ||
            state.target_command.interaction.operation !=
                robot_base::InteractionRequest::Operation::NONE;
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
                if (switch_stage != SwitchStage::IDLE) {
                    blocked_policy = desired_policy;
                    switch_stage = SwitchStage::IDLE;
                }
                state.transition.active = false;
                state.pending_policy.clear();
                state.fault_ack_sequence = 0;
                state.reference_start_pulse = false;
                state.reference_start_requested = false;
                state.target_command.interaction.operation =
                    robot_base::InteractionRequest::Operation::NONE;
                state.target_command.interaction.action.clear();
                requested_interaction.clear();
                ZeroVelocity(&state);
                have_ros_command = false;
                if (state.page == HmiPage::VELOCITY ||
                    state.page == HmiPage::INTERACTION_SELECT) {
                    state.page = HmiPage::MAIN;
                }
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
            if (switch_stage != SwitchStage::IDLE) {
                blocked_policy = desired_policy;
                switch_stage = SwitchStage::IDLE;
            }
            state.last_action = "FSM 请求超时，已停止重发";
            send_immediately = true;
            dirty = true;
        }
        if (!state.pending_policy.empty() &&
            std::chrono::duration<double>(now - state.policy_requested_at).count()
                > config.hmi.request_timeout) {
            state.pending_policy.clear();
            if (switch_stage != SwitchStage::IDLE) {
                blocked_policy = desired_policy;
                switch_stage = SwitchStage::IDLE;
            }
            state.last_action = "策略切换未获 Control 确认，已停止重发";
            send_immediately = true;
            dirty = true;
        }

        if (state.target_command.interaction.operation !=
                robot_base::InteractionRequest::Operation::NONE &&
            std::chrono::duration<double>(
                now - state.interaction_requested_at).count() >
                config.hmi.request_timeout) {
            state.target_command.interaction.operation =
                robot_base::InteractionRequest::Operation::NONE;
            state.target_command.interaction.action.clear();
            state.last_action = "交互动作请求超时，已停止重发";
            send_immediately = true;
            dirty = true;
        }

        if (state.highlighted_key >= 0 && now >= state.highlight_until) {
            state.highlighted_key = -1;
            dirty = true;
        }

        if (state.reference_start_pulse &&
            now >= state.reference_start_until) {
            state.reference_start_pulse = false;
            send_immediately = true;
            dirty = true;
        }

        if (switch_stage != SwitchStage::IDLE &&
            (!state.status_online || state.fault.latched ||
                state.status.mode == ControlMode::SAFETY ||
                std::chrono::duration<double>(now - switch_started_at).count()
                    > 4.0 * config.hmi.request_timeout)) {
            blocked_policy = desired_policy;
            switch_stage = SwitchStage::IDLE;
            state.transition.active = false;
            state.pending_policy.clear();
            ZeroVelocity(&state);
            state.last_action = "自动切换已停止：状态、安全条件或超时";
            send_immediately = true;
            dirty = true;
        }
        if (automatic_policy_switch && switch_stage == SwitchStage::IDLE &&
            !desired_policy.empty() && desired_policy != blocked_policy &&
            state.status_online && !state.fault.latched &&
            state.status.mode != ControlMode::SAFETY &&
            state.status.mode != ControlMode::POWER_OFF &&
            state.status.active_policy != desired_policy &&
            !state.transition.active && state.pending_policy.empty()) {
            switch_target = desired_policy;
            switch_stage = SwitchStage::TO_DAMP;
            switch_started_at = now;
            ZeroVelocity(&state);
            send_immediately = true;
            dirty = true;
        }
        if (switch_stage != SwitchStage::IDLE &&
            !state.transition.active && state.pending_policy.empty()) {
            switch (switch_stage) {
            case SwitchStage::TO_DAMP:
                if (state.status.mode == ControlMode::DAMP) {
                    switch_stage = SwitchStage::SWITCH;
                } else if (RequestTransition(
                    &state, ControlMode::DAMP, 1, now)) {
                    send_immediately = true;
                }
                break;
            case SwitchStage::SWITCH:
                if (state.status.active_policy == switch_target) {
                    switch_stage = SwitchStage::TO_HOME;
                } else if (state.status.mode == ControlMode::DAMP) {
                    switch_target = desired_policy;
                    state.pending_policy = switch_target;
                    state.policy_source = state.status.active_policy;
                    state.policy_requested_at = now;
                    state.last_action = "自动请求切换策略 → " + switch_target;
                    send_immediately = true;
                }
                break;
            case SwitchStage::TO_HOME:
                if (state.status.mode == ControlMode::HOME) {
                    switch_stage = SwitchStage::TO_ZERO;
                } else if (state.status.mode == ControlMode::DAMP &&
                    RequestTransition(&state, ControlMode::HOME, 4, now)) {
                    send_immediately = true;
                }
                break;
            case SwitchStage::TO_ZERO:
                if (state.status.mode == ControlMode::ZERO) {
                    switch_stage = SwitchStage::TO_RL;
                } else if (state.status.mode == ControlMode::HOME &&
                    RequestTransition(&state, ControlMode::ZERO, 2, now)) {
                    send_immediately = true;
                }
                break;
            case SwitchStage::TO_RL:
                if (state.status.mode == ControlMode::RL) {
                    switch_stage = SwitchStage::IDLE;
                    state.last_action = "自动切换完成 → " +
                        state.status.active_policy;
                    dirty = true;
                    if (have_ros_command &&
                        state.status.active_policy == desired_policy) {
                        const auto *limits = ActiveCommandLimits(state);
                        if (limits) {
                            const bool moving = requested_vx != 0.0 ||
                                requested_vy != 0.0 || requested_wz != 0.0;
                            state.target_command.vx = std::clamp(
                                static_cast<float>(requested_vx +
                                    (moving ? cmd_vel_bias_x : 0.0)),
                                limits->min_vx, limits->max_vx);
                            state.target_command.vy = std::clamp(
                                static_cast<float>(requested_vy +
                                    (moving ? cmd_vel_bias_y : 0.0)),
                                limits->min_vy, limits->max_vy);
                            state.target_command.wz = std::clamp(
                                static_cast<float>(requested_wz +
                                    (moving ? cmd_vel_bias_yaw : 0.0)),
                                limits->min_wz, limits->max_wz);
                            send_immediately = true;
                        }
                    }
                } else if (state.status.mode == ControlMode::ZERO &&
                    state.status.zero_ready &&
                    RequestTransition(&state, ControlMode::RL, 3, now)) {
                    send_immediately = true;
                }
                break;
            case SwitchStage::IDLE:
                break;
            }
        }
        if (!requested_interaction.empty() &&
            switch_stage == SwitchStage::IDLE &&
            state.status_online && state.status.hmi_connected &&
            state.status.mode == ControlMode::RL &&
            state.status.active_policy == "stand_mjlab") {
            const auto *actions = ActiveInteractionActions(state);
            const auto action = actions ? std::find_if(actions->begin(),
                actions->end(), [&](const InteractionAction &candidate) {
                    return candidate.key == requested_interaction;
                }) : std::vector<InteractionAction>::const_iterator{};
            if (actions && action != actions->end()) {
                send_immediately = RequestInteractionStart(
                    &state, *action, now) || send_immediately;
            } else {
                state.last_action = "未找到交互动作: " + requested_interaction;
            }
            requested_interaction.clear();
            dirty = true;
        }
        const int key = ReadUiKey();
        if (key >= 0) {
            dirty = true;
            if (key == 'f') {
                if (switch_stage != SwitchStage::IDLE) {
                    blocked_policy = desired_policy;
                    switch_stage = SwitchStage::IDLE;
                    state.pending_policy.clear();
                    state.transition.active = false;
                    ZeroVelocity(&state);
                }
                send_immediately = RequestShortcut(&state, key, now) ||
                    send_immediately;
            } else if (key == 'x') {
                send_immediately = RequestFaultAcknowledgement(&state) ||
                    send_immediately;
            } else if (key == 'c') {
                send_immediately = RequestInteractionCancel(&state, now) ||
                    send_immediately;
            } else if (switch_stage != SwitchStage::IDLE) {
                state.last_action = "自动切换进行中，请等待 Control 确认";
            } else if (state.page == HmiPage::POLICY_SELECT) {
                if ((key == kKeyUp || key == 'k') && !state.policies.empty()) {
                    state.policy_cursor_idx = (state.policy_cursor_idx - 1 +
                        static_cast<int>(state.policies.size())) %
                        static_cast<int>(state.policies.size());
                } else if ((key == kKeyDown || key == 'j') &&
                    !state.policies.empty()) {
                    state.policy_cursor_idx = (state.policy_cursor_idx + 1) %
                        static_cast<int>(state.policies.size());
                } else if ((key == '\r' || key == '\n') &&
                    !state.policies.empty()) {
                    if (!state.status_online) {
                        state.last_action = "策略未切换：Control 状态已断开";
                    } else if (state.fault.latched) {
                        state.last_action = "策略未切换：存在锁存故障";
                    } else if (state.status.mode != ControlMode::POWER_OFF &&
                        state.status.mode != ControlMode::DAMP) {
                        state.last_action =
                            "策略未切换：请先进入 POWER_OFF 或 DAMP";
                    } else if (state.policy_cursor_idx ==
                        state.active_policy_idx) {
                        state.last_action = "所选策略已经生效";
                        state.page = HmiPage::MAIN;
                    } else {
                        state.pending_policy =
                            state.policies[state.policy_cursor_idx];
                        state.policy_source = state.status.active_policy;
                        state.policy_requested_at = now;
                        state.last_action = "请求切换策略 → "
                            + state.pending_policy + "，等待 Control 确认";
                        state.page = HmiPage::MAIN;
                        send_immediately = true;
                    }
                } else if (key == kKeyEscape || key == 'p') {
                    state.policy_cursor_idx = state.active_policy_idx;
                    state.page = HmiPage::MAIN;
                    state.last_action = "取消策略选择";
                }
            } else if (state.page == HmiPage::INTERACTION_SELECT) {
                const auto *actions = ActiveInteractionActions(state);
                const int count = actions
                    ? static_cast<int>(actions->size()) : 0;
                if ((key == kKeyUp || key == 'k') && count > 0) {
                    state.interaction_cursor_idx =
                        (state.interaction_cursor_idx - 1 + count) % count;
                } else if ((key == kKeyDown || key == 'j') && count > 0) {
                    state.interaction_cursor_idx =
                        (state.interaction_cursor_idx + 1) % count;
                } else if ((key == '\r' || key == '\n') && count > 0) {
                    const bool started = RequestInteractionStart(&state,
                        (*actions)[state.interaction_cursor_idx], now);
                    send_immediately = started || send_immediately;
                } else if (key == kKeyEscape || key == 'a') {
                    state.page = HmiPage::MAIN;
                    state.last_action = "取消交互动作选择";
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
                    state.target_command.vx = std::clamp(
                        state.target_command.vx + config.hmi.step_vx,
                        limits->min_vx, limits->max_vx);
                    state.last_action = "W：增加前进速度";
                    send_immediately = true;
                } else if (key == 's') {
                    state.target_command.vx = std::clamp(
                        state.target_command.vx - config.hmi.step_vx,
                        limits->min_vx, limits->max_vx);
                    state.last_action = "S：增加后退速度";
                    send_immediately = true;
                } else if (key == 'a') {
                    state.target_command.vy = std::clamp(
                        state.target_command.vy + config.hmi.step_vy,
                        limits->min_vy, limits->max_vy);
                    state.last_action = "A：增加左移速度";
                    send_immediately = true;
                } else if (key == 'd') {
                    state.target_command.vy = std::clamp(
                        state.target_command.vy - config.hmi.step_vy,
                        limits->min_vy, limits->max_vy);
                    state.last_action = "D：增加右移速度";
                    send_immediately = true;
                } else if (key == 'q') {
                    state.target_command.wz = std::clamp(
                        state.target_command.wz + config.hmi.step_wz,
                        limits->min_wz, limits->max_wz);
                    state.last_action = "Q：增加左转角速度";
                    send_immediately = true;
                } else if (key == 'e') {
                    state.target_command.wz = std::clamp(
                        state.target_command.wz - config.hmi.step_wz,
                        limits->min_wz, limits->max_wz);
                    state.last_action = "E：增加右转角速度";
                    send_immediately = true;
                } else if (key == ' ') {
                    ZeroVelocity(&state);
                    have_ros_command = false;
                    state.last_action = "SPACE：速度清零";
                    send_immediately = true;
                }
                if (send_immediately && (key == 'w' || key == 's' ||
                    key == 'a' || key == 'd' || key == 'q' ||
                    key == 'e' || key == ' ')) {
                    state.highlighted_key = key;
                    state.highlight_until = now + std::chrono::milliseconds(
                        config.hmi.key_highlight_ms);
                }
            } else {
                if (key == 'g') {
                    send_immediately = RequestReferenceStart(&state, now) ||
                        send_immediately;
                } else if (key == kKeyLeft) {
                    send_immediately = RequestByArrow(&state, false, now) ||
                        send_immediately;
                } else if (key == kKeyRight) {
                    send_immediately = RequestByArrow(&state, true, now) ||
                        send_immediately;
                } else if (key == 'p') {
                    if (state.policies.empty()) {
                        state.last_action = "没有配置可选策略";
                    } else {
                        state.policy_cursor_idx = state.active_policy_idx;
                        state.page = HmiPage::POLICY_SELECT;
                        state.last_action = "选择策略";
                    }
                } else if (key == 'a') {
                    const auto *actions = ActiveInteractionActions(state);
                    if (!actions || actions->empty()) {
                        state.last_action = "当前策略没有注册交互动作";
                    } else if (!state.status_online ||
                        !state.status.hmi_connected ||
                        state.status.mode != ControlMode::RL) {
                        state.last_action =
                            "交互动作页仅在真实 FSM=RL 且心跳正常时开放";
                    } else {
                        state.page = HmiPage::INTERACTION_SELECT;
                        state.last_action = "选择交互动作";
                    }
                } else if (key == 'v' || key == '\r' || key == '\n') {
                    if (state.status_online && state.status.hmi_connected &&
                        state.status.mode == ControlMode::RL &&
                        ActiveCommandLimits(state)) {
                        state.page = HmiPage::VELOCITY;
                        state.last_action = "进入键盘/ROS 速度控制";
                    } else if (state.status_online &&
                        state.status.mode == ControlMode::RL) {
                        state.last_action =
                            "当前策略未配置 command.limits，不接受速度命令";
                    } else {
                        state.last_action =
                            "速度页仅在真实 FSM=RL 且心跳正常时开放";
                    }
                } else if (key == 'o' || key == 'h' || key == 'z' ||
                    key == 'r') {
                    send_immediately = RequestShortcut(&state, key, now) ||
                        send_immediately;
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
    state.reference_start_pulse = false;
    state.reference_start_requested = false;
    state.target_command.interaction.sequence =
        NextInteractionSequence(state);
    state.target_command.interaction.operation =
        robot_base::InteractionRequest::Operation::CANCEL;
    state.target_command.interaction.action.clear();
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
