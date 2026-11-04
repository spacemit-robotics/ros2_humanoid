/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file g1_body_tf_node.cpp
 * @brief Read-only G1 waist/IMU TF from native hardware state SHM.
 */

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_base.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "transport_packet.h"

namespace
{

// Matches humanoid_common/src/transport/shm/shm_transport.cpp. This observer
// must never call Shm::Read(): its read_index belongs to control_runtime.
class StateShmTap
{
public:
    explicit StateShmTap(std::string name) : name_(std::move(name)) {}
    ~StateShmTap() { Close(); }
    StateShmTap(const StateShmTap&) = delete;
    StateShmTap& operator=(const StateShmTap&) = delete;

    void Close()
    {
        if (mapping_) munmap(mapping_, size_);
        if (fd_ >= 0) close(fd_);
        mapping_ = nullptr;
        fd_ = -1;
        have_sequence_ = false;
    }

    bool ReadLatest(transport::RobotStatePacket* packet)
    {
        if (!mapping_ && !Open()) return false;
        for (int attempt = 0; attempt < 3; ++attempt)
        {
            const auto capacity = Word(2);
            const auto slot_size = Word(3);
            const auto write_index = Word(0);
            if (capacity < 2 || slot_size < sizeof(*packet) || write_index >= capacity ||
                capacity > (size_ - kHeaderSize) / slot_size)
            {
                return false;
            }
            const auto latest = write_index == 0 ? capacity - 1 : write_index - 1;
            const auto* slot = static_cast<const uint8_t*>(mapping_) + kHeaderSize +
                static_cast<std::size_t>(latest) * slot_size;
            transport::RobotStatePacket candidate{};
            std::memcpy(&candidate, slot, sizeof(candidate));
            __atomic_thread_fence(__ATOMIC_ACQUIRE);
            transport::RobotStatePacket confirm{};
            std::memcpy(&confirm, slot, sizeof(confirm));
            __atomic_thread_fence(__ATOMIC_ACQUIRE);
            // A ring wrap can restore write_index while a preempted copy is in
            // progress. Compare the whole sample, not just its sequence header.
            if (Word(0) != write_index || std::memcmp(&confirm, &candidate, sizeof(confirm)) != 0)
                continue;
            if (!transport::ValidRobotStatePacket(candidate)) return false;
            if (have_sequence_ && candidate.header.seq == last_sequence_) return false;
            have_sequence_ = true;
            last_sequence_ = candidate.header.seq;
            *packet = candidate;
            return true;
        }
        return false;
    }

private:
    bool Open()
    {
        fd_ = shm_open(name_.c_str(), O_RDONLY, 0);
        if (fd_ < 0) return false;
        struct stat status {};
        if (fstat(fd_, &status) != 0 || status.st_size < static_cast<off_t>(kHeaderSize))
        {
            Close();
            return false;
        }
        size_ = static_cast<std::size_t>(status.st_size);
        mapping_ = mmap(nullptr, size_, PROT_READ, MAP_SHARED, fd_, 0);
        if (mapping_ == MAP_FAILED)
        {
            mapping_ = nullptr;
            Close();
            return false;
        }
        return true;
    }

    uint32_t Word(std::size_t index) const
    {
        return __atomic_load_n(static_cast<const uint32_t*>(mapping_) + index, __ATOMIC_ACQUIRE);
    }

    static constexpr std::size_t kHeaderSize = 64;
    std::string name_;
    int fd_ = -1;
    void* mapping_ = nullptr;
    std::size_t size_ = 0;
    bool have_sequence_ = false;
    uint32_t last_sequence_ = 0;
};

tf2::Quaternion Rpy(double roll, double pitch, double yaw)
{
    tf2::Quaternion result;
    result.setRPY(roll, pitch, yaw);
    return result;
}

geometry_msgs::msg::TransformStamped Transform(
    const std::string& parent, const std::string& child, const rclcpp::Time& stamp,
    const std::array<double, 3>& xyz, const tf2::Quaternion& rotation)
{
    geometry_msgs::msg::TransformStamped result;
    result.header.stamp = stamp;
    result.header.frame_id = parent;
    result.child_frame_id = child;
    result.transform.translation.x = xyz[0];
    result.transform.translation.y = xyz[1];
    result.transform.translation.z = xyz[2];
    result.transform.rotation.x = rotation.x();
    result.transform.rotation.y = rotation.y();
    result.transform.rotation.z = rotation.z();
    result.transform.rotation.w = rotation.w();
    return result;
}

}  // namespace

class G1BodyTfNode final : public rclcpp::Node
{
public:
    explicit G1BodyTfNode(const std::string& config_path) : Node("g1_body_tf")
    {
        const auto yaml = robot_base::YamlFile::Load(config_path);
        if (yaml.Read<std::string>("transport.type").value_or("shm") != "shm")
            throw std::runtime_error("g1_body_tf_node requires transport.type=shm");
        if (get_parameter("use_sim_time").as_bool())
            throw std::runtime_error("g1_body_tf_node uses hardware receipt time; use_sim_time must be false");
        const auto names = yaml.Read<std::vector<std::string>>("robot_base.joint_names")
            .value_or(std::vector<std::string>{});
        num_dof_ = names.size();
        if (names.empty() || names.size() > transport::kMaxDof ||
            std::set<std::string>(names.begin(), names.end()).size() != names.size())
            throw std::runtime_error("robot_base.joint_names must contain 1..64 unique names");
        const std::array<std::string, 3> waist_names = {
            "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"};
        for (std::size_t i = 0; i < waist_names.size(); ++i)
        {
            const auto found = std::find(names.begin(), names.end(), waist_names[i]);
            if (found == names.end()) throw std::runtime_error("missing joint: " + waist_names[i]);
            waist_indices_[i] = static_cast<std::size_t>(std::distance(names.begin(), found));
        }
        const auto prefix = yaml.Read<std::string>("transport.shm.prefix").value_or("g1");
        if (prefix.empty() || prefix.find('/') != std::string::npos)
            throw std::runtime_error("transport.shm.prefix must be nonempty and contain no slash");
        state_tap_ = std::make_unique<StateShmTap>("/hmrs_" + prefix + "_state");

        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
        footprint_frame_ = declare_parameter<std::string>("footprint_frame", "base_footprint");
        camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_link");
        const std::vector<std::string> frames = {base_frame_, footprint_frame_, camera_frame_,
            "pelvis", "waist_yaw_link", "waist_roll_link", "torso_link", "d435_link"};
        if (std::set<std::string>(frames.begin(), frames.end()).size() != frames.size() ||
            std::any_of(frames.begin(), frames.end(), [](const auto& frame) {
                return frame.empty() || frame.front() == '/';
            }))
            throw std::runtime_error("TF frame names must be distinct, nonempty and have no leading slash");
        base_height_ = ReadPositive("base_height", 0.793);
        publish_footprint_ = declare_parameter<bool>("publish_footprint_tf", true);
        state_timeout_ = ReadPositive("state_timeout_s", 0.1);
        const double rate = ReadPositive("publish_rate_hz", 50.0);
        if (rate > 1000.0) throw std::runtime_error("publish_rate_hz must not exceed 1000");
        std::array<double, 3> camera_xyz{}, camera_rpy{};
        const std::array<std::string, 3> axes = {"x", "y", "z"};
        const std::array<std::string, 3> angles = {"roll", "pitch", "yaw"};
        const std::array<double, 3> default_xyz = {0.0576235, 0.01753, 0.42987};
        for (std::size_t i = 0; i < 3; ++i)
        {
            camera_xyz[i] = ReadFinite("camera_" + axes[i], default_xyz[i]);
            camera_rpy[i] = ReadFinite("camera_" + angles[i], i == 1 ? 0.8307767239493009 : 0.0);
        }

        dynamic_tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
        const auto stamp = now();
        // g1_29dof_rev_1_0.urdf: static mechanical mount; RealSense owns internal TF.
        static_tf_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{
            Transform(base_frame_, "pelvis", stamp, {0, 0, 0}, Rpy(0, 0, 0)),
            Transform("torso_link", "d435_link", stamp, camera_xyz,
                Rpy(camera_rpy[0], camera_rpy[1], camera_rpy[2])),
            Transform("d435_link", camera_frame_, stamp, {0, 0, 0}, Rpy(0, 0, 0)),
        });
        last_progress_ = std::chrono::steady_clock::now();
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / rate)), [this]() { Tick(); });
        RCLCPP_INFO(get_logger(), "Read-only G1 TF: /hmrs_%s_state; pelvis height=%.3f m (configured)",
            prefix.c_str(), base_height_);
    }

private:
    double ReadFinite(const std::string& name, double default_value)
    {
        const double value = declare_parameter<double>(name, default_value);
        if (!std::isfinite(value)) throw std::runtime_error(name + " must be finite");
        return value;
    }

    double ReadPositive(const std::string& name, double default_value)
    {
        const double value = ReadFinite(name, default_value);
        if (value <= 0.0) throw std::runtime_error(name + " must be positive");
        return value;
    }

    void Tick()
    {
        const auto steady_now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(steady_now - last_progress_).count() > state_timeout_)
        {
            // Also reconnect if driver_runtime unlinks and recreates the SHM object.
            state_tap_->Close();
            have_device_time_ = false;
            last_progress_ = steady_now;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Waiting for valid, advancing G1 state; dynamic TF is paused");
        }
        transport::RobotStatePacket packet{};
        if (!state_tap_->ReadLatest(&packet)) return;
        if (packet.num_dof != static_cast<int32_t>(num_dof_) || packet.fault.active)
            return;
        const double norm = std::hypot(std::hypot(packet.base_quat[0], packet.base_quat[1]),
            std::hypot(packet.base_quat[2], packet.base_quat[3]));
        if (!std::isfinite(norm) || norm <= 1.0e-6) return;
        // HI91 time is device uptime, not ROS/system/steady time. Require actual
        // device-time progress, even if the driver republishes faults with new seq.
        // Establish a baseline before publishing, so a leftover SHM slot is ignored.
        if (!have_device_time_)
        {
            device_time_ = packet.time;
            have_device_time_ = true;
            return;
        }
        if (packet.time == device_time_) return;
        device_time_ = packet.time;
        last_progress_ = steady_now;
        const auto stamp = now();  // ROS receipt time; camera driver uses the same host clock.
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        if (publish_footprint_)
        {
            const tf2::Quaternion base(packet.base_quat[1] / norm, packet.base_quat[2] / norm,
                packet.base_quat[3] / norm, packet.base_quat[0] / norm);
            const double yaw = std::atan2(2 * (base.w() * base.z() + base.x() * base.y()),
                1 - 2 * (base.y() * base.y() + base.z() * base.z()));
            transforms.push_back(Transform(footprint_frame_, base_frame_, stamp,
                {0, 0, base_height_}, Rpy(0, 0, yaw).inverse() * base));
        }
        // URDF joint origin * joint axis rotation, including the roll pivot offset.
        transforms.push_back(Transform("pelvis", "waist_yaw_link", stamp, {0, 0, 0},
            Rpy(0, 0, packet.joint_pos[waist_indices_[0]])));
        transforms.push_back(Transform("waist_yaw_link", "waist_roll_link", stamp,
            {-0.0039635, 0, 0.044}, Rpy(packet.joint_pos[waist_indices_[1]], 0, 0)));
        transforms.push_back(Transform("waist_roll_link", "torso_link", stamp, {0, 0, 0},
            Rpy(0, packet.joint_pos[waist_indices_[2]], 0)));
        dynamic_tf_->sendTransform(transforms);
    }

    std::unique_ptr<StateShmTap> state_tap_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::array<std::size_t, 3> waist_indices_{};
    std::size_t num_dof_ = 0;
    std::string base_frame_, footprint_frame_, camera_frame_;
    double base_height_ = 0.793;
    double state_timeout_ = 0.1;
    bool publish_footprint_ = true;
    bool have_device_time_ = false;
    double device_time_ = 0.0;
    std::chrono::steady_clock::time_point last_progress_{};
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    const auto args = rclcpp::remove_ros_arguments(argc, argv);
    if (args.size() != 2 || args[1] == "-h" || args[1] == "--help")
    {
        fprintf(stderr, "Usage: %s <g1.yaml> [--ros-args <parameters>]\n", argv[0]);
        rclcpp::shutdown();
        return args.size() == 2 ? 0 : 1;
    }
    try
    {
        rclcpp::spin(std::make_shared<G1BodyTfNode>(args[1]));
    }
    catch (const std::exception& error)
    {
        fprintf(stderr, "[g1_body_tf] %s\n", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
