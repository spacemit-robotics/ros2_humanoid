/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file humanoid_head_tf_node.cpp
 * @brief Publish LingLong head and camera TF from a read-only state SHM tap.
 */

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_base.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "transport_packet.h"

namespace
{

constexpr std::size_t kShmHeaderSize = 64;

uint32_t LoadAcquire(const uint32_t* value)
{
    return __atomic_load_n(value, __ATOMIC_ACQUIRE);
}

geometry_msgs::msg::Quaternion QuaternionFromRpy(double roll, double pitch, double yaw)
{
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);

    geometry_msgs::msg::Quaternion result;
    result.w = cr * cp * cy + sr * sp * sy;
    result.x = sr * cp * cy - cr * sp * sy;
    result.y = cr * sp * cy + sr * cp * sy;
    result.z = cr * cp * sy - sr * sp * cy;
    return result;
}

class StateShmTap
{
public:
    explicit StateShmTap(std::string channel_name)
        : channel_name_(std::move(channel_name))
    {
    }

    ~StateShmTap()
    {
        Close();
    }

    StateShmTap(const StateShmTap&) = delete;
    StateShmTap& operator=(const StateShmTap&) = delete;

    bool ReadLatest(transport::RobotStatePacket* packet)
    {
        if (!packet || (!mapping_ && !Open()))
        {
            return false;
        }

        for (int attempt = 0; attempt < 3; ++attempt)
        {
            const uint32_t capacity = LoadAcquire(HeaderWord(2));
            const uint32_t slot_size = LoadAcquire(HeaderWord(3));
            const uint32_t write_index = LoadAcquire(HeaderWord(0));
            if (capacity == 0 || write_index >= capacity ||
                slot_size < sizeof(transport::RobotStatePacket) ||
                kShmHeaderSize + static_cast<std::size_t>(capacity) * slot_size > mapping_size_)
            {
                return false;
            }

            const uint32_t latest_index = write_index == 0 ? capacity - 1 : write_index - 1;
            const auto* slot = static_cast<const uint8_t*>(mapping_) + kShmHeaderSize +
                static_cast<std::size_t>(latest_index) * slot_size;
            transport::RobotStatePacket candidate{};
            std::memcpy(&candidate, slot, sizeof(candidate));
            __atomic_thread_fence(__ATOMIC_ACQUIRE);
            if (LoadAcquire(HeaderWord(0)) != write_index)
            {
                continue;
            }
            if (!transport::ValidHeader(candidate.header, transport::MsgType::ROBOT_STATE))
            {
                return false;
            }
            if (candidate.header.seq == last_sequence_)
            {
                return false;
            }
            last_sequence_ = candidate.header.seq;
            *packet = candidate;
            return true;
        }
        return false;
    }

private:
    bool Open()
    {
        file_descriptor_ = shm_open(channel_name_.c_str(), O_RDONLY, 0);
        if (file_descriptor_ < 0)
        {
            return false;
        }

        struct stat status {};
        if (fstat(file_descriptor_, &status) != 0 ||
            status.st_size < static_cast<off_t>(kShmHeaderSize))
        {
            Close();
            return false;
        }
        mapping_size_ = static_cast<std::size_t>(status.st_size);
        mapping_ = mmap(nullptr, mapping_size_, PROT_READ, MAP_SHARED, file_descriptor_, 0);
        if (mapping_ == MAP_FAILED)
        {
            mapping_ = nullptr;
            Close();
            return false;
        }
        return true;
    }

    void Close()
    {
        if (mapping_)
        {
            munmap(mapping_, mapping_size_);
            mapping_ = nullptr;
        }
        if (file_descriptor_ >= 0)
        {
            close(file_descriptor_);
            file_descriptor_ = -1;
        }
        mapping_size_ = 0;
    }

    const uint32_t* HeaderWord(std::size_t index) const
    {
        return static_cast<const uint32_t*>(mapping_) + index;
    }

    std::string channel_name_;
    int file_descriptor_ = -1;
    void* mapping_ = nullptr;
    std::size_t mapping_size_ = 0;
    uint32_t last_sequence_ = 0;
};

std::size_t FindJointIndex(
    const std::vector<std::string>& joint_names, const std::string& target)
{
    const auto position = std::find(joint_names.begin(), joint_names.end(), target);
    if (position == joint_names.end())
    {
        throw std::runtime_error("joint not found in robot_base.joint_names: " + target);
    }
    return static_cast<std::size_t>(std::distance(joint_names.begin(), position));
}

}  // namespace

class HumanoidHeadTfNode final : public rclcpp::Node
{
public:
    explicit HumanoidHeadTfNode(const std::string& config_path)
        : Node("humanoid_head_tf")
    {
        const auto yaml = robot_base::YamlFile::Load(config_path);
        if (yaml.Read<std::string>("transport.type").value_or("shm") != "shm")
        {
            throw std::runtime_error("humanoid_head_tf_node requires transport.type=shm");
        }
        const auto joint_names = yaml.Read<std::vector<std::string>>(
            "robot_base.joint_names").value_or(std::vector<std::string>{});
        yaw_joint_index_ = FindJointIndex(joint_names, "head_yaw_joint");
        pitch_joint_index_ = FindJointIndex(joint_names, "head_pitch_joint");

        const std::string shm_prefix =
            yaml.Read<std::string>("transport.shm.prefix").value_or("robot_name");
        state_tap_ = std::make_unique<StateShmTap>("/hmrs_" + shm_prefix + "_state");

        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
        yaw_frame_ = declare_parameter<std::string>("yaw_frame", "head_yaw_link");
        pitch_frame_ = declare_parameter<std::string>("pitch_frame", "head_pitch_link");
        camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_link");
        head_mount_xyz_ = ReadVectorParameter("head_mount_xyz", {0.0, 0.0, 0.4168});
        camera_xyz_ = ReadVectorParameter("camera_xyz", {0.0, 0.0, 0.0});
        camera_rpy_ = ReadVectorParameter("camera_rpy", {0.0, 0.0, 0.0});
        camera_tf_enabled_ = declare_parameter<bool>("camera_tf_enabled", false);
        publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
        state_timeout_s_ = declare_parameter<double>("state_timeout_s", 0.1);
        if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0 ||
            !std::isfinite(state_timeout_s_) || state_timeout_s_ <= 0.0)
        {
            throw std::runtime_error("publish_rate_hz and state_timeout_s must be positive");
        }

        dynamic_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
        if (camera_tf_enabled_)
        {
            PublishCameraTransform();
        }
        else
        {
            RCLCPP_WARN(get_logger(),
                "camera TF is disabled; set camera_tf_enabled=true and provide camera_xyz/rpy");
        }

        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / publish_rate_hz_)),
            [this]() { Tick(); });
        RCLCPP_INFO(get_logger(),
            "Head TF tap started: joints=(%zu,%zu) state_shm=/hmrs_%s_state",
            yaw_joint_index_, pitch_joint_index_, shm_prefix.c_str());
    }

private:
    std::array<double, 3> ReadVectorParameter(
        const std::string& name, const std::vector<double>& defaults)
    {
        const auto values = declare_parameter<std::vector<double>>(name, defaults);
        if (values.size() != 3 ||
            !std::all_of(values.begin(), values.end(), [](double value) {
                return std::isfinite(value);
            }))
        {
            throw std::runtime_error(name + " must contain three finite values");
        }
        return {values[0], values[1], values[2]};
    }

    void PublishCameraTransform()
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = now();
        transform.header.frame_id = pitch_frame_;
        transform.child_frame_id = camera_frame_;
        transform.transform.translation.x = camera_xyz_[0];
        transform.transform.translation.y = camera_xyz_[1];
        transform.transform.translation.z = camera_xyz_[2];
        transform.transform.rotation =
            QuaternionFromRpy(camera_rpy_[0], camera_rpy_[1], camera_rpy_[2]);
        static_broadcaster_->sendTransform(transform);
    }

    void Tick()
    {
        transport::RobotStatePacket packet{};
        if (state_tap_->ReadLatest(&packet))
        {
            if (packet.num_dof <= static_cast<int32_t>(
                    std::max(yaw_joint_index_, pitch_joint_index_)))
            {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                    "state has %d joints, but head joint indices are %zu and %zu",
                    packet.num_dof, yaw_joint_index_, pitch_joint_index_);
                return;
            }
            yaw_position_ = packet.joint_pos[yaw_joint_index_];
            pitch_position_ = packet.joint_pos[pitch_joint_index_];
            last_state_time_ = std::chrono::steady_clock::now();
            have_state_ = true;
        }

        if (!have_state_ ||
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - last_state_time_).count() > state_timeout_s_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "waiting for fresh humanoid state SHM feedback");
            return;
        }

        const auto stamp = now();
        geometry_msgs::msg::TransformStamped yaw_transform;
        yaw_transform.header.stamp = stamp;
        yaw_transform.header.frame_id = base_frame_;
        yaw_transform.child_frame_id = yaw_frame_;
        yaw_transform.transform.translation.x = head_mount_xyz_[0];
        yaw_transform.transform.translation.y = head_mount_xyz_[1];
        yaw_transform.transform.translation.z = head_mount_xyz_[2];
        yaw_transform.transform.rotation = QuaternionFromRpy(0.0, 0.0, yaw_position_);

        geometry_msgs::msg::TransformStamped pitch_transform;
        pitch_transform.header.stamp = stamp;
        pitch_transform.header.frame_id = yaw_frame_;
        pitch_transform.child_frame_id = pitch_frame_;
        pitch_transform.transform.rotation = QuaternionFromRpy(0.0, pitch_position_, 0.0);
        dynamic_broadcaster_->sendTransform({yaw_transform, pitch_transform});
    }

    std::string base_frame_;
    std::string yaw_frame_;
    std::string pitch_frame_;
    std::string camera_frame_;
    std::array<double, 3> head_mount_xyz_{};
    std::array<double, 3> camera_xyz_{};
    std::array<double, 3> camera_rpy_{};
    bool camera_tf_enabled_ = false;
    double publish_rate_hz_ = 50.0;
    double state_timeout_s_ = 0.1;
    std::size_t yaw_joint_index_ = 0;
    std::size_t pitch_joint_index_ = 0;
    double yaw_position_ = 0.0;
    double pitch_position_ = 0.0;
    bool have_state_ = false;
    std::chrono::steady_clock::time_point last_state_time_{};
    std::unique_ptr<StateShmTap> state_tap_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    const auto arguments = rclcpp::remove_ros_arguments(argc, argv);
    if (arguments.size() < 2 || arguments[1] == "-h" || arguments[1] == "--help")
    {
        const bool help = arguments.size() >= 2;
        fprintf(help ? stdout : stderr,
            "Usage: %s <config.yaml> [--ros-args <parameters>]\n", argv[0]);
        rclcpp::shutdown();
        return help ? 0 : 1;
    }

    try
    {
        rclcpp::spin(std::make_shared<HumanoidHeadTfNode>(arguments[1]));
    }
    catch (const std::exception& error)
    {
        fprintf(stderr, "[humanoid_head_tf] %s\n", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
