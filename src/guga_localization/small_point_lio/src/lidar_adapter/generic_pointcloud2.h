/**
 * Generic PointCloud2 adapter for Small Point-LIO.
 *
 * Reads only x/y/z from a sensor_msgs/PointCloud2 message, without requiring
 * livox-specific fields ("tag", "timestamp"). This makes Small Point-LIO work
 * with simulation point clouds (ign_sim_pointcloud_tool / ros_gz_bridge) that
 * lack those fields, while the livox adapters keep serving the real robot.
 *
 * NOTE: point timestamps are unavailable, so the preprocessor falls back to
 * the scan header stamp (all points share the same time).
 */

#pragma once

#include "base_lidar.h"
#include <cmath>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace small_point_lio {

    class GenericPointCloud2Adapter : public LidarAdapterBase {
    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;
        double last_frame_time_{ -1.0 };   // 上一帧帧头时间
        double frame_period_{ 0.05 };      // 帧周期(默认 20Hz),由相邻帧差自适应

    public:
        inline void setup_subscription(rclcpp::Node *node, const std::string &topic, std::function<void(const std::vector<common::Point> &)> callback) override {
            // Simulation PointCloud2 may contain a synthetic `time` field added by
            // ign_sim_pointcloud_tool. Treat scans as frame-synchronous unless
            // valid per-point timing is explicitly enabled.
            node->declare_parameter<bool>("use_point_time", false);
            const bool use_point_time = node->get_parameter("use_point_time").as_bool();
            subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
                    topic,
                    rclcpp::SensorDataQoS(),
                    [callback, this, use_point_time](const sensor_msgs::msg::PointCloud2 &msg) {
                        sensor_msgs::PointCloud2ConstIterator<float> out_x(msg, "x");
                        sensor_msgs::PointCloud2ConstIterator<float> out_y(msg, "y");
                        sensor_msgs::PointCloud2ConstIterator<float> out_z(msg, "z");
                        size_t size = msg.width * msg.height;
                        const double frame_time =
                            static_cast<double>(msg.header.stamp.sec) +
                            static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
                        // 用相邻帧帧头时间差估计帧周期,限制帧内时间偏移不超过一帧。
                        if (last_frame_time_ > 0.0 && frame_time > last_frame_time_) {
                            const double dt = frame_time - last_frame_time_;
                            if (dt > 0.005 && dt < 1.0) { frame_period_ = dt; }
                        }
                        last_frame_time_ = frame_time;
                        // 帧内逐点时间偏移(time 字段,单位 s)。仿真 ign_sim_pointcloud_tool
                        // 生成 0~0.1s 的偏移,但帧周期可能只有 0.05s(20Hz),直接加会导致
                        // 相邻帧点时间戳重叠 → EKF 时间同步错乱 → 运动发散。
                        // 这里把偏移归一化到 [0, frame_period_).
                        bool has_time_field = false;
                        float time_max = 0.0f;
                        for (const auto &field : msg.fields) {
                            if (field.name == "time") { has_time_field = true; break; }
                        }
                        std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> out_time;
                        if (use_point_time && has_time_field) {
                            out_time = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(
                                msg, "time");
                            // 先扫一遍求 time 最大值(避免依赖 0.1 硬编码)
                            auto scan = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(msg, "time");
                            for (size_t i = 0; i < size; ++i) {
                                const float t = **scan;
                                if (t > time_max) { time_max = t; }
                                ++(*scan);
                            }
                        }
                        const double period = std::max(0.01, frame_period_);
                        const double scale = (time_max > 1e-6) ? (period / static_cast<double>(time_max)) : 1.0;
                        std::vector<common::Point> pointcloud;
                        pointcloud.reserve(size);
                        for (size_t i = 0; i < size; ++i) {
                            common::Point new_point;
                            new_point.position << *out_x, *out_y, *out_z;
                            // Gazebo/bridge clouds may contain NaN/Inf returns.
                            // Letting them through bypasses the squared-distance
                            // checks and can leave isolated ghost points in ivox.
                            if (!std::isfinite(new_point.position.x()) ||
                                !std::isfinite(new_point.position.y()) ||
                                !std::isfinite(new_point.position.z())) {
                                ++out_x;
                                ++out_y;
                                ++out_z;
                                if (use_point_time && has_time_field && out_time) { ++(*out_time); }
                                continue;
                            }
                            if (use_point_time && has_time_field && out_time) {
                                new_point.timestamp = frame_time
                                    + static_cast<double>(**out_time) * scale;
                            } else {
                                new_point.timestamp = frame_time;
                            }
                            pointcloud.push_back(new_point);
                            ++out_x;
                            ++out_y;
                            ++out_z;
                            if (use_point_time && has_time_field && out_time) { ++(*out_time); }
                        }
                        callback(pointcloud);
                    });
        }
    };

}// namespace small_point_lio
