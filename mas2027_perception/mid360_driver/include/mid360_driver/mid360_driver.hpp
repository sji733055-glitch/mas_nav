/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#define ASIO_NO_DEPRECATED
#include <utility>
#include <asio.hpp>
#include <atomic>
#include <functional>
#include <unordered_map>
#include <vector>

namespace mid360_driver {

    struct DriverRobustnessConfig {
        bool validate_crc;
        double max_packet_time_jump;
        double max_packet_time_span;
        double max_point_range;
        double max_imu_acc;
        double max_imu_gyro;
        double min_drop_log_interval;
        // 某台雷达静默超过这么多秒后，允许重新锚定它的时间戳（雷达重启/断网恢复）。
        // <=0 关闭重锚定。
        double packet_resync_silence;
    };

    struct Point {
        double timestamp;
        float x, y, z;
        float intensity;
    };

    struct ImuMsg {
        double timestamp;
        float angular_velocity_x;
        float angular_velocity_y;
        float angular_velocity_z;
        float linear_acceleration_x;
        float linear_acceleration_y;
        float linear_acceleration_z;
    };

    struct IpAddressHasher {
        std::size_t operator()(const asio::ip::address &addr) const noexcept;
    };

    // 单路数据流（点云或 IMU，两者时间戳独立校验）里每台雷达的时间戳锚点：
    // timestamp = 上一次接受的包时间戳（水位线），wall_time = 接受它时的主机墙钟。
    struct TimestampAnchor {
        double timestamp = 0.0;
        double wall_time = 0.0;
    };

    class Mid360Driver {
    private:
        std::atomic<bool> is_running = true;
        asio::ip::address host_ip;
        asio::ip::udp::socket receive_pointcloud_socket;
        asio::ip::udp::socket receive_imu_socket;
        DriverRobustnessConfig robustness_config;
        std::unordered_map<asio::ip::address, double, IpAddressHasher> delta_time_map;
        std::unordered_map<asio::ip::address, TimestampAnchor, IpAddressHasher> last_lidar_timestamp_map;
        std::unordered_map<asio::ip::address, TimestampAnchor, IpAddressHasher> last_imu_timestamp_map;
        std::function<void(const asio::ip::address &lidar_ip, const std::vector<Point> &points)> on_receive_pointcloud;
        std::function<void(const asio::ip::address &lidar_ip, const ImuMsg &imu_msg)> on_receive_imu;

        // 解析一个包的时间戳：NO_SYNC 时把雷达内部时钟换算到主机墙钟，再做跳变校验。
        // 静默超过 packet_resync_silence 才允许重锚定，否则一次超过 max_packet_time_jump 的断线会让水位线永久卡死。
        bool resolve_packet_timestamp(const asio::ip::address &address,
                                      bool no_sync_timestamp,
                                      double raw_timestamp,
                                      std::unordered_map<asio::ip::address, TimestampAnchor, IpAddressHasher> &anchors,
                                      double &timestamp_out);

    public:
        Mid360Driver(asio::io_context &io_context,
                     const asio::ip::address &host_ip,
                     DriverRobustnessConfig robustness_config,
                     std::function<void(const asio::ip::address &lidar_ip, const std::vector<Point> &points)> on_receive_pointcloud,
                     std::function<void(const asio::ip::address &lidar_ip, const ImuMsg &imu_msg)> on_receive_imu);

        ~Mid360Driver();

        void stop();

        asio::awaitable<void> receive_pointcloud();

        asio::awaitable<void> receive_imu();
    };

}// namespace mid360_driver
