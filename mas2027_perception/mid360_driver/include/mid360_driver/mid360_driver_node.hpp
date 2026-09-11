/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "mid360_driver/mid360_driver.hpp"
#include <deque>
#include <mutex>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unordered_map>

namespace mid360_driver {

    constexpr std::size_t MAX_PENDING_POINTS = 200000;
    constexpr std::size_t MAX_PENDING_IMU = 1000;
    // 单台雷达最多缓存几帧等待配对。两路都按同一个定时器切帧，正常只会有 1 帧在队里；
    // 一台掉线时另一台涨到这个上限就丢最旧的，避免无限增长。
    constexpr std::size_t MAX_MERGE_QUEUE_FRAMES = 8;

    class LidarPublisher {
    private:
        std::vector<Point> points_wait_to_publish;
        std::vector<ImuMsg> imu_wait_to_publish;
        std::vector<Point> points_to_publish;
        std::vector<ImuMsg> imu_to_publish;
        bool is_initialized = false;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pointcloud_publisher;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_publisher;
        DriverRobustnessConfig robustness_config;

    public:
        LidarPublisher() = default;
        void configure_robustness(const DriverRobustnessConfig &config);
        void ensure_initialized(rclcpp::Node &node, const std::string &lidar_topic, const std::string &imu_topic);
        void ensure_initialized(rclcpp::Node &node, const std::string &lidar_topic, const std::string &imu_topic, const asio::ip::address &lidar_ip);
        void on_receive_pointcloud(const std::vector<Point> &points);
        void on_receive_imu(const ImuMsg &imu_msg);
        void prepare_pointcloud_to_publish();
        void prepare_imu_to_publish();
        void publish_pointcloud(const std::string &frame_id) const;
        // 发布外部组好的一帧（融合帧走这条），不经过 points_to_publish。
        void publish_points(const std::vector<Point> &points, const std::string &frame_id) const;
        void publish_imu(const std::string &frame_id) const;
    };

    class Mid360DriverNode : public rclcpp::Node {
    private:
        asio::io_context io_context;
        std::mutex multi_lidar_mutex_;
        std::thread io_thread;
        std::unique_ptr<mid360_driver::Mid360Driver> mid360_driver;
        LidarPublisher lidar_publisher;
        std::unordered_map<asio::ip::address, LidarPublisher, IpAddressHasher> multi_lidar_publishers;
        rclcpp::TimerBase::SharedPtr publish_pointcloud_timer;
        // 融合用：每台雷达先按定时器切成帧，再按 base_timestamp 配对。
        // base_timestamp 取该帧点时间戳的最小值，和 publish_pointcloud 写 header.stamp 的取法一致。
        struct MergeFrame { double base_timestamp; std::vector<Point> points; };
        std::vector<Point> merge_front_pending_;
        std::vector<Point> merge_back_pending_;
        std::deque<MergeFrame> merge_front_queue_;
        std::deque<MergeFrame> merge_back_queue_;
        std::size_t merge_drop_count_ = 0;
        std::size_t merge_starve_count_ = 0;
        double merge_max_interval_s_ = 0.005;
        rclcpp::TimerBase::SharedPtr publish_imu_timer;

        static void stage_merge_points(std::vector<Point> &pending, const std::vector<Point> &points);
        void enqueue_merge_frame(std::deque<MergeFrame> &queue, std::vector<Point> &&points);
        // 取出所有能配对的融合帧；不发布，发布放到锁外做。
        std::vector<std::vector<Point>> collect_merged_frames();

    public:
        explicit Mid360DriverNode(const rclcpp::NodeOptions &options);
        ~Mid360DriverNode() override;
    };

}// namespace mid360_driver
