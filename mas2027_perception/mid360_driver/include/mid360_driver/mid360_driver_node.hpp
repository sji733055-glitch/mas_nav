/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "mid360_driver/merge_failover.hpp"
#include "mid360_driver/mid360_driver.hpp"
#include <cstddef>
#include <deque>
#include <mutex>
#include <utility>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unordered_map>

namespace mid360_driver {

    constexpr std::size_t MAX_PENDING_POINTS = 200000;
    constexpr std::size_t MAX_PENDING_IMU = 1000;
    // 单台雷达融合队列的帧数下限；实际上限按 merge_stale_timeout_s / 发布周期算
    // （见 Mid360DriverNode::merge_queue_limit_frames_），保证掉线判定窗口内的帧不会被提前丢掉。
    constexpr std::size_t MAX_MERGE_QUEUE_FRAMES = 8;
    // 融合队列帧数上限的绝对上限，防止参数取极端值时内存失控。
    constexpr std::size_t MAX_MERGE_QUEUE_FRAMES_HARD_LIMIT = 200;

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
        // 发布外部选好的一组 IMU（融合模式换源走这条），不经过 imu_to_publish。
        void publish_imu_messages(const std::vector<ImuMsg> &messages, const std::string &frame_id) const;
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
        std::size_t merge_queue_limit_frames_ = MAX_MERGE_QUEUE_FRAMES;
        // 一台雷达掉线时用另一台单独发帧（单雷达降级），否则 /lidar 会整段静默、导航直接断流。
        LidarHealthGate merge_cloud_gate_;
        // 参考雷达 IMU 掉线时切到另一台 IMU（换算到参考雷达系），否则 LIO 无输入。
        LidarHealthGate merge_imu_gate_;
        std::vector<ImuMsg> merge_imu_front_pending_;
        std::vector<ImuMsg> merge_imu_back_pending_;
        bool merge_front_cloud_online_ = false;
        bool merge_back_cloud_online_ = false;
        bool merge_imu_from_front_ = true;
        bool merge_imu_has_source_ = false;
        std::size_t merge_degrade_count_ = 0;
        std::size_t merge_imu_switch_count_ = 0;
        rclcpp::TimerBase::SharedPtr publish_imu_timer;

        static void stage_merge_points(std::vector<Point> &pending, const std::vector<Point> &points);
        static void stage_merge_imu(std::vector<ImuMsg> &pending, const ImuMsg &imu_msg);
        void enqueue_merge_frame(std::deque<MergeFrame> &queue, std::vector<Point> &&points);
        // 取出所有能配对的融合帧；不发布，发布放到锁外做。
        std::vector<std::vector<Point>> collect_merged_frames();
        static void drain_merge_queue(std::deque<MergeFrame> &queue, std::vector<std::pair<double, std::vector<Point>>> &out);
        // 单雷达模式：把还新鲜的那一路（最多两路）按时间顺序发出去，掉线那一路的积压直接丢。
        std::vector<std::vector<Point>> collect_single_lidar_frames(double now_s);
        // 双雷达/单雷达切换时打一条日志（含降级原因与累计次数）。
        void log_cloud_mode_change_locked(bool front_online, bool back_online, double now_s);
        // 一次点云 tick（调用方持锁）：入队本 tick 的帧，返回本 tick 要发布的帧。
        std::vector<std::vector<Point>> merge_pointcloud_tick_locked(double now_s);
        // 一次 IMU tick（调用方持锁）：按新鲜度选定 IMU 来源，返回本 tick 要发布的 IMU。
        std::vector<ImuMsg> merge_imu_tick_locked(double now_s);

    public:
        explicit Mid360DriverNode(const rclcpp::NodeOptions &options);
        ~Mid360DriverNode() override;
    };

}// namespace mid360_driver
