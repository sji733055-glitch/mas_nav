/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "mid360_driver/mid360_driver_node.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mid360_driver {

    namespace {

        struct RigidTransform {
            float r00, r01, r02;
            float r10, r11, r12;
            float r20, r21, r22;
            float tx, ty, tz;
        };

        RigidTransform make_transform(const std::vector<double> &xyz_rpy) {
            const double roll = xyz_rpy[3];
            const double pitch = xyz_rpy[4];
            const double yaw = xyz_rpy[5];
            const double cr = std::cos(roll);
            const double sr = std::sin(roll);
            const double cp = std::cos(pitch);
            const double sp = std::sin(pitch);
            const double cy = std::cos(yaw);
            const double sy = std::sin(yaw);

            return {
                static_cast<float>(cp * cy),
                static_cast<float>(sr * sp * cy - cr * sy),
                static_cast<float>(cr * sp * cy + sr * sy),
                static_cast<float>(cp * sy),
                static_cast<float>(sr * sp * sy + cr * cy),
                static_cast<float>(cr * sp * sy - sr * cy),
                static_cast<float>(-sp),
                static_cast<float>(sr * cp),
                static_cast<float>(cr * cp),
                static_cast<float>(xyz_rpy[0]),
                static_cast<float>(xyz_rpy[1]),
                static_cast<float>(xyz_rpy[2])
            };
        }

        std::vector<Point> transform_points(const std::vector<Point> &points, const RigidTransform &tf) {
            std::vector<Point> transformed;
            transformed.reserve(points.size());
            for (const auto &point : points) {
                Point output = point;
                output.x = tf.r00 * point.x + tf.r01 * point.y + tf.r02 * point.z + tf.tx;
                output.y = tf.r10 * point.x + tf.r11 * point.y + tf.r12 * point.z + tf.ty;
                output.z = tf.r20 * point.x + tf.r21 * point.y + tf.r22 * point.z + tf.tz;
                transformed.push_back(output);
            }
            return transformed;
        }

    } // namespace

    void LidarPublisher::configure_robustness(const DriverRobustnessConfig &config) {
        robustness_config = config;
    }

    void LidarPublisher::ensure_initialized(rclcpp::Node &node, const std::string &lidar_topic, const std::string &imu_topic) {
        if (!is_initialized) {
            pointcloud_publisher = node.create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic, 1000);
            imu_publisher = node.create_publisher<sensor_msgs::msg::Imu>(imu_topic, 1000);
            is_initialized = true;
        }
    }

    void LidarPublisher::ensure_initialized(rclcpp::Node &node, const std::string &lidar_topic, const std::string &imu_topic, const asio::ip::address &lidar_ip) {
        if (!is_initialized) {
            auto lidar_ip_bytes = lidar_ip.to_v4().to_bytes();
            std::string lidar_ip_str;
            lidar_ip_str.push_back('_');
            lidar_ip_str.append(std::to_string(static_cast<int>(lidar_ip_bytes[0])));
            lidar_ip_str.push_back('_');
            lidar_ip_str.append(std::to_string(static_cast<int>(lidar_ip_bytes[1])));
            lidar_ip_str.push_back('_');
            lidar_ip_str.append(std::to_string(static_cast<int>(lidar_ip_bytes[2])));
            lidar_ip_str.push_back('_');
            lidar_ip_str.append(std::to_string(static_cast<int>(lidar_ip_bytes[3])));
            pointcloud_publisher = node.create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic + lidar_ip_str, 1000);
            imu_publisher = node.create_publisher<sensor_msgs::msg::Imu>(imu_topic + lidar_ip_str, 1000);
            is_initialized = true;
        }
    }

    void LidarPublisher::on_receive_pointcloud(const std::vector<Point> &points) {
        if (points_wait_to_publish.size() + points.size() > MAX_PENDING_POINTS) {
            points_wait_to_publish.clear();
        }
        points_wait_to_publish.reserve(points_wait_to_publish.size() + points.size());
        std::copy(points.begin(), points.end(), std::back_inserter(points_wait_to_publish));
    }

    void LidarPublisher::on_receive_imu(const ImuMsg &imu_msg) {
        if (imu_wait_to_publish.size() >= MAX_PENDING_IMU) {
            imu_wait_to_publish.clear();
        }
        imu_wait_to_publish.push_back(imu_msg);
    }

    void LidarPublisher::prepare_pointcloud_to_publish() {
        std::swap(points_wait_to_publish, points_to_publish);
        points_wait_to_publish.clear();
    }

    void LidarPublisher::prepare_imu_to_publish() {
        std::swap(imu_wait_to_publish, imu_to_publish);
        imu_wait_to_publish.clear();
    }

    void LidarPublisher::publish_pointcloud(const std::string &frame_id) const {
        publish_points(points_to_publish, frame_id);
    }

    void LidarPublisher::publish_points(const std::vector<Point> &points, const std::string &frame_id) const {
        if (points.empty()) {
            return;
        }
        double min_timestamp = std::numeric_limits<double>::max();
        double max_timestamp = std::numeric_limits<double>::lowest();
        for (const auto &point: points) {
            if (std::isfinite(point.timestamp)) {
                min_timestamp = std::min(min_timestamp, point.timestamp);
                max_timestamp = std::max(max_timestamp, point.timestamp);
            }
        }
        if (!std::isfinite(min_timestamp) || !std::isfinite(max_timestamp)) {
            return;
        }
        const double frame_time_span = max_timestamp - min_timestamp;
        if (frame_time_span < 0.0 || frame_time_span > robustness_config.max_packet_time_span * 2.0) {
            RCLCPP_WARN(
                rclcpp::get_logger("mid360_driver"),
                "drop point cloud with invalid timestamp span: min=%.6f max=%.6f span=%.6f points=%zu",
                min_timestamp,
                max_timestamp,
                frame_time_span,
                points.size()
            );
            return;
        }

        std::vector<const Point *> valid_points;
        valid_points.reserve(points.size());
        const double max_range2 = robustness_config.max_point_range * robustness_config.max_point_range;
        for (const auto &point: points) {
            const double range2 = point.x * point.x + point.y * point.y + point.z * point.z;
            if (std::isfinite(point.timestamp)
                && std::isfinite(point.x)
                && std::isfinite(point.y)
                && std::isfinite(point.z)
                && std::isfinite(point.intensity)
                && range2 <= max_range2) {
                valid_points.push_back(&point);
            }
        }
        if (valid_points.empty()) {
            return;
        }

        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp.sec = static_cast<int32_t>(std::floor(min_timestamp));
        msg.header.stamp.nanosec = static_cast<uint32_t>((min_timestamp - msg.header.stamp.sec) * 1e9);
        msg.header.frame_id = frame_id;
        msg.width = static_cast<uint32_t>(valid_points.size());
        msg.height = 1;
        msg.fields.reserve(4);
        sensor_msgs::msg::PointField field;
        field.name = "x";
        field.offset = 0;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        msg.fields.push_back(field);
        field.name = "y";
        field.offset = 4;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        msg.fields.push_back(field);
        field.name = "z";
        field.offset = 8;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        msg.fields.push_back(field);
        field.name = "intensity";
        field.offset = 12;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        msg.fields.push_back(field);
        field.name = "timestamp";
        field.offset = 16;
        field.datatype = sensor_msgs::msg::PointField::FLOAT64;
        field.count = 1;
        msg.fields.push_back(field);
        msg.is_bigendian = false;
        msg.point_step = 24;
        msg.row_step = msg.width * msg.point_step;
        msg.data.resize(msg.row_step * msg.height);
        auto* pointer = reinterpret_cast<float*>(msg.data.data());
        for (const auto *point: valid_points) {
            *pointer = point->x;
            ++pointer;
            *pointer = point->y;
            ++pointer;
            *pointer = point->z;
            ++pointer;
            *pointer = point->intensity;
            ++pointer;
            *reinterpret_cast<double *>(pointer) = point->timestamp;
            pointer += 2;
        }
        msg.is_dense = true;
        pointcloud_publisher->publish(msg);
    }

    void LidarPublisher::publish_imu(const std::string &frame_id) const {
        for (const auto &imu: imu_to_publish) {
            sensor_msgs::msg::Imu msg;
            msg.header.stamp.sec = static_cast<int32_t>(std::floor(imu.timestamp));
            msg.header.stamp.nanosec = static_cast<uint32_t>((imu.timestamp - msg.header.stamp.sec) * 1e9);
            msg.header.frame_id = frame_id;
            msg.angular_velocity.x = imu.angular_velocity_x;
            msg.angular_velocity.y = imu.angular_velocity_y;
            msg.angular_velocity.z = imu.angular_velocity_z;
            msg.linear_acceleration.x = imu.linear_acceleration_x;
            msg.linear_acceleration.y = imu.linear_acceleration_y;
            msg.linear_acceleration.z = imu.linear_acceleration_z;
            imu_publisher->publish(msg);
        }
    }

    void Mid360DriverNode::stage_merge_points(std::vector<Point> &pending, const std::vector<Point> &points) {
        if (pending.size() + points.size() > MAX_PENDING_POINTS) {
            pending.clear();
        }
        pending.reserve(pending.size() + points.size());
        std::copy(points.begin(), points.end(), std::back_inserter(pending));
    }

    void Mid360DriverNode::enqueue_merge_frame(std::deque<MergeFrame> &queue, std::vector<Point> &&points) {
        if (points.empty()) {
            return;
        }
        double base_timestamp = std::numeric_limits<double>::max();
        for (const auto &point : points) {
            if (std::isfinite(point.timestamp)) {
                base_timestamp = std::min(base_timestamp, point.timestamp);
            }
        }
        if (!std::isfinite(base_timestamp)) {
            return;
        }
        if (queue.size() >= MAX_MERGE_QUEUE_FRAMES) {
            queue.pop_front();
        }
        queue.push_back(MergeFrame{base_timestamp, std::move(points)});
    }

    std::vector<std::vector<Point>> Mid360DriverNode::collect_merged_frames() {
        std::vector<std::vector<Point>> merged_frames;
        // 对齐 navi InternalLidarMerger：两队都非空才配对，禁止发半帧；超窗就丢更旧的队头。
        while (!merge_front_queue_.empty() && !merge_back_queue_.empty()) {
            const double front_base = merge_front_queue_.front().base_timestamp;
            const double back_base = merge_back_queue_.front().base_timestamp;
            if (std::abs(front_base - back_base) > merge_max_interval_s_) {
                const bool drop_front = front_base <= back_base;
                if ((merge_drop_count_++ % 20) == 0) {
                    RCLCPP_WARN(
                        get_logger(),
                        "lidar merge dropped stale %s frame: front=%.6f back=%.6f dt=%.3f ms window=%.3f ms",
                        drop_front ? "front" : "back",
                        front_base,
                        back_base,
                        std::abs(front_base - back_base) * 1e3,
                        merge_max_interval_s_ * 1e3
                    );
                }
                (drop_front ? merge_front_queue_ : merge_back_queue_).pop_front();
                continue;
            }

            MergeFrame front = std::move(merge_front_queue_.front());
            merge_front_queue_.pop_front();
            MergeFrame back = std::move(merge_back_queue_.front());
            merge_back_queue_.pop_front();

            // 借 HERO 的双指针归并：两路各自已按时间递增，归并后整帧仍按绝对时间有序，
            // 避免 LIO Preprocess 的 last_timestamp_lidar 水位线把后半帧整段砍掉。
            std::vector<Point> merged(front.points.size() + back.points.size());
            std::merge(
                front.points.begin(), front.points.end(),
                back.points.begin(), back.points.end(),
                merged.begin(),
                [](const Point &lhs, const Point &rhs) { return lhs.timestamp < rhs.timestamp; }
            );
            merged_frames.push_back(std::move(merged));
        }
        const bool front_starved = merge_back_queue_.size() >= 3 && merge_front_queue_.empty();
        const bool back_starved = merge_front_queue_.size() >= 3 && merge_back_queue_.empty();
        if ((front_starved || back_starved) && (merge_starve_count_++ % 20) == 0) {
            RCLCPP_WARN(
                get_logger(),
                "lidar merge is waiting for the %s lidar: front_queue=%zu back_queue=%zu",
                front_starved ? "front" : "back",
                merge_front_queue_.size(),
                merge_back_queue_.size()
            );
        }
        return merged_frames;
    }

    Mid360DriverNode::Mid360DriverNode(const rclcpp::NodeOptions &options) : Node("mid360_driver", options) {
        std::string lidar_topic = declare_parameter<std::string>("lidar_topic");
        std::string lidar_frame = declare_parameter<std::string>("lidar_frame");
        std::string imu_topic = declare_parameter<std::string>("imu_topic");
        std::string imu_frame = declare_parameter<std::string>("imu_frame");
        std::string host_ip = declare_parameter<std::string>("host_ip");
        double lidar_publish_time_interval = declare_parameter<double>("lidar_publish_time_interval");
        bool is_topic_name_with_lidar_ip = declare_parameter<bool>("is_topic_name_with_lidar_ip");
        const std::string front_lidar_ip = declare_parameter<std::string>("front_lidar_ip", "192.168.1.136");
        const std::string back_lidar_ip = declare_parameter<std::string>("back_lidar_ip", "192.168.1.193");
        const std::string front_lidar_frame = declare_parameter<std::string>("front_lidar_frame", lidar_frame);
        const std::string back_lidar_frame = declare_parameter<std::string>("back_lidar_frame", "lidar_back_link");
        const bool enable_lidar_merge = declare_parameter<bool>("enable_lidar_merge", false);
        const std::string merge_front_ip = declare_parameter<std::string>("merge_front_ip", "192.168.1.136");
        const std::string merge_back_ip = declare_parameter<std::string>("merge_back_ip", "192.168.1.193");
        const double merge_max_interval_ms = declare_parameter<double>("merge_max_interval_ms", 5.0);
        const std::vector<double> merge_extrinsic = declare_parameter<std::vector<double>>(
            "merge_extrinsic_back_to_front",
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
        );
        asio::ip::address front_lidar_address;
        asio::ip::address back_lidar_address;
        asio::ip::address merge_front_address;
        asio::ip::address merge_back_address;
        RigidTransform merge_back_to_front{};
        if (enable_lidar_merge && is_topic_name_with_lidar_ip) {
            throw std::invalid_argument(
                "enable_lidar_merge and is_topic_name_with_lidar_ip cannot both be true"
            );
        }
        if (is_topic_name_with_lidar_ip) {
            front_lidar_address = asio::ip::make_address(front_lidar_ip);
            back_lidar_address = asio::ip::make_address(back_lidar_ip);
            if (front_lidar_address == back_lidar_address) {
                throw std::invalid_argument("front_lidar_ip and back_lidar_ip must be distinct");
            }
        }
        if (enable_lidar_merge) {
            merge_front_address = asio::ip::make_address(merge_front_ip);
            merge_back_address = asio::ip::make_address(merge_back_ip);
            if (merge_front_address == merge_back_address) {
                throw std::invalid_argument("merge_front_ip and merge_back_ip must be distinct");
            }
            if (merge_extrinsic.size() != 6
                || !std::all_of(merge_extrinsic.begin(), merge_extrinsic.end(), [](double value) {
                    return std::isfinite(value);
                })) {
                throw std::invalid_argument(
                    "merge_extrinsic_back_to_front must contain 6 finite values: x, y, z, roll, pitch, yaw"
                );
            }
            if (!(merge_max_interval_ms > 0.0) || !std::isfinite(merge_max_interval_ms)) {
                throw std::invalid_argument("merge_max_interval_ms must be a positive finite number");
            }
            merge_max_interval_s_ = merge_max_interval_ms * 1e-3;
            merge_back_to_front = make_transform(merge_extrinsic);
            RCLCPP_INFO(
                get_logger(),
                "merging lidar %s into %s in frame '%s', pairing window %.1f ms",
                merge_back_ip.c_str(),
                merge_front_ip.c_str(),
                lidar_frame.c_str(),
                merge_max_interval_ms
            );
        }
        DriverRobustnessConfig robustness_config;
        robustness_config.validate_crc = declare_parameter<bool>("validate_crc");
        robustness_config.max_packet_time_jump = declare_parameter<double>("max_packet_time_jump");
        robustness_config.max_packet_time_span = declare_parameter<double>("max_packet_time_span");
        robustness_config.max_point_range = declare_parameter<double>("max_point_range");
        robustness_config.max_imu_acc = declare_parameter<double>("max_imu_acc");
        robustness_config.max_imu_gyro = declare_parameter<double>("max_imu_gyro");
        robustness_config.min_drop_log_interval = declare_parameter<double>("min_drop_log_interval");
        if (!is_topic_name_with_lidar_ip) {
            lidar_publisher.configure_robustness(robustness_config);
            lidar_publisher.ensure_initialized(*this, lidar_topic, imu_topic);
        }
        mid360_driver = std::make_unique<mid360_driver::Mid360Driver>(
                io_context,
                asio::ip::make_address(host_ip),
                robustness_config,
                [this, is_topic_name_with_lidar_ip, enable_lidar_merge, merge_front_address, merge_back_address, merge_back_to_front, robustness_config](const asio::ip::address &lidar_ip, const std::vector<Point> &points) {
                    std::lock_guard lock(multi_lidar_mutex_);
                    if (enable_lidar_merge) {
                        // 只暂存，等定时器切帧后按时间配对；后雷达在入队前就变到前雷达系。
                        if (lidar_ip == merge_front_address) {
                            stage_merge_points(merge_front_pending_, points);
                        } else if (lidar_ip == merge_back_address) {
                            stage_merge_points(merge_back_pending_, transform_points(points, merge_back_to_front));
                        }
                    } else if (is_topic_name_with_lidar_ip) {
                        auto iter = multi_lidar_publishers.try_emplace(lidar_ip).first;
                        iter->second.configure_robustness(robustness_config);
                        iter->second.on_receive_pointcloud(points);
                    } else {
                        lidar_publisher.on_receive_pointcloud(points);
                    }
                },
                [this, is_topic_name_with_lidar_ip, enable_lidar_merge, merge_front_address, robustness_config](const asio::ip::address &lidar_ip, const ImuMsg &imu_msg) {
                    std::lock_guard lock(multi_lidar_mutex_);
                    if (enable_lidar_merge) {
                        if (lidar_ip == merge_front_address) {
                            lidar_publisher.on_receive_imu(imu_msg);
                        }
                    } else if (is_topic_name_with_lidar_ip) {
                        auto iter = multi_lidar_publishers.try_emplace(lidar_ip).first;
                        iter->second.configure_robustness(robustness_config);
                        iter->second.on_receive_imu(imu_msg);
                    } else {
                        lidar_publisher.on_receive_imu(imu_msg);
                    }
                });
        if (is_topic_name_with_lidar_ip) {
            publish_pointcloud_timer = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(100), [this, lidar_topic, imu_topic, lidar_frame, front_lidar_address, back_lidar_address, front_lidar_frame, back_lidar_frame]() {
                std::vector<std::pair<asio::ip::address, LidarPublisher*>> snapshot;
                {
                    std::lock_guard lock(multi_lidar_mutex_);
                    snapshot.reserve(multi_lidar_publishers.size());
                    for (auto &[lidar_ip, publisher] : multi_lidar_publishers) {
                        publisher.prepare_pointcloud_to_publish();
                        snapshot.emplace_back(lidar_ip, &publisher);
                    }
                }
                for (auto &[lidar_ip, publisher] : snapshot) {
                    publisher->ensure_initialized(*this, lidar_topic, imu_topic, lidar_ip);
                    const std::string &pointcloud_frame = lidar_ip == back_lidar_address
                        ? back_lidar_frame
                        : (lidar_ip == front_lidar_address ? front_lidar_frame : lidar_frame);
                    publisher->publish_pointcloud(pointcloud_frame);
                }
            });
            publish_imu_timer = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(1), [this, lidar_topic, imu_topic, imu_frame]() {
                std::vector<std::pair<asio::ip::address, LidarPublisher*>> snapshot;
                {
                    std::lock_guard lock(multi_lidar_mutex_);
                    snapshot.reserve(multi_lidar_publishers.size());
                    for (auto &[lidar_ip, publisher] : multi_lidar_publishers) {
                        publisher.prepare_imu_to_publish();
                        snapshot.emplace_back(lidar_ip, &publisher);
                    }
                }
                for (auto &[lidar_ip, publisher] : snapshot) {
                    publisher->ensure_initialized(*this, lidar_topic, imu_topic, lidar_ip);
                    publisher->publish_imu(imu_frame);
                }
            });
        } else if (enable_lidar_merge) {
            publish_pointcloud_timer = rclcpp::create_timer(this, get_clock(), std::chrono::duration<double, std::ratio<1, 1>>(lidar_publish_time_interval), [this, lidar_frame]() {
                std::vector<std::vector<Point>> merged_frames;
                {
                    std::lock_guard lock(multi_lidar_mutex_);
                    // 两路在同一个 tick 切帧，随后按 base_timestamp 配对。
                    enqueue_merge_frame(merge_front_queue_, std::move(merge_front_pending_));
                    merge_front_pending_.clear();
                    enqueue_merge_frame(merge_back_queue_, std::move(merge_back_pending_));
                    merge_back_pending_.clear();
                    merged_frames = collect_merged_frames();
                }
                for (const auto &merged : merged_frames) {
                    lidar_publisher.publish_points(merged, lidar_frame);
                }
            });
            publish_imu_timer = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(1), [this, imu_frame]() {
                {
                    std::lock_guard lock(multi_lidar_mutex_);
                    lidar_publisher.prepare_imu_to_publish();
                }
                lidar_publisher.publish_imu(imu_frame);
            });
        } else {
            publish_pointcloud_timer = rclcpp::create_timer(this, get_clock(), std::chrono::duration<double, std::ratio<1, 1>>(lidar_publish_time_interval), [this, lidar_frame]() {
                {
                    std::lock_guard lock(multi_lidar_mutex_);
                    lidar_publisher.prepare_pointcloud_to_publish();
                }
                lidar_publisher.publish_pointcloud(lidar_frame);
            });
            publish_imu_timer = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(1), [this, imu_frame]() {
                {
                    std::lock_guard lock(multi_lidar_mutex_);
                    lidar_publisher.prepare_imu_to_publish();
                }
                lidar_publisher.publish_imu(imu_frame);
            });
        }
        io_thread = std::thread([this]() {
            io_context.run();
        });
    }

    Mid360DriverNode::~Mid360DriverNode() {
        if (mid360_driver) {
            mid360_driver->stop();
        }
        io_context.stop();
        io_thread.join();
    }

}// namespace mid360_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(mid360_driver::Mid360DriverNode)
