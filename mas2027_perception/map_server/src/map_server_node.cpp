#include <map_server/map_server_node.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/imgproc.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/time.h>

namespace map_server {

MapServerNode::MapServerNode(const rclcpp::NodeOptions& options) : Node("map_server", options) {
    const auto path = declare_parameter<std::string>("terrain_map_path", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    origin_x_ = declare_parameter<double>("origin_x", 0.0);
    origin_y_ = declare_parameter<double>("origin_y", 0.0);
    bypass_dynamic_obstacle_ = declare_parameter<bool>("bypass_dynamic_obstacle", false);

    map_utils::MapInflationParams inflation;
    inflation.full_cost_radius_m = declare_parameter<double>("full_cost_radius_m", 0.0);
    inflation.cutoff_radius_m = declare_parameter<double>("cutoff_radius_m", 0.25);
    inflation.decay_rate_per_m = declare_parameter<double>("decay_rate_per_m", 8.0);
    inflation.direction_non_body_magnitude_cap = declare_parameter<double>(
        "direction_non_body_magnitude_cap", 0.9);
    if (path.empty()) {
        throw std::invalid_argument("terrain_map_path must point to a converted msgpack map");
    }
    maps_ = map_utils::load_navigation_maps(path, inflation);

    roi_xy_min_ = declare_parameter<double>("local_map.roi_xy_radius_min", 0.4);
    roi_xy_max_ = declare_parameter<double>("local_map.roi_xy_radius_max", 8.0);
    roi_z_min_ = declare_parameter<double>("local_map.roi_z_min", -0.55);
    roi_z_max_ = declare_parameter<double>("local_map.roi_z_max", 0.2);
    dynamic_distance_threshold_ = declare_parameter<double>("local_map.distance_threshold", 0.2);
    low_obstacle_min_height_ = declare_parameter<double>("local_map.low_obstacle_min_height", 0.06);
    low_obstacle_match_distance_ = declare_parameter<double>("local_map.low_obstacle_match_distance", 0.07);
    voxel_size_ = declare_parameter<double>("local_map.voxel_size", 0.02);
    cloud_accumulate_frames_ = declare_parameter<int>("local_map.cloud_accumulate_frames", 3);
    min_points_per_cell_ = declare_parameter<int>("local_map.min_points_per_cell", 2);
    min_cluster_cells_ = declare_parameter<int>("local_map.min_cluster_cells", 3);
    dropout_hold_seconds_ = declare_parameter<double>("local_map.dropout_hold_seconds", 0.3);
    local_inflation_ = {
        .full_cost_radius_m = declare_parameter<double>("local_map.full_cost_radius_m", 0.2),
        .cutoff_radius_m = declare_parameter<double>("local_map.cutoff_radius_m", 0.4),
        .decay_rate_per_m = declare_parameter<double>("local_map.decay_rate_per_m", 16.0),
        .direction_non_body_magnitude_cap = 0.9,
        .resolution = maps_.resolution,
    };
    if (roi_xy_min_ < 0.0 || roi_xy_max_ <= roi_xy_min_ || roi_z_max_ <= roi_z_min_ ||
        dynamic_distance_threshold_ <= 0.0 || voxel_size_ <= 0.0 ||
        low_obstacle_min_height_ <= 0.0 || low_obstacle_match_distance_ <= 0.0 ||
        !std::isfinite(dropout_hold_seconds_) || dropout_hold_seconds_ < 0.0 ||
        cloud_accumulate_frames_ < 1 || min_points_per_cell_ < 1 || min_cluster_cells_ < 1) {
        throw std::invalid_argument("invalid local dynamic map parameters");
    }
    last_seen_dynamic_ = cv::Mat(maps_.height, maps_.width, CV_64FC1,
        cv::Scalar(-std::numeric_limits<double>::infinity()));
    const auto global_cloud_path = declare_parameter<std::string>("global_cloud_path", "");
    if (!bypass_dynamic_obstacle_) {
        if (global_cloud_path.empty()) {
            throw std::invalid_argument("global_cloud_path is required for reliable dynamic detection");
        }
        global_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile(global_cloud_path, *global_cloud_) < 0 || global_cloud_->empty()) {
            throw std::runtime_error("cannot load global static cloud: " + global_cloud_path);
        }
        global_tree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        global_tree_->setInputCloud(global_cloud_);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    }

    cost_grid_ = to_occupancy_grid(maps_.cost_map, maps_.width, maps_.height,
        maps_.resolution, origin_x_, origin_y_, frame_id_);
    direction_image_ = to_image(maps_.direction_map, "bgr8", frame_id_, now());
    const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    cost_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("cost_map", map_qos);
    direction_pub_ = create_publisher<sensor_msgs::msg::Image>("direction_map", map_qos);
    dynamic_cost_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("dynamic_cost_map", map_qos);
    if (!bypass_dynamic_obstacle_) {
        const auto cloud_topic = declare_parameter<std::string>("local_map.cloud_topic", "/cloud_registered");
        local_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { local_cloud_callback(msg); });
    }
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] { publish_maps(); });
    RCLCPP_INFO(get_logger(), "loaded %dx%d terrain map at %.3f m/px%s",
        maps_.width, maps_.height, maps_.resolution,
        bypass_dynamic_obstacle_ ? " (dynamic obstacle bypassed)" : "");
}

void MapServerNode::publish_maps() {
    const auto stamp = now();
    cost_grid_.header.stamp = stamp;
    direction_image_.header.stamp = stamp;
    cost_pub_->publish(cost_grid_);
    direction_pub_->publish(direction_image_);
    if (bypass_dynamic_obstacle_) {
        auto empty = cost_grid_;
        std::fill(empty.data.begin(), empty.data.end(), 0);
        empty.header.stamp = stamp;
        dynamic_cost_pub_->publish(empty);
    }

}

void MapServerNode::local_cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    geometry_msgs::msg::TransformStamped cloud_tf, robot_tf;
    try {
        const rclcpp::Time cloud_stamp(msg->header.stamp);
        cloud_tf = tf_buffer_->lookupTransform(
            frame_id_, msg->header.frame_id, cloud_stamp, tf2::durationFromSec(0.05));
        robot_tf = tf_buffer_->lookupTransform(
            frame_id_, "base_link", cloud_stamp, tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Dynamic map waiting for cloud/robot TF: %s", ex.what());
        return;
    }
    const auto& q = cloud_tf.transform.rotation;
    const Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
    const Eigen::Vector3d translation(cloud_tf.transform.translation.x,
        cloud_tf.transform.translation.y, cloud_tf.transform.translation.z);
    const Eigen::Vector3d robot(robot_tf.transform.translation.x,
        robot_tf.transform.translation.y, robot_tf.transform.translation.z);
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    try {
        sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x"), y(*msg, "y"), z(*msg, "z");
        for (; x != x.end(); ++x, ++y, ++z) {
            if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) continue;
            const Eigen::Vector3d p = rotation * Eigen::Vector3d(*x, *y, *z) + translation;
            const double radius = std::hypot(p.x() - robot.x(), p.y() - robot.y());
            const double relative_z = p.z() - robot.z();
            if (radius < roi_xy_min_ || radius > roi_xy_max_ ||
                relative_z < roi_z_min_ || relative_z > roi_z_max_) continue;
            cloud->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
        }
    } catch (const std::runtime_error& ex) {
        RCLCPP_WARN(get_logger(), "Invalid PointCloud2: %s", ex.what());
        return;
    }
    cloud_queue_.push_front(cloud);
    while (cloud_queue_.size() > static_cast<size_t>(cloud_accumulate_frames_)) cloud_queue_.pop_back();
    auto accumulated = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& frame : cloud_queue_) *accumulated += *frame;
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(accumulated);
    voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    pcl::PointCloud<pcl::PointXYZ> downsampled;
    voxel.filter(downsampled);

    // Sparse or slightly misregistered prior PCDs must not turn the dominant floor into obstacles.
    // Only remove a broad, near-horizontal plane below the robot; retain raised objects.
    std::vector<bool> ground(downsampled.size(), false);
    bool ground_found = false;
    double ground_height = 0.0;
    if (downsampled.size() >= 30) {
        pcl::SACSegmentation<pcl::PointXYZ> segmenter;
        segmenter.setOptimizeCoefficients(true);
        segmenter.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        segmenter.setMethodType(pcl::SAC_RANSAC);
        segmenter.setAxis(Eigen::Vector3f::UnitZ());
        segmenter.setEpsAngle(0.35);
        segmenter.setDistanceThreshold(0.04);
        segmenter.setMaxIterations(100);
        segmenter.setInputCloud(downsampled.makeShared());
        pcl::PointIndices inliers;
        pcl::ModelCoefficients coefficients;
        segmenter.segment(inliers, coefficients);
        if (inliers.indices.size() >= 30 && inliers.indices.size() * 5 >= downsampled.size()) {
            double height = 0.0;
            for (int index : inliers.indices) height += downsampled.points[index].z;
            height /= inliers.indices.size();
            if (height <= robot.z() - 0.04) {
                ground_found = true;
                ground_height = height;
                for (int index : inliers.indices) ground[index] = true;
            }
        }
    }

    cv::Mat counts = cv::Mat::zeros(maps_.height, maps_.width, CV_32SC1);
    std::vector<int> nearest(1);
    std::vector<float> distances(1);
    for (size_t i = 0; i < downsampled.size(); ++i) {
        if (ground[i]) continue;
        const auto& point = downsampled.points[i];
        // A floor PCD point can be within 0.2 m of a real low obstacle. Match raised
        // near-floor points more tightly, but keep the wider tolerance elsewhere.
        const double match_distance = ground_found &&
            point.z > ground_height + low_obstacle_min_height_ &&
            point.z < ground_height + 0.35 ?
            std::min(dynamic_distance_threshold_, low_obstacle_match_distance_) :
            dynamic_distance_threshold_;
        if (global_tree_->nearestKSearch(point, 1, nearest, distances) > 0 &&
            distances[0] <= match_distance * match_distance) continue;
        const int ix = static_cast<int>(std::floor((point.x - origin_x_) / maps_.resolution));
        const int iy = static_cast<int>(std::floor((point.y - origin_y_) / maps_.resolution));
        if (ix >= 0 && iy >= 0 && ix < maps_.width && iy < maps_.height) {
            ++counts.at<int>(iy, ix);
        }
    }
    cv::Mat mask = cv::Mat::zeros(maps_.height, maps_.width, CV_8UC1);
    for (int y = 0; y < maps_.height; ++y) {
        for (int x = 0; x < maps_.width; ++x) {
            if (counts.at<int>(y, x) >= min_points_per_cell_) mask.at<uint8_t>(y, x) = 255;
        }
    }
    cv::Mat labels, stats, centroids;
    const int components = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8);
    const double steady_now = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    for (int y = 0; y < maps_.height; ++y) {
        for (int x = 0; x < maps_.width; ++x) {
            const int label = labels.at<int>(y, x);
            if (label > 0 && stats.at<int>(label, cv::CC_STAT_AREA) < min_cluster_cells_) {
                mask.at<uint8_t>(y, x) = 0;
            }
            auto& cell = mask.at<uint8_t>(y, x);
            auto& last_seen = last_seen_dynamic_.at<double>(y, x);
            if (cell) last_seen = steady_now;
            else if (steady_now - last_seen < dropout_hold_seconds_) cell = 255;
        }
    }

    const cv::Mat current = map_utils::inflate_cost_map_bounded(mask, local_inflation_);
    auto dynamic = to_occupancy_grid(current, maps_.width, maps_.height,
        maps_.resolution, origin_x_, origin_y_, frame_id_);
    dynamic.header.stamp = msg->header.stamp;
    dynamic_cost_pub_->publish(dynamic);
}

nav_msgs::msg::OccupancyGrid MapServerNode::to_occupancy_grid(
    const cv::Mat& map, const int width, const int height, const double resolution,
    const double origin_x, const double origin_y, const std::string& frame_id) {
    nav_msgs::msg::OccupancyGrid result;
    result.header.frame_id = frame_id;
    result.info.width = static_cast<uint32_t>(width);
    result.info.height = static_cast<uint32_t>(height);
    result.info.resolution = static_cast<float>(resolution);
    result.info.origin.position.x = origin_x;
    result.info.origin.position.y = origin_y;
    result.info.origin.orientation.w = 1.0;
    result.data.resize(static_cast<size_t>(width) * static_cast<size_t>(height));
    for (size_t i = 0; i < result.data.size(); ++i) {
        result.data[i] = static_cast<int8_t>(std::min(100, (static_cast<int>(map.data[i]) * 100) / 255));
    }
    return result;
}

sensor_msgs::msg::Image MapServerNode::to_image(
    const cv::Mat& map, const std::string& encoding,
    const std::string& frame_id, const rclcpp::Time& stamp) {
    sensor_msgs::msg::Image result;
    result.header.stamp = stamp;
    result.header.frame_id = frame_id;
    result.height = static_cast<uint32_t>(map.rows);
    result.width = static_cast<uint32_t>(map.cols);
    result.encoding = encoding;
    result.is_bigendian = false;
    result.step = static_cast<uint32_t>(map.step);
    result.data.assign(map.datastart, map.dataend);
    return result;
}

}  // namespace map_server

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(map_server::MapServerNode)
