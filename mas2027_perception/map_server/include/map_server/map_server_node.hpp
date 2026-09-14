#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <deque>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <map_server/utils.hpp>

namespace map_server {

class MapServerNode final : public rclcpp::Node {
public:
    explicit MapServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void publish_maps();
    void local_cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
    static nav_msgs::msg::OccupancyGrid to_occupancy_grid(
        const cv::Mat& map, int width, int height, double resolution,
        double origin_x, double origin_y, const std::string& frame_id);
    static sensor_msgs::msg::Image to_image(
        const cv::Mat& map, const std::string& encoding,
        const std::string& frame_id, const rclcpp::Time& stamp);

    map_utils::NavigationMapData maps_;
    double origin_x_{};
    double origin_y_{};
    std::string frame_id_;
    bool bypass_dynamic_obstacle_{};
    double roi_xy_min_{0.4}, roi_xy_max_{8.0}, roi_z_min_{-0.55}, roi_z_max_{0.2};
    double dynamic_distance_threshold_{0.2}, voxel_size_{0.02};
    double low_obstacle_min_height_{0.06}, low_obstacle_match_distance_{0.07};
    double dropout_hold_seconds_{0.3};
    int cloud_accumulate_frames_{3}, min_points_per_cell_{2}, min_cluster_cells_{3};
    cv::Mat last_seen_dynamic_;
    map_utils::MapInflationParams local_inflation_{};
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr global_tree_;
    std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_queue_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    nav_msgs::msg::OccupancyGrid cost_grid_;
    sensor_msgs::msg::Image direction_image_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr direction_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr dynamic_cost_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_cloud_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace map_server
