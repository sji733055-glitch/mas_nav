#include <map_server/map_server_node.hpp>

#include <algorithm>
#include <chrono>
#include <stdexcept>

namespace map_server {

MapServerNode::MapServerNode(const rclcpp::NodeOptions& options) : Node("map_server", options) {
    const auto path = declare_parameter<std::string>("terrain_map_path", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    origin_x_ = declare_parameter<double>("origin_x", 0.0);
    origin_y_ = declare_parameter<double>("origin_y", 0.0);
    bypass_dynamic_obstacle_ = declare_parameter<bool>("bypass_dynamic_obstacle", true);

    map_utils::MapInflationParams inflation;
    inflation.full_cost_radius_m = declare_parameter<double>("full_cost_radius_m", 0.20);
    inflation.cutoff_radius_m = declare_parameter<double>("cutoff_radius_m", 0.60);
    inflation.decay_rate_per_m = declare_parameter<double>("decay_rate_per_m", 4.0);
    inflation.direction_non_body_magnitude_cap = declare_parameter<double>(
        "direction_non_body_magnitude_cap", 0.9);
    if (path.empty()) {
        throw std::invalid_argument("terrain_map_path must point to a converted msgpack map");
    }
    maps_ = map_utils::load_navigation_maps(path, inflation);

    cost_grid_ = to_occupancy_grid(maps_.cost_map, maps_.width, maps_.height,
        maps_.resolution, origin_x_, origin_y_, frame_id_);
    direction_image_ = to_image(maps_.direction_map, "bgr8", frame_id_, now());
    const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    cost_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("cost_map", map_qos);
    direction_pub_ = create_publisher<sensor_msgs::msg::Image>("direction_map", map_qos);
    cost_maps_pub_ = create_publisher<interfaces::msg::CostMaps>("cost_maps", map_qos);
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

    interfaces::msg::CostMaps msg;
    msg.prediction_dt = 0.0;
    msg.maps.push_back(cost_grid_);
    cost_maps_pub_->publish(msg);
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
