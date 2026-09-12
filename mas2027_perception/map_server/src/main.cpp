#include <rclcpp/rclcpp.hpp>
#include <map_server/map_server_node.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<map_server::MapServerNode>());
    rclcpp::shutdown();
    return 0;
}
