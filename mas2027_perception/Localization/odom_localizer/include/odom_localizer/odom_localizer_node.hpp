#ifndef ODOM_LOCALIZER__ODOM_LOCALIZER_NODE_HPP_
#define ODOM_LOCALIZER__ODOM_LOCALIZER_NODE_HPP_

#include <chrono>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/registration.hpp>

namespace odom_localizer {

class EMAIsometry {
public:
  explicit EMAIsometry(double keep_ratio);
  void initialize(const Eigen::Isometry3d & value);
  void update(const Eigen::Isometry3d & value);
  Eigen::Isometry3d value() const;
  bool initialized() const;

private:
  double keep_ratio_;
  bool initialized_{false};
  Eigen::Isometry3d value_{Eigen::Isometry3d::Identity()};
};

class OdomLocalizerNode : public rclcpp::Node {
public:
  explicit OdomLocalizerNode(const rclcpp::NodeOptions & options);

private:
  struct RegistrationMetrics {
    bool converged = false;
    size_t iterations = 0;
    size_t num_inliers = 0;
    double normalized_error = std::numeric_limits<double>::infinity();
    double overlap_ratio = 0.0;
    double min_information_eigenvalue = 0.0;
    double max_information_eigenvalue = 0.0;
    double information_condition_number = std::numeric_limits<double>::infinity();
  };

  struct RegistrationAssessment {
    bool accepted = false;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    RegistrationMetrics metrics;
    std::string reason;
  };

  struct PreparedSource {
    small_gicp::PointCloud::Ptr cloud;
    size_t raw_points = 0;
    size_t downsampled_points = 0;
    std::string label;
  };

  void load_parameters();
  void load_map();
  void create_interfaces();

  void publish_timer_callback();
  void registration_timer_callback();
  void registered_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  Eigen::Isometry3d get_current_map_to_odom() const;
  void initialize_transform(const Eigen::Isometry3d & transform, const char * reason);
  std::optional<PreparedSource> prepare_source() const;
  RegistrationAssessment align_to_map(const PreparedSource & source) const;
  bool evaluate_registration_result(
    const small_gicp::RegistrationResult & result,
    size_t source_points,
    RegistrationMetrics & metrics,
    std::string & reason) const;
  bool apply_registration_update(const Eigen::Isometry3d & transform);
  void publish_transform();
  void maybe_publish_prior_cloud();

  small_gicp::PointCloud::Ptr convert_pointcloud2(
    const sensor_msgs::msg::PointCloud2::SharedPtr & msg) const;
  sensor_msgs::msg::PointCloud2 to_pointcloud2(
    const small_gicp::PointCloud & cloud, const std::string & frame) const;

  bool enable_debug_{false};
  int num_threads_{2};
  double publish_rate_hz_{20.0};

  std::string map_frame_{"map"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  std::string registered_cloud_topic_{"/cloud_registered"};
  std::string initialpose_topic_{"/initialpose"};

  std::string prior_pcd_file_;
  double map_downsample_resolution_{0.05};
  int map_covariance_neighbors_{20};
  bool publish_prior_cloud_{false};

  size_t registered_window_frames_{6};
  size_t min_points_raw_{800};
  size_t min_points_downsampled_{400};
  double source_downsample_resolution_{0.05};
  int source_covariance_neighbors_{20};

  Eigen::Isometry3d initial_transform_{Eigen::Isometry3d::Identity()};
  double bootstrap_duration_s_{10.0};

  double registration_period_s_{1.0};
  double bootstrap_max_correspondence_distance_{2.0};
  double locked_max_correspondence_distance_{0.75};
  int max_iterations_{32};
  double translation_eps_{0.001};
  double rotation_eps_{0.001};

  double max_normalized_error_{0.35};
  double min_overlap_ratio_{0.45};
  double min_information_eigenvalue_{1.0e2};
  double max_information_condition_number_{1.0e7};

  double ema_ratio_{0.6};
  double max_translation_step_{0.3};
  double max_rotation_step_{0.15};
  bool publish_tf_direct_{true};
  std::string map_to_odom_topic_{"/tf_maintainer/map_to_odom"};

  small_gicp::PointCloud::Ptr map_cloud_;
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> map_kd_tree_;
  std::deque<small_gicp::PointCloud::Ptr> registered_cloud_window_;
  std::unique_ptr<EMAIsometry> map_to_odom_filter_;
  std::optional<Eigen::Isometry3d> last_accepted_registration_transform_;
  bool has_successful_registration_{false};
  std::chrono::steady_clock::time_point initialize_time_;

  mutable std::mutex transform_state_mutex_;
  mutable std::mutex source_data_mutex_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr registered_cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr prior_cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr map_to_odom_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr registration_timer_;
};

}  // namespace odom_localizer

#endif
