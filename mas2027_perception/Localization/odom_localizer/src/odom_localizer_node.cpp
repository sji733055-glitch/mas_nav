#include "odom_localizer/odom_localizer_node.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

namespace odom_localizer {
using small_gicp::GICPFactor;
using small_gicp::KdTree;
using small_gicp::KdTreeBuilderOMP;
using small_gicp::ParallelReductionOMP;
using small_gicp::PointCloud;
using small_gicp::Registration;
using small_gicp::RegistrationResult;

namespace {

Eigen::Isometry3d transform_from_xyz_xyzw(const std::vector<double> & values, const std::string & name)
{
  if (values.size() != 7) {
    throw std::runtime_error(name + " must contain 7 elements: [x, y, z, qx, qy, qz, qw]");
  }
  Eigen::Quaterniond quaternion(values[6], values[3], values[4], values[5]);
  if (!std::isfinite(quaternion.norm()) || quaternion.norm() <= std::numeric_limits<double>::epsilon()) {
    throw std::runtime_error(name + " contains an invalid quaternion");
  }
  quaternion.normalize();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translate(Eigen::Vector3d(values[0], values[1], values[2]));
  transform.rotate(quaternion);
  return transform;
}

double translation_distance(const Eigen::Isometry3d & lhs, const Eigen::Isometry3d & rhs)
{
  return (lhs.translation() - rhs.translation()).norm();
}

double rotation_distance(const Eigen::Isometry3d & lhs, const Eigen::Isometry3d & rhs)
{
  return Eigen::Quaterniond(lhs.rotation()).angularDistance(Eigen::Quaterniond(rhs.rotation()));
}

std::string transform_to_string(const Eigen::Isometry3d & transform)
{
  const Eigen::Quaterniond q(transform.rotation());
  char buffer[160];
  std::snprintf(
    buffer, sizeof(buffer),
    "T=(%.3f, %.3f, %.3f) Q=(%.4f, %.4f, %.4f, %.4f)",
    transform.translation().x(), transform.translation().y(), transform.translation().z(),
    q.x(), q.y(), q.z(), q.w());
  return std::string(buffer);
}

PointCloud::Ptr from_pcl_xyz(const pcl::PointCloud<pcl::PointXYZ> & src)
{
  auto cloud = std::make_shared<PointCloud>();
  cloud->points.reserve(src.size());
  for (const auto & point : src.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    cloud->points.emplace_back(point.x, point.y, point.z, 1.0);
  }
  cloud->normals.resize(cloud->points.size());
  cloud->covs.resize(cloud->points.size());
  return cloud;
}

}  // namespace

EMAIsometry::EMAIsometry(double keep_ratio) : keep_ratio_(keep_ratio) {}

void EMAIsometry::initialize(const Eigen::Isometry3d & value)
{
  value_ = value;
  initialized_ = true;
}

void EMAIsometry::update(const Eigen::Isometry3d & value)
{
  if (!initialized_) {
    initialize(value);
    return;
  }
  value_.translation() =
    keep_ratio_ * value_.translation() + (1.0 - keep_ratio_) * value.translation();
  value_.linear() = Eigen::Quaterniond(value_.linear())
                      .slerp(1.0 - keep_ratio_, Eigen::Quaterniond(value.linear()))
                      .toRotationMatrix();
}

Eigen::Isometry3d EMAIsometry::value() const { return value_; }

bool EMAIsometry::initialized() const { return initialized_; }

OdomLocalizerNode::OdomLocalizerNode(const rclcpp::NodeOptions & options)
: Node("odom_localizer", options)
{
  load_parameters();
  if (enable_debug_) {
    get_logger().set_level(rclcpp::Logger::Level::Debug);
  }
  map_to_odom_filter_ = std::make_unique<EMAIsometry>(ema_ratio_);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  if (publish_tf_direct_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  }
  load_map();
  create_interfaces();
  initialize_transform(initial_transform_, "startup.initial_transform");
  initialize_time_ = std::chrono::steady_clock::now();
}

void OdomLocalizerNode::load_parameters()
{
  enable_debug_ = declare_parameter<bool>("general.enable_debug", enable_debug_);
  num_threads_ = declare_parameter<int>("general.num_threads", num_threads_);
  publish_rate_hz_ = declare_parameter<double>("general.publish_rate_hz", publish_rate_hz_);

  map_frame_ = declare_parameter<std::string>("frames.map", map_frame_);
  odom_frame_ = declare_parameter<std::string>("frames.odom", odom_frame_);
  base_frame_ = declare_parameter<std::string>("frames.base", base_frame_);

  registered_cloud_topic_ =
    declare_parameter<std::string>("topics.registered_cloud", registered_cloud_topic_);
  initialpose_topic_ = declare_parameter<std::string>("topics.initialpose", initialpose_topic_);

  prior_pcd_file_ = declare_parameter<std::string>("map.prior_pcd_file", prior_pcd_file_);
  map_downsample_resolution_ =
    declare_parameter<double>("map.downsample_resolution", map_downsample_resolution_);
  map_covariance_neighbors_ =
    declare_parameter<int>("map.covariance_neighbors", map_covariance_neighbors_);
  publish_prior_cloud_ = declare_parameter<bool>("map.publish_prior_cloud", publish_prior_cloud_);

  registered_window_frames_ = static_cast<size_t>(
    declare_parameter<int>("source.registered_window_frames", static_cast<int>(registered_window_frames_)));
  min_points_raw_ = static_cast<size_t>(
    declare_parameter<int>("source.min_points_raw", static_cast<int>(min_points_raw_)));
  min_points_downsampled_ = static_cast<size_t>(
    declare_parameter<int>("source.min_points_downsampled", static_cast<int>(min_points_downsampled_)));
  source_downsample_resolution_ =
    declare_parameter<double>("source.downsample_resolution", source_downsample_resolution_);
  source_covariance_neighbors_ =
    declare_parameter<int>("source.covariance_neighbors", source_covariance_neighbors_);

  initial_transform_ = transform_from_xyz_xyzw(
    declare_parameter<std::vector<double>>(
      "startup.initial_transform", std::vector<double>{0, 0, 0, 0, 0, 0, 1}),
    "startup.initial_transform");
  bootstrap_duration_s_ =
    declare_parameter<double>("startup.bootstrap_duration_s", bootstrap_duration_s_);

  registration_period_s_ =
    declare_parameter<double>("registration.period_s", registration_period_s_);
  bootstrap_max_correspondence_distance_ = declare_parameter<double>(
    "registration.gicp.bootstrap_max_correspondence_distance", bootstrap_max_correspondence_distance_);
  locked_max_correspondence_distance_ = declare_parameter<double>(
    "registration.gicp.locked_max_correspondence_distance", locked_max_correspondence_distance_);
  max_iterations_ = declare_parameter<int>("registration.gicp.max_iterations", max_iterations_);
  translation_eps_ =
    declare_parameter<double>("registration.gicp.translation_eps", translation_eps_);
  rotation_eps_ = declare_parameter<double>("registration.gicp.rotation_eps", rotation_eps_);

  max_normalized_error_ =
    declare_parameter<double>("registration.quality.max_normalized_error", max_normalized_error_);
  min_overlap_ratio_ =
    declare_parameter<double>("registration.quality.min_overlap_ratio", min_overlap_ratio_);
  min_information_eigenvalue_ = declare_parameter<double>(
    "registration.quality.min_information_eigenvalue", min_information_eigenvalue_);
  max_information_condition_number_ = declare_parameter<double>(
    "registration.quality.max_information_condition_number", max_information_condition_number_);

  ema_ratio_ = declare_parameter<double>("update.ema_ratio", ema_ratio_);
  max_translation_step_ =
    declare_parameter<double>("update.max_translation_step", max_translation_step_);
  max_rotation_step_ = declare_parameter<double>("update.max_rotation_step", max_rotation_step_);
  lock_z_ = declare_parameter<bool>("update.lock_z", lock_z_);
  publish_tf_direct_ = declare_parameter<bool>("tf.publish_direct", publish_tf_direct_);
  map_to_odom_topic_ = declare_parameter<std::string>("tf.transform_topic", map_to_odom_topic_);

  if (num_threads_ < 1) {
    throw std::runtime_error("general.num_threads must be >= 1");
  }
  if (registered_window_frames_ == 0) {
    throw std::runtime_error("source.registered_window_frames must be > 0");
  }
}

void OdomLocalizerNode::load_map()
{
  if (prior_pcd_file_.empty()) {
    RCLCPP_WARN(get_logger(), "No prior PCD specified; publishing initial map->odom only");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  if (pcl::io::loadPCDFile(prior_pcd_file_, pcl_cloud) == -1) {
    throw std::runtime_error("Failed to load prior PCD: " + prior_pcd_file_);
  }
  map_cloud_ = from_pcl_xyz(pcl_cloud);
  RCLCPP_INFO(
    get_logger(),
    "Loaded prior PCD %s (%zu points). Frame is mapping-session odom/map; not applying lidar extrinsics.",
    prior_pcd_file_.c_str(), map_cloud_->size());

  if (map_downsample_resolution_ > 0.0) {
    map_cloud_ = small_gicp::voxelgrid_sampling_omp(*map_cloud_, map_downsample_resolution_, num_threads_);
    RCLCPP_INFO(get_logger(), "Downsampled prior cloud to %zu points", map_cloud_->size());
  }

  map_kd_tree_ = std::make_shared<KdTree<PointCloud>>(map_cloud_, KdTreeBuilderOMP(num_threads_));
  small_gicp::estimate_covariances_omp(
    *map_cloud_, *map_kd_tree_, map_covariance_neighbors_, num_threads_);
}

void OdomLocalizerNode::create_interfaces()
{
  registered_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    registered_cloud_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { registered_cloud_callback(msg); });

  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    initialpose_topic_, 10,
    [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
      initial_pose_callback(msg);
    });

  if (publish_prior_cloud_) {
    prior_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/odom_localizer/prior_cloud", rclcpp::QoS(1).transient_local());
    maybe_publish_prior_cloud();
  }

  map_to_odom_pub_ = create_publisher<geometry_msgs::msg::TransformStamped>(
    map_to_odom_topic_, rclcpp::QoS(1).transient_local());

  const auto publish_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_hz_));
  publish_timer_ = create_wall_timer(publish_period, [this]() { publish_timer_callback(); });

  if (map_cloud_ && map_kd_tree_) {
    const auto registration_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(registration_period_s_));
    registration_timer_ =
      create_wall_timer(registration_period, [this]() { registration_timer_callback(); });
  }
}

void OdomLocalizerNode::publish_timer_callback() { publish_transform(); }

Eigen::Isometry3d OdomLocalizerNode::get_current_map_to_odom() const
{
  std::lock_guard<std::mutex> lock(transform_state_mutex_);
  return map_to_odom_filter_->value();
}

Eigen::Isometry3d OdomLocalizerNode::maybe_lock_z(const Eigen::Isometry3d & transform) const
{
  if (!lock_z_) {
    return transform;
  }
  Eigen::Isometry3d out = transform;
  Eigen::Vector3d translation = out.translation();
  translation.z() = initial_transform_.translation().z();
  out.translation() = translation;
  return out;
}

void OdomLocalizerNode::initialize_transform(const Eigen::Isometry3d & transform, const char * reason)
{
  const Eigen::Isometry3d constrained = maybe_lock_z(transform);
  {
    std::lock_guard<std::mutex> lock(transform_state_mutex_);
    map_to_odom_filter_->initialize(constrained);
    last_accepted_registration_transform_ = std::nullopt;
    has_successful_registration_ = false;
  }
  RCLCPP_INFO(
    get_logger(), "Initialized map->odom from %s: %s%s",
    reason, transform_to_string(constrained).c_str(),
    lock_z_ ? " (z locked)" : "");
}

void OdomLocalizerNode::registered_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto cloud = convert_pointcloud2(msg);
  if (cloud->empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(source_data_mutex_);
  registered_cloud_window_.push_back(std::move(cloud));
  while (registered_cloud_window_.size() > registered_window_frames_) {
    registered_cloud_window_.pop_front();
  }
}

void OdomLocalizerNode::initial_pose_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  Eigen::Isometry3d map_to_base = Eigen::Isometry3d::Identity();
  map_to_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z;
  map_to_base.linear() = Eigen::Quaterniond(
                           msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                           msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                           .toRotationMatrix();

  Eigen::Isometry3d odom_to_base = Eigen::Isometry3d::Identity();
  try {
    const auto tf_msg =
      tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
    odom_to_base = tf2::transformToEigen(tf_msg.transform);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      get_logger(),
      "initialpose: odom->base TF unavailable (%s), assuming identity (typical at boot)",
      ex.what());
  }

  initialize_transform(map_to_base * odom_to_base.inverse(), "/initialpose");
}

void OdomLocalizerNode::registration_timer_callback()
{
  if (!map_cloud_ || !map_kd_tree_) {
    return;
  }
  const auto start_time = std::chrono::steady_clock::now();
  const auto prepared = prepare_source();
  if (!prepared) {
    return;
  }

  const Eigen::Isometry3d current = get_current_map_to_odom();
  const RegistrationAssessment assessment = align_to_map(*prepared);
  const double elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();

  if (!assessment.accepted) {
    RCLCPP_WARN(
      get_logger(),
      "Registration failed (%s): %s [current=%s gicp=%s iters=%zu inliers=%zu "
      "error=%.4f overlap=%.4f] (%.2f s)",
      prepared->label.c_str(), assessment.reason.c_str(), transform_to_string(current).c_str(),
      transform_to_string(assessment.transform).c_str(), assessment.metrics.iterations,
      assessment.metrics.num_inliers, assessment.metrics.normalized_error,
      assessment.metrics.overlap_ratio, elapsed);
    return;
  }

  if (!apply_registration_update(assessment.transform)) {
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "Accepted registration (%s): [current=%s gicp=%s update=%s inliers=%zu "
    "error=%.4f overlap=%.4f] (%.2f s)",
    prepared->label.c_str(), transform_to_string(current).c_str(),
    transform_to_string(assessment.transform).c_str(),
    transform_to_string(get_current_map_to_odom()).c_str(), assessment.metrics.num_inliers,
    assessment.metrics.normalized_error, assessment.metrics.overlap_ratio, elapsed);
}

std::optional<OdomLocalizerNode::PreparedSource> OdomLocalizerNode::prepare_source() const
{
  std::deque<PointCloud::Ptr> window;
  {
    std::lock_guard<std::mutex> lock(source_data_mutex_);
    if (registered_cloud_window_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Registration skipped: no /cloud_registered yet (need SensorDataQoS match)");
      return std::nullopt;
    }
    window = registered_cloud_window_;
  }

  PointCloud accumulated;
  for (const auto & cloud : window) {
    accumulated.points.insert(accumulated.points.end(), cloud->points.begin(), cloud->points.end());
  }
  const size_t raw_points = accumulated.points.size();
  if (raw_points < min_points_raw_) {
    RCLCPP_WARN(
      get_logger(), "Registration skipped: raw source too small (%zu < %zu)", raw_points,
      min_points_raw_);
    return std::nullopt;
  }

  PointCloud::Ptr downsampled;
  if (source_downsample_resolution_ > 0.0) {
    downsampled =
      small_gicp::voxelgrid_sampling_omp(accumulated, source_downsample_resolution_, num_threads_);
  } else {
    downsampled = std::make_shared<PointCloud>(std::move(accumulated));
  }
  if (downsampled->size() < min_points_downsampled_) {
    RCLCPP_WARN(
      get_logger(), "Registration skipped: downsampled source too small (%zu < %zu)",
      downsampled->size(), min_points_downsampled_);
    return std::nullopt;
  }

  auto source_tree =
    std::make_shared<KdTree<PointCloud>>(downsampled, KdTreeBuilderOMP(num_threads_));
  small_gicp::estimate_covariances_omp(
    *downsampled, *source_tree, source_covariance_neighbors_, num_threads_);

  const double elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - initialize_time_).count();
  PreparedSource prepared;
  prepared.cloud = std::move(downsampled);
  prepared.raw_points = raw_points;
  prepared.downsampled_points = prepared.cloud->size();
  prepared.label = elapsed < bootstrap_duration_s_ ? "cloud_registered bootstrap window"
                                                   : "cloud_registered periodic window";
  return prepared;
}

OdomLocalizerNode::RegistrationAssessment OdomLocalizerNode::align_to_map(
  const PreparedSource & source) const
{
  RegistrationAssessment assessment;
  Registration<GICPFactor, ParallelReductionOMP> registration;
  registration.reduction.num_threads = num_threads_;
  registration.optimizer.max_iterations = max_iterations_;
  registration.criteria.translation_eps = translation_eps_;
  registration.criteria.rotation_eps = rotation_eps_;

  const bool bootstrapping = !has_successful_registration_ ||
    std::chrono::duration<double>(std::chrono::steady_clock::now() - initialize_time_).count() <
      bootstrap_duration_s_;
  const double max_corr = bootstrapping ? bootstrap_max_correspondence_distance_
                                        : locked_max_correspondence_distance_;
  registration.rejector.max_dist_sq = max_corr * max_corr;

  const Eigen::Isometry3d initial_guess = get_current_map_to_odom();
  try {
    const RegistrationResult result =
      registration.align(*map_cloud_, *source.cloud, *map_kd_tree_, initial_guess);
    assessment.transform = result.T_target_source;
    assessment.accepted =
      evaluate_registration_result(result, source.cloud->size(), assessment.metrics, assessment.reason);
  } catch (const std::exception & exception) {
    assessment.reason = std::string("GICP exception: ") + exception.what();
  }
  return assessment;
}

bool OdomLocalizerNode::evaluate_registration_result(
  const RegistrationResult & result,
  const size_t source_points,
  RegistrationMetrics & metrics,
  std::string & reason) const
{
  metrics.converged = result.converged;
  metrics.iterations = result.iterations + 1;
  metrics.num_inliers = result.num_inliers;
  if (!result.converged) {
    reason = "solver did not converge";
    return false;
  }
  if (source_points == 0 || result.num_inliers == 0) {
    reason = "no valid inliers";
    return false;
  }
  if (!std::isfinite(result.error)) {
    reason = "non-finite registration error";
    return false;
  }
  metrics.normalized_error = result.error / static_cast<double>(result.num_inliers);
  metrics.overlap_ratio = static_cast<double>(result.num_inliers) / static_cast<double>(source_points);
  if (metrics.normalized_error > max_normalized_error_) {
    reason = "normalized error exceeds threshold";
    return false;
  }
  if (metrics.overlap_ratio < min_overlap_ratio_) {
    reason = "overlap ratio below threshold";
    return false;
  }

  const Eigen::Matrix<double, 6, 6> information = 0.5 * (result.H + result.H.transpose());
  if (!information.allFinite()) {
    reason = "information matrix contains non-finite values";
    return false;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigen_solver(information);
  if (eigen_solver.info() != Eigen::Success) {
    reason = "failed to decompose information matrix";
    return false;
  }
  const Eigen::Matrix<double, 6, 1> eigenvalues = eigen_solver.eigenvalues();
  metrics.min_information_eigenvalue = eigenvalues.minCoeff();
  metrics.max_information_eigenvalue = eigenvalues.maxCoeff();
  if (metrics.min_information_eigenvalue <= 0.0) {
    reason = "information matrix is not positive definite";
    return false;
  }
  if (metrics.min_information_eigenvalue < min_information_eigenvalue_) {
    reason = "minimum information eigenvalue below threshold";
    return false;
  }
  metrics.information_condition_number =
    metrics.max_information_eigenvalue / metrics.min_information_eigenvalue;
  if (
    max_information_condition_number_ > 0.0 &&
    metrics.information_condition_number > max_information_condition_number_)
  {
    reason = "information matrix is too ill-conditioned";
    return false;
  }
  return true;
}

bool OdomLocalizerNode::apply_registration_update(const Eigen::Isometry3d & transform)
{
  const Eigen::Isometry3d constrained = maybe_lock_z(transform);
  {
    std::lock_guard<std::mutex> lock(transform_state_mutex_);
    if (!has_successful_registration_) {
      map_to_odom_filter_->initialize(constrained);
      last_accepted_registration_transform_ = constrained;
      has_successful_registration_ = true;
      RCLCPP_INFO(
        get_logger(), "Accepted first successful registration directly: %s%s",
        transform_to_string(constrained).c_str(), lock_z_ ? " (z locked)" : "");
      return true;
    }
  }

  Eigen::Isometry3d previous;
  double translation_delta = 0.0;
  double rotation_delta = 0.0;
  {
    std::lock_guard<std::mutex> lock(transform_state_mutex_);
    previous = *last_accepted_registration_transform_;
    translation_delta = translation_distance(constrained, previous);
    rotation_delta = rotation_distance(constrained, previous);
    if (translation_delta <= max_translation_step_ && rotation_delta <= max_rotation_step_) {
      map_to_odom_filter_->update(constrained);
      last_accepted_registration_transform_ = constrained;
    }
  }

  if (translation_delta > max_translation_step_) {
    RCLCPP_WARN(
      get_logger(),
      "Discarded registration update: translation jump %.3f m exceeds %.3f m",
      translation_delta, max_translation_step_);
    return false;
  }
  if (rotation_delta > max_rotation_step_) {
    RCLCPP_WARN(
      get_logger(),
      "Discarded registration update: rotation jump %.3f rad exceeds %.3f rad",
      rotation_delta, max_rotation_step_);
    return false;
  }
  return true;
}

void OdomLocalizerNode::publish_transform()
{
  if (!map_to_odom_filter_->initialized()) {
    return;
  }
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = now() + rclcpp::Duration::from_seconds(1.0 / publish_rate_hz_ + 0.05);
  msg.header.frame_id = map_frame_;
  msg.child_frame_id = odom_frame_;
  msg.transform = tf2::eigenToTransform(get_current_map_to_odom()).transform;
  map_to_odom_pub_->publish(msg);
  if (tf_broadcaster_) {
    tf_broadcaster_->sendTransform(msg);
  }
}

void OdomLocalizerNode::maybe_publish_prior_cloud()
{
  if (!prior_cloud_pub_ || !map_cloud_) {
    return;
  }
  auto msg = to_pointcloud2(*map_cloud_, map_frame_);
  msg.header.stamp = now();
  prior_cloud_pub_->publish(msg);
}

PointCloud::Ptr OdomLocalizerNode::convert_pointcloud2(
  const sensor_msgs::msg::PointCloud2::SharedPtr & msg) const
{
  auto cloud = std::make_shared<PointCloud>();
  size_t offset_x = std::numeric_limits<size_t>::max();
  size_t offset_y = std::numeric_limits<size_t>::max();
  size_t offset_z = std::numeric_limits<size_t>::max();
  for (const auto & field : msg->fields) {
    if (field.name == "x") {
      offset_x = field.offset;
    } else if (field.name == "y") {
      offset_y = field.offset;
    } else if (field.name == "z") {
      offset_z = field.offset;
    }
  }
  if (
    offset_x == std::numeric_limits<size_t>::max() ||
    offset_y == std::numeric_limits<size_t>::max() ||
    offset_z == std::numeric_limits<size_t>::max())
  {
    RCLCPP_WARN(get_logger(), "PointCloud2 missing x/y/z fields");
    return cloud;
  }

  const size_t num_points = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
  const uint8_t * data_ptr = msg->data.data();
  cloud->points.reserve(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    std::memcpy(&x, data_ptr + offset_x, sizeof(float));
    std::memcpy(&y, data_ptr + offset_y, sizeof(float));
    std::memcpy(&z, data_ptr + offset_z, sizeof(float));
    data_ptr += msg->point_step;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    cloud->points.emplace_back(x, y, z, 1.0);
  }
  cloud->normals.resize(cloud->points.size());
  cloud->covs.resize(cloud->points.size());
  return cloud;
}

sensor_msgs::msg::PointCloud2 OdomLocalizerNode::to_pointcloud2(
  const PointCloud & cloud, const std::string & frame) const
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = frame;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(cloud.size());
  msg.is_dense = false;
  msg.is_bigendian = false;
  msg.point_step = 12;
  msg.row_step = msg.point_step * msg.width;
  sensor_msgs::msg::PointField field;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  field.name = "x";
  field.offset = 0;
  msg.fields.push_back(field);
  field.name = "y";
  field.offset = 4;
  msg.fields.push_back(field);
  field.name = "z";
  field.offset = 8;
  msg.fields.push_back(field);
  msg.data.resize(msg.row_step);
  for (size_t i = 0; i < cloud.size(); ++i) {
    const float x = static_cast<float>(cloud.points[i].x());
    const float y = static_cast<float>(cloud.points[i].y());
    const float z = static_cast<float>(cloud.points[i].z());
    std::memcpy(msg.data.data() + i * 12 + 0, &x, 4);
    std::memcpy(msg.data.data() + i * 12 + 4, &y, 4);
    std::memcpy(msg.data.data() + i * 12 + 8, &z, 4);
  }
  return msg;
}

}  // namespace odom_localizer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(odom_localizer::OdomLocalizerNode)
