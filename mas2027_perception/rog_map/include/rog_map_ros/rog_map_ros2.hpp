/**
 * This file is part of ROG-Map
 *
 * Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
 * Developed by Yunfan REN <renyf at connect dot hku dot hk>
 * for more information see <https://github.com/hku-mars/ROG-Map>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * ROG-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ROG-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ROG-Map. If not, see <http://www.gnu.org/licenses/>.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#ifndef ROG_MAP_ROS_HPP
#define ROG_MAP_ROS_HPP

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rog_map/rog_map.h>
#include <rog_map/rog_map_visualizer.hpp>
#include <super_utils/color_msg_utils.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace rog_map {
using namespace super_utils;

class ROGMapROS : public ROGMap
{
  rclcpp::Node::SharedPtr nh_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_nh_;
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<ROGMapVisualizer> visualizer_driver_;

  const double getSystemWalltimeNow() override { return now().seconds(); }

  void getSystemWalltimeNow(rclcpp::Time & _in) { _in = now(); };

  rclcpp::Time now() const { return node_clock_->get_clock()->now(); }

  bool getPriorMapTransform(PriorMapTransform2D & transform) override
  {
    if (cfg_.prior_map_frame == cfg_.frame_id) {
      transform = PriorMapTransform2D{};
      return true;
    }
    if (!tf_) {
      RCLCPP_WARN_THROTTLE(node_logging_->get_logger(),
        *node_clock_->get_clock(),
        2000,
        "[ROGMap] prior map TF is unavailable because the shared TF buffer is null");
      return false;
    }
    try {
      // lookupTransform(target, source, ...) returns T_target_source.
      const auto tf_msg = tf_->lookupTransform(cfg_.prior_map_frame, cfg_.frame_id, tf2::TimePointZero);
      transform.tx = tf_msg.transform.translation.x;
      transform.ty = tf_msg.transform.translation.y;
      const auto & q = tf_msg.transform.rotation;
      transform.yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      if (std::isfinite(transform.tx) && std::isfinite(transform.ty) &&
          std::isfinite(transform.yaw)) {
        return true;
      }
      RCLCPP_WARN_THROTTLE(node_logging_->get_logger(),
        *node_clock_->get_clock(),
        2000,
        "[ROGMap] prior map TF from '%s' to '%s' contains a non-finite 2D transform",
        cfg_.frame_id.c_str(),
        cfg_.prior_map_frame.c_str());
      return false;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(node_logging_->get_logger(),
        *node_clock_->get_clock(),
        2000,
        "[ROGMap] cannot transform projection from '%s' to prior map frame '%s': %s",
        cfg_.frame_id.c_str(),
        cfg_.prior_map_frame.c_str(),
        error.what());
      return false;
    }
  }

  template <typename NodeT> void bindNode(NodeT node)
  {
    node_base_ = node->get_node_base_interface();
    node_topics_ = node->get_node_topics_interface();
    node_timers_ = node->get_node_timers_interface();
    node_clock_ = node->get_node_clock_interface();
    node_logging_ = node->get_node_logging_interface();
    node_parameters_ = node->get_node_parameters_interface();
  }

  rclcpp::CallbackGroup::SharedPtr createCallbackGroup(rclcpp::CallbackGroupType type)
  {
    return node_base_->create_callback_group(type);
  }

  template <typename MsgT>
  typename rclcpp::Publisher<MsgT>::SharedPtr createPublisher(
    const std::string & topic, const rclcpp::QoS & qos)
  {
    return rclcpp::create_publisher<MsgT>(node_parameters_, node_topics_, topic, qos);
  }

  template <typename MsgT, typename CallbackT>
  typename rclcpp::Subscription<MsgT>::SharedPtr createSubscription(const std::string & topic,
    const rclcpp::QoS & qos,
    CallbackT && callback,
    const rclcpp::SubscriptionOptions & options = rclcpp::SubscriptionOptions())
  {
    return rclcpp::create_subscription<MsgT>(
      node_parameters_, node_topics_, topic, qos, std::forward<CallbackT>(callback), options);
  }

  template <typename DurationRepT, typename DurationT, typename CallbackT>
  rclcpp::TimerBase::SharedPtr createWallTimer(std::chrono::duration<DurationRepT, DurationT> period,
    CallbackT && callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
  {
    return rclcpp::create_wall_timer(
      period, std::forward<CallbackT>(callback), group, node_base_.get(), node_timers_.get());
  }

  ROGMapVisualizer::Publishers vm_;

  struct ROSCallback
  {
    rclcpp::CallbackGroup::SharedPtr odom_me_cbk_group, cloud_me_cbk_group, update_cbk_group;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
    int unfinished_frame_cnt{0};
    Pose pc_pose;
    PointCloud pc;
    double pc_odom_age_ms{0.0};
    rclcpp::TimerBase::SharedPtr update_timer;
    std::mutex updete_lock;
    std::condition_variable cv;
    std::atomic<bool> stop_worker{false};
    std::thread update_worker;
  } rc_;
  std::mutex map_io_mutex_;
  std::mutex viz_frame_mutex_;
  struct VizFrame
  {
    sensor_msgs::msg::PointCloud2 occ, raw_occ, unknown, occ_inf, unknown_inf, frontier, esdf,
      height_delta, field, decay;
    nav_msgs::msg::OccupancyGrid layer_value, layer_dynamic, layer_static, layer_type, layer_confidence;
    visualization_msgs::msg::MarkerArray markers;
    bool has_occ{false};
    bool has_raw_occ{false};
    bool has_unknown{false};
    bool has_occ_inf{false};
    bool has_unknown_inf{false};
    bool has_frontier{false};
    bool has_esdf{false};
    bool has_height_delta{false};
    bool has_field{false};
    bool has_decay{false};
    bool has_layer_value{false};
    bool has_layer_dynamic{false};
    bool has_layer_static{false};
    bool has_layer_type{false};
    bool has_layer_confidence{false};
    bool has_markers{false};
  };
  std::shared_ptr<const VizFrame> viz_frame_;
  std::shared_ptr<const VizFrame> viz_heavy_;
  double last_heavy_viz_s_{0.0};

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
  {
    if (performance_monitor_) {
      performance_monitor_->recordOdom(now().seconds());
    }
    std::lock_guard<std::mutex> lk(rc_.updete_lock);
    updateRobotState(std::make_pair(
      Vec3f(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z),
      Quatf(odom_msg->pose.pose.orientation.w,
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z)));
  }

  void cloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr cloud_msg)
  {
    const double cbk_t = now().seconds();
    const double msg_stamp = rclcpp::Time(cloud_msg->header.stamp).seconds();
    const double queue_delay_ms = msg_stamp > 0.0 ? std::max(0.0, cbk_t - msg_stamp) * 1000.0 : 0.0;
    const double msg_points =
      static_cast<double>(cloud_msg->width) * static_cast<double>(cloud_msg->height);
    if (performance_monitor_) {
      performance_monitor_->recordCloudCallback(cbk_t, msg_points, queue_delay_ms, 0.0);
    }
    if (msg_points <= 0.0) {
      if (performance_monitor_) {
        performance_monitor_->recordCloudDropEmpty();
      }
      return;
    }
    if (!robot_state_.rcv) {
      if (performance_monitor_) {
        performance_monitor_->recordCloudDropNoOdom();
      }
      std::cout << YELLOW << " -- [ROS] No odom received, skip cloud callback." << RESET << std::endl;
      return;
    }
    if (cbk_t - robot_state_.rcv_time > cfg_.odom_timeout) {
      if (performance_monitor_) {
        performance_monitor_->recordCloudDropOdomTimeout();
      }
      std::cout << YELLOW << " -- [ROS] Odom timeout, skip cloud callback." << RESET << std::endl;
      return;
    }
    PointCloud temp_pc;
    const auto convert_start = std::chrono::steady_clock::now();
    pcl::fromROSMsg(*cloud_msg, temp_pc);
    const double convert_time_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - convert_start).count();
    if (performance_monitor_) {
      performance_monitor_->recordCloudConvertTime(convert_time_ms);
    }
    if (temp_pc.empty()) {
      if (performance_monitor_) {
        performance_monitor_->recordCloudDropEmpty();
      }
      return;
    }
    rc_.updete_lock.lock();
    rc_.pc = temp_pc;
    rc_.pc_pose = std::make_pair(robot_state_.p, robot_state_.q);
    rc_.pc_odom_age_ms = std::max(0.0, cbk_t - robot_state_.rcv_time) * 1000.0;
    rc_.unfinished_frame_cnt++;
    map_empty_ = false;
    rc_.updete_lock.unlock();
    rc_.cv.notify_one();
  }

  void watchdogCallback()
  {
    if (map_empty_ && cfg_.ros_callback_en) {
      std::cout << YELLOW << " -- [ROG WARN] No point cloud input, check the topic name." << RESET
                << std::endl;
    }
  }

  void stopUpdateWorker()
  {
    rc_.stop_worker.store(true);
    rc_.cv.notify_all();
    if (rc_.update_worker.joinable()) {
      rc_.update_worker.join();
    }
  }

  void updateWorkerLoop()
  {
    while (!rc_.stop_worker.load()) {
      PointCloud temp_pc;
      Pose temp_pose;
      double temp_odom_age_ms = 0.0;
      int dropped = 0;
      {
        std::unique_lock<std::mutex> lk(rc_.updete_lock);
        rc_.cv.wait(lk, [this]() {
          return rc_.stop_worker.load() || rc_.unfinished_frame_cnt > 0;
        });
        if (rc_.stop_worker.load()) {
          break;
        }
        dropped = rc_.unfinished_frame_cnt;
        temp_pc.swap(rc_.pc);
        temp_pose = rc_.pc_pose;
        temp_odom_age_ms = rc_.pc_odom_age_ms;
        rc_.unfinished_frame_cnt = 0;
      }
      if (dropped > 1) {
        static double last_warn_t = 0.0;
        const double cur_t = now().seconds();
        if (cur_t - last_warn_t > 1.0) {
          std::cout << YELLOW << " -- [ROG WARN] Unfinished frame cnt > 1 (dropped "
                    << (dropped - 1) << "), the map may not work in real-time" << RESET
                    << std::endl;
          last_warn_t = cur_t;
        }
      }
      if (performance_monitor_) {
        performance_monitor_->recordValidCloud(temp_odom_age_ms);
      }
      {
        std::lock_guard<std::mutex> io(map_io_mutex_);
        updateMapInternal(temp_pc, temp_pose);
      }
      if (cfg_.visualization_en && hasVisualizationSubscriber()) {
        captureVizFrame();
      }
    }
  }

  void collectOccupiedForViz(const Vec3f & box_min, const Vec3f & box_max, vec_E<Vec3f> & out)
  {
    out.clear();
    if (!cfg_.decay_active_list_en) {
      boxSearch(box_min, box_max, OCCUPIED, out);
      return;
    }
    out.reserve(active_ids_.size());
    for (const int hash_id : active_ids_) {
      if (hash_id < 0 || hash_id >= static_cast<int>(occupancy_buffer_.size()) ||
          hash_id >= static_cast<int>(active_flags_.size()) || !active_flags_[hash_id] ||
          !isOccupied(occupancy_buffer_[hash_id])) {
        continue;
      }
      Vec3f pos;
      hashIdToPos(hash_id, pos);
      if ((pos.array() < box_min.array()).any() || (pos.array() > box_max.array()).any()) {
        continue;
      }
      out.push_back(pos);
    }
  }

  void captureVizFrame()
  {
    if (map_empty_) {
      return;
    }
    auto frame = std::make_shared<VizFrame>();
    Vec3f box_max = robot_state_.p + cfg_.visualization_range / 2;
    Vec3f box_min = robot_state_.p - cfg_.visualization_range / 2;

    boundBoxByLocalMap(box_min, box_max);
    if ((box_max - box_min).minCoeff() <= 0) {
      cout << YELLOW << " -- [ROGMap] Visualization range is too small." << RESET << endl;
      return;
    }

    if (layer_ && !layer_->empty()) {
      if (vm_.layer_value_pub && vm_.layer_value_pub->get_subscription_count() >= 1) {
        fillLayerMaskGrid(fused_projection_mask_, frame->layer_value);
        frame->has_layer_value = true;
      }
      if (vm_.layer_value_dynamic_pub &&
          vm_.layer_value_dynamic_pub->get_subscription_count() >= 1) {
        fillLayerMaskGrid(layer_->mask(), frame->layer_dynamic);
        frame->has_layer_dynamic = true;
      }
      if (vm_.layer_value_static_pub &&
          vm_.layer_value_static_pub->get_subscription_count() >= 1) {
        fillLayerMaskGrid(prior_projection_mask_, frame->layer_static);
        frame->has_layer_static = true;
      }
      if (vm_.layer_type_pub && vm_.layer_type_pub->get_subscription_count() >= 1) {
        std::vector<uint8_t> types(layer_->cells().size(), 0U);
        for (size_t i = 0; i < layer_->cells().size(); ++i) {
          switch (layer_->cells()[i].type) {
          case CellType::UNKNOWN:
            types[i] = 0U;
            break;
          case CellType::FREE:
            types[i] = 33U;
            break;
          case CellType::PASSABLE:
            types[i] = 66U;
            break;
          case CellType::OCCUPIED:
            types[i] = 100U;
            break;
          }
        }
        fillLayerGrid(types, frame->layer_type);
        frame->has_layer_type = true;
      }
      if (vm_.layer_confidence_pub && vm_.layer_confidence_pub->get_subscription_count() >= 1) {
        std::vector<uint8_t> confidence(layer_->cells().size(), 0U);
        for (size_t i = 0; i < layer_->cells().size(); ++i) {
          confidence[i] = static_cast<uint8_t>(
            std::clamp(static_cast<int>(std::round(layer_->cells()[i].confidence * 100.0f)), 0, 100));
        }
        fillLayerGrid(confidence, frame->layer_confidence);
        frame->has_layer_confidence = true;
      }
      if (vm_.layer_height_delta_pub && vm_.layer_height_delta_pub->get_subscription_count() >= 1) {
        fillLayerHeightDeltaCloud(frame->height_delta);
        frame->has_height_delta = true;
      }
    }

    if (field_ && field_->isValid() && vm_.field_pub && vm_.field_pub->get_subscription_count() >= 1) {
      fillFieldCloud(frame->field);
      frame->has_field = true;
    }
    if (vm_.decay_cells_pub && vm_.decay_cells_pub->get_subscription_count() >= 1) {
      fillDecayCellsCloud(frame->decay);
      frame->has_decay = true;
    }

    vec_E<Vec3f> occ_map;
    if ((vm_.occ_pub && vm_.occ_pub->get_subscription_count() >= 1) ||
        (vm_.raw_occ_pub && vm_.raw_occ_pub->get_subscription_count() >= 1)) {
      collectOccupiedForViz(box_min, box_max, occ_map);
    }
    if (vm_.occ_pub && vm_.occ_pub->get_subscription_count() >= 1) {
      vecEVec3fToPC2(occ_map, frame->occ);
      frame->has_occ = true;
    }
    if (vm_.raw_occ_pub && vm_.raw_occ_pub->get_subscription_count() >= 1) {
      vecEVec3fToPC2(occ_map, frame->raw_occ);
      frame->has_raw_occ = true;
    }

    if (vm_.mkr_arr_pub && vm_.mkr_arr_pub->get_subscription_count() >= 1) {
      visualization_msgs::msg::MarkerArray mkr_arr;
      visualizeBoundingBox(
        mkr_arr, now(), box_min, box_max, "Visualization Range", Color::Purple());
      visualizeText(mkr_arr,
        now(),
        "Visualization Range Text",
        "Visualization Range",
        box_max + Vec3f(0, 0, 0.5),
        Color::Purple(),
        0.6,
        0);

      Vec3f local_map_max(999, 999, 999), local_map_min(-999, -999, -999);
      boundBoxByLocalMap(local_map_min, local_map_max);
      visualizeBoundingBox(
        mkr_arr, now(), local_map_min, local_map_max, "Local Map Range", Color::Orange());
      visualizeText(mkr_arr,
        now(),
        "Local Map Range Text",
        "Local Map Range",
        local_map_max + Vec3f(0, 0, 1.0),
        Color::Orange(),
        0.6,
        0);

      visualizeBoundingBox(mkr_arr,
        now(),
        raycast_data_.cache_box_min,
        raycast_data_.cache_box_max,
        "Updating Range",
        Color::Green());
      visualizeText(mkr_arr,
        now(),
        "Updating Range Text",
        "Updating Range",
        raycast_data_.cache_box_max + Vec3f(0, 0, 0.5),
        Color::Green(),
        0.6,
        0);

      visualizePoint(
        mkr_arr, now(), local_map_origin_d_, Color::Red(), "Local Map Origin", 0.2, 0);

      if (cfg_.esdf_en) {
        Vec3f esdf_box_max, esdf_box_min;
        esdf_map_->getUpdatedBbox(esdf_box_min, esdf_box_max);
        visualizeText(mkr_arr,
          now(),
          "ESDF Map Text",
          "ESDF Map",
          esdf_box_max + Vec3f(0, 0, 1.0),
          Color::Blue(),
          0.6,
          0);
        visualizeBoundingBox(
          mkr_arr, now(), esdf_box_min, esdf_box_max, "ESDF Updating Range", Color::Blue());
      }

      for (auto & marker : mkr_arr.markers) {
        marker.header.frame_id = cfg_.visualization_frame_id;
      }
      frame->markers = std::move(mkr_arr);
      frame->has_markers = true;
    }

    const double now_s = now().seconds();
    const bool heavy_due = (now_s - last_heavy_viz_s_) >= 0.5;
    std::shared_ptr<VizFrame> heavy;
    if (heavy_due) {
      last_heavy_viz_s_ = now_s;
      heavy = std::make_shared<VizFrame>();
      if (vm_.unknown_pub && vm_.unknown_pub->get_subscription_count() >= 1) {
        vec_E<Vec3f> unknown_map;
        boxSearch(box_min, box_max, UNKNOWN, unknown_map);
        vecEVec3fToPC2(unknown_map, heavy->unknown);
        heavy->has_unknown = true;
      }
      if (cfg_.unk_inflation_en && vm_.unknown_inf_pub &&
          vm_.unknown_inf_pub->get_subscription_count() >= 1) {
        vec_E<Vec3f> inf_unknown_map;
        boxSearchInflate(box_min, box_max, UNKNOWN, inf_unknown_map);
        vecEVec3fToPC2(inf_unknown_map, heavy->unknown_inf);
        heavy->has_unknown_inf = true;
      }
      if (cfg_.frontier_extraction_en && vm_.frontier_pub &&
          vm_.frontier_pub->get_subscription_count() >= 1) {
        vec_E<Vec3f> frontier_map;
        boxSearch(box_min, box_max, FRONTIER, frontier_map);
        vecEVec3fToPC2(frontier_map, heavy->frontier);
        heavy->has_frontier = true;
      }
      if (vm_.occ_inf_pub && vm_.occ_inf_pub->get_subscription_count() >= 1) {
        vec_E<Vec3f> inf_occ_map;
        boxSearchInflate(box_min, box_max, OCCUPIED, inf_occ_map);
        vecEVec3fToPC2(inf_occ_map, heavy->occ_inf);
        heavy->has_occ_inf = true;
      }
      if (cfg_.esdf_en && vm_.esdf_pub && vm_.esdf_pub->get_subscription_count() >= 1) {
        PointCloud pc;
        esdf_map_->getPositiveESDFPointCloud(box_min, box_max, robot_state_.p.z() - 0.5, pc);
        pcl::toROSMsg(pc, heavy->esdf);
        heavy->esdf.header.frame_id = cfg_.visualization_frame_id;
        heavy->esdf.header.stamp = now();
        heavy->has_esdf = true;
      }
    }

    {
      std::lock_guard<std::mutex> lk(viz_frame_mutex_);
      viz_frame_ = std::move(frame);
      if (heavy) {
        viz_heavy_ = std::move(heavy);
      }
    }
  }

  void vizCallback()
  {
    if (!cfg_.visualization_en) {
      return;
    }
    if (!hasVisualizationSubscriber()) {
      return;
    }
    std::shared_ptr<const VizFrame> frame;
    std::shared_ptr<const VizFrame> heavy;
    {
      std::lock_guard<std::mutex> lk(viz_frame_mutex_);
      frame = viz_frame_;
      heavy = viz_heavy_;
    }
    if (!frame && !heavy) {
      return;
    }
    if (heavy) {
      if (heavy->has_unknown && vm_.unknown_pub) {
        vm_.unknown_pub->publish(heavy->unknown);
      }
      if (heavy->has_unknown_inf && vm_.unknown_inf_pub) {
        vm_.unknown_inf_pub->publish(heavy->unknown_inf);
      }
      if (heavy->has_frontier && vm_.frontier_pub) {
        vm_.frontier_pub->publish(heavy->frontier);
      }
      if (heavy->has_occ_inf && vm_.occ_inf_pub) {
        vm_.occ_inf_pub->publish(heavy->occ_inf);
      }
      if (heavy->has_esdf && vm_.esdf_pub) {
        vm_.esdf_pub->publish(heavy->esdf);
      }
    }
    if (!frame) {
      return;
    }
    if (frame->has_layer_value && vm_.layer_value_pub) {
      vm_.layer_value_pub->publish(frame->layer_value);
    }
    if (frame->has_layer_dynamic && vm_.layer_value_dynamic_pub) {
      vm_.layer_value_dynamic_pub->publish(frame->layer_dynamic);
    }
    if (frame->has_layer_static && vm_.layer_value_static_pub) {
      vm_.layer_value_static_pub->publish(frame->layer_static);
    }
    if (frame->has_layer_type && vm_.layer_type_pub) {
      vm_.layer_type_pub->publish(frame->layer_type);
    }
    if (frame->has_layer_confidence && vm_.layer_confidence_pub) {
      vm_.layer_confidence_pub->publish(frame->layer_confidence);
    }
    if (frame->has_height_delta && vm_.layer_height_delta_pub) {
      vm_.layer_height_delta_pub->publish(frame->height_delta);
    }
    if (frame->has_field && vm_.field_pub) {
      vm_.field_pub->publish(frame->field);
    }
    if (frame->has_decay && vm_.decay_cells_pub) {
      vm_.decay_cells_pub->publish(frame->decay);
    }
    if (frame->has_occ && vm_.occ_pub) {
      vm_.occ_pub->publish(frame->occ);
    }
    if (frame->has_raw_occ && vm_.raw_occ_pub) {
      vm_.raw_occ_pub->publish(frame->raw_occ);
    }
    if (frame->has_markers && vm_.mkr_arr_pub) {
      vm_.mkr_arr_pub->publish(frame->markers);
    }
  }

  bool hasVisualizationSubscriber()
  {
    return (vm_.unknown_pub && vm_.unknown_pub->get_subscription_count() >= 1) ||
           (vm_.unknown_inf_pub && vm_.unknown_inf_pub->get_subscription_count() >= 1) ||
           (vm_.layer_value_pub && vm_.layer_value_pub->get_subscription_count() >= 1) ||
           (vm_.layer_value_dynamic_pub &&
            vm_.layer_value_dynamic_pub->get_subscription_count() >= 1) ||
           (vm_.layer_value_static_pub &&
            vm_.layer_value_static_pub->get_subscription_count() >= 1) ||
           (vm_.layer_type_pub && vm_.layer_type_pub->get_subscription_count() >= 1) ||
           (vm_.layer_confidence_pub && vm_.layer_confidence_pub->get_subscription_count() >= 1) ||
           (vm_.layer_height_delta_pub && vm_.layer_height_delta_pub->get_subscription_count() >= 1) ||
           (vm_.field_pub && vm_.field_pub->get_subscription_count() >= 1) ||
           (vm_.decay_cells_pub && vm_.decay_cells_pub->get_subscription_count() >= 1) ||
           (vm_.frontier_pub && vm_.frontier_pub->get_subscription_count() >= 1) ||
           (vm_.occ_pub && vm_.occ_pub->get_subscription_count() >= 1) ||
           (vm_.raw_occ_pub && vm_.raw_occ_pub->get_subscription_count() >= 1) ||
           (vm_.occ_inf_pub && vm_.occ_inf_pub->get_subscription_count() >= 1) ||
           (vm_.esdf_pub && vm_.esdf_pub->get_subscription_count() >= 1) ||
           (vm_.mkr_arr_pub && vm_.mkr_arr_pub->get_subscription_count() >= 1);
  }

  void vecEVec3fToPC2(const vec_E<Vec3f> & points, sensor_msgs::msg::PointCloud2 & cloud)
  {
    // 设置header信息
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl_cloud.resize(points.size());
    for (long unsigned int i = 0; i < points.size(); i++) {
      pcl_cloud[i].x = static_cast<float>(points[i][0]);
      pcl_cloud[i].y = static_cast<float>(points[i][1]);
      pcl_cloud[i].z = static_cast<float>(points[i][2]);
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = now();
    cloud.header.frame_id = cfg_.visualization_frame_id;
  }

  void fillLayerGrid(const std::vector<uint8_t> & data, nav_msgs::msg::OccupancyGrid & grid)
  {
    grid.header.stamp = now();
    grid.header.frame_id = cfg_.visualization_frame_id;
    grid.info.resolution = static_cast<float>(layer_->resolution());
    grid.info.width = static_cast<uint32_t>(layer_->width());
    grid.info.height = static_cast<uint32_t>(layer_->height());
    grid.info.origin.position.x = layer_->origin().x();
    grid.info.origin.position.y = layer_->origin().y();
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
      grid.data[i] = static_cast<int8_t>(std::min<int>(100, data[i]));
    }
  }

  void fillLayerMaskGrid(const std::vector<uint8_t> & mask, nav_msgs::msg::OccupancyGrid & grid)
  {
    std::vector<uint8_t> occupancy(mask.size(), 0U);
    for (size_t i = 0; i < mask.size(); ++i) {
      occupancy[i] = mask[i] == 0U ? 100U : 0U;
    }
    fillLayerGrid(occupancy, grid);
  }

  void fillLayerHeightDeltaCloud(sensor_msgs::msg::PointCloud2 & cloud)
  {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    const auto & cells = layer_->cells();
    pcl_cloud.reserve(cells.size());
    for (int y = 0; y < layer_->height(); ++y) {
      for (int x = 0; x < layer_->width(); ++x) {
        const size_t idx =
          static_cast<size_t>(y) * static_cast<size_t>(layer_->width()) + static_cast<size_t>(x);
        if (idx >= cells.size()) {
          continue;
        }
        const auto & cell = cells[idx];
        if (cell.type == CellType::UNKNOWN) {
          continue;
        }
        pcl::PointXYZI p;
        p.x =
          static_cast<float>(layer_->origin().x() + (static_cast<double>(x) + 0.5) * layer_->resolution());
        p.y =
          static_cast<float>(layer_->origin().y() + (static_cast<double>(y) + 0.5) * layer_->resolution());
        p.z = cell.occupied_z_max_abs;
        p.intensity = cell.height_delta;
        pcl_cloud.push_back(p);
      }
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = now();
    cloud.header.frame_id = cfg_.visualization_frame_id;
  }

  void fillFieldCloud(sensor_msgs::msg::PointCloud2 & cloud)
  {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    const auto distances = field_->distances();
    const int width = field_->width();
    const int height = field_->height();
    const double resolution = field_->resolution();
    const Eigen::Vector2d origin = field_->origin();
    pcl_cloud.reserve(distances.size());
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x);
        if (idx >= distances.size()) {
          continue;
        }
        const double d = distances[idx];
        if (!std::isfinite(d)) {
          continue;
        }
        pcl::PointXYZI p;
        p.x = static_cast<float>(origin.x() + (static_cast<double>(x) + 0.5) * resolution);
        p.y = static_cast<float>(origin.y() + (static_cast<double>(y) + 0.5) * resolution);
        p.z = 0.0f;
        p.intensity = static_cast<float>(d);
        pcl_cloud.push_back(p);
      }
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = now();
    cloud.header.frame_id = cfg_.visualization_frame_id;
  }

  void fillDecayCellsCloud(sensor_msgs::msg::PointCloud2 & cloud)
  {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl_cloud.reserve(active_ids_.size());
    const double now_s = now().seconds();
    for (const int hash_id : active_ids_) {
      if (hash_id < 0 || hash_id >= static_cast<int>(occupancy_buffer_.size()) || !active_flags_[hash_id] ||
          !isOccupied(occupancy_buffer_[hash_id])) {
        continue;
      }
      Vec3f pos;
      hashIdToPos(hash_id, pos);
      pcl::PointXYZI p;
      p.x = static_cast<float>(pos.x());
      p.y = static_cast<float>(pos.y());
      p.z = static_cast<float>(pos.z());
      p.intensity = static_cast<float>(std::max(0.0, now_s - static_cast<double>(last_hit_time_[hash_id])));
      pcl_cloud.push_back(p);
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = now();
    cloud.header.frame_id = cfg_.visualization_frame_id;
  }

  void initializeRos()
  {
    // TODO: The current implementation uses a lenient QoS configuration for message transmission.
    const rclcpp::QoS qos(rclcpp::QoS(1).best_effort().keep_last(1).durability_volatile());

    if (cfg_.prior_map_enable) {
      prior_map_ = loadPriorMap(cfg_.prior_map_yaml_path, cfg_.prior_map_pgm_path);
      RCLCPP_INFO(node_logging_->get_logger(),
        "[ROGMap] loaded prior map %dx%d at %.3f m/cell in frame '%s'",
        prior_map_.width,
        prior_map_.height,
        prior_map_.resolution,
        cfg_.prior_map_frame.c_str());
    }
    init();
    /// Initialize visualization module
    if (cfg_.visualization_en) {
      visualizer_driver_ = std::make_unique<ROGMapVisualizer>();
      visualizer_driver_->configure(node_base_,
        node_parameters_,
        node_topics_,
        node_timers_,
        cfg_,
        std::bind(&ROGMapROS::vizCallback, this));
      const auto & pubs = visualizer_driver_->publishers();
      vm_.occ_pub = pubs.occ_pub;
      vm_.raw_occ_pub = pubs.raw_occ_pub;
      vm_.unknown_pub = pubs.unknown_pub;
      vm_.esdf_neg_pub = pubs.esdf_neg_pub;
      vm_.esdf_occ_pub = pubs.esdf_occ_pub;
      vm_.occ_inf_pub = pubs.occ_inf_pub;
      vm_.unknown_inf_pub = pubs.unknown_inf_pub;
      vm_.frontier_pub = pubs.frontier_pub;
      vm_.esdf_pub = pubs.esdf_pub;
      vm_.layer_height_delta_pub = pubs.layer_height_delta_pub;
      vm_.field_pub = pubs.field_pub;
      vm_.decay_cells_pub = pubs.decay_cells_pub;
      vm_.layer_value_pub = pubs.layer_value_pub;
      vm_.layer_value_dynamic_pub = pubs.layer_value_dynamic_pub;
      vm_.layer_value_static_pub = pubs.layer_value_static_pub;
      vm_.layer_type_pub = pubs.layer_type_pub;
      vm_.layer_confidence_pub = pubs.layer_confidence_pub;
      vm_.mkr_arr_pub = pubs.mkr_arr_pub;
    }

    if (cfg_.ros_callback_en) {
#ifdef _OPENMP
      omp_set_dynamic(0);
      omp_set_num_threads(2);
#endif
      rc_.odom_me_cbk_group = createCallbackGroup(rclcpp::CallbackGroupType::MutuallyExclusive);
      rc_.cloud_me_cbk_group = createCallbackGroup(rclcpp::CallbackGroupType::MutuallyExclusive);
      rclcpp::SubscriptionOptions so;
      so.callback_group = rc_.odom_me_cbk_group;
      rc_.odom_sub = createSubscription<nav_msgs::msg::Odometry>(
        cfg_.odom_topic, qos, std::bind(&ROGMapROS::odomCallback, this, std::placeholders::_1), so);
      so.callback_group = rc_.cloud_me_cbk_group;
      so.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
      rc_.cloud_sub = createSubscription<sensor_msgs::msg::PointCloud2>(
        cfg_.cloud_topic,
        rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) { this->cloudCallback(std::move(msg)); },
        so);
      rc_.update_cbk_group = createCallbackGroup(rclcpp::CallbackGroupType::MutuallyExclusive);
      rc_.update_timer = createWallTimer(std::chrono::seconds(1),
        std::bind(&ROGMapROS::watchdogCallback, this),
        rc_.update_cbk_group);
      rc_.stop_worker.store(false);
      rc_.update_worker = std::thread([this]() { this->updateWorkerLoop(); });
    }
  }

public:
  typedef shared_ptr<ROGMapROS> Ptr;

  ROGMapROS(const ROGMapROS &) = delete;
  ROGMapROS & operator=(const ROGMapROS &) = delete;

  ~ROGMapROS()
  {
    stopUpdateWorker();
  }

  ROGMapROS(const rclcpp_lifecycle::LifecycleNode::SharedPtr nh,
    const rog_map::Config & cfg,
    const std::shared_ptr<tf2_ros::Buffer> & tf = nullptr)
  : lifecycle_nh_(nh), tf_(tf)
  {
    bindNode(lifecycle_nh_);
    cfg_ = cfg;
    initializeRos();
  }

  ROGMapROS(const rclcpp::Node::SharedPtr nh,
    const rog_map::Config & cfg,
    const std::shared_ptr<tf2_ros::Buffer> & tf = nullptr)
  : nh_(nh), tf_(tf)
  {
    bindNode(nh_);
    cfg_ = cfg;
    initializeRos();
  }

  std::shared_ptr<rog_map::MapQueryInterface> queryInterface() const
  {
    return getQueryInterface();
  }

private:
  static void visualizeBoundingBox(visualization_msgs::msg::MarkerArray & mkrarr,
    const rclcpp::Time & stamp,
    const Vec3f & box_min,
    const Vec3f & box_max,
    const string & ns,
    const Color & color,
    const double & size_x = 0.1,
    const double & alpha = 1.0,
    const bool & print_ns = true)
  {
    Vec3f size = (box_max - box_min) / 2;
    Vec3f vis_pos_world = (box_min + box_max) / 2;
    double width = size.x();
    double length = size.y();
    double hight = size.z();

    // Publish Bounding box
    int id = 0;
    visualization_msgs::msg::Marker line_strip;
    line_strip.header.stamp = stamp;
    line_strip.header.frame_id = "world";
    line_strip.action = visualization_msgs::msg::Marker::ADD;
    line_strip.ns = ns;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = id++;  // unique id, useful when multiple markers exist.
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;  // marker type
    line_strip.scale.x = size_x;

    line_strip.color = color;
    line_strip.color.a = alpha;  //不透明度，设0则全透明
    geometry_msgs::msg::Point p[8];

    // vis_pos_world是目标物的坐标
    p[0].x = vis_pos_world(0) - width;
    p[0].y = vis_pos_world(1) + length;
    p[0].z = vis_pos_world(2) + hight;
    p[1].x = vis_pos_world(0) - width;
    p[1].y = vis_pos_world(1) - length;
    p[1].z = vis_pos_world(2) + hight;
    p[2].x = vis_pos_world(0) - width;
    p[2].y = vis_pos_world(1) - length;
    p[2].z = vis_pos_world(2) - hight;
    p[3].x = vis_pos_world(0) - width;
    p[3].y = vis_pos_world(1) + length;
    p[3].z = vis_pos_world(2) - hight;
    p[4].x = vis_pos_world(0) + width;
    p[4].y = vis_pos_world(1) + length;
    p[4].z = vis_pos_world(2) - hight;
    p[5].x = vis_pos_world(0) + width;
    p[5].y = vis_pos_world(1) - length;
    p[5].z = vis_pos_world(2) - hight;
    p[6].x = vis_pos_world(0) + width;
    p[6].y = vis_pos_world(1) - length;
    p[6].z = vis_pos_world(2) + hight;
    p[7].x = vis_pos_world(0) + width;
    p[7].y = vis_pos_world(1) + length;
    p[7].z = vis_pos_world(2) + hight;
    // LINE_STRIP类型仅仅将line_strip.points中相邻的两个点相连，如0和1，1和2，2和3
    for (int i = 0; i < 8; i++) {
      line_strip.points.push_back(p[i]);
    }
    //为了保证矩形框的八条边都存在：
    line_strip.points.push_back(p[0]);
    line_strip.points.push_back(p[3]);
    line_strip.points.push_back(p[2]);
    line_strip.points.push_back(p[5]);
    line_strip.points.push_back(p[6]);
    line_strip.points.push_back(p[1]);
    line_strip.points.push_back(p[0]);
    line_strip.points.push_back(p[7]);
    line_strip.points.push_back(p[4]);
    mkrarr.markers.push_back(line_strip);
  }

  static void visualizeText(visualization_msgs::msg::MarkerArray & mkr_arr,
    const rclcpp::Time & stamp,
    const std::string & ns,
    const std::string & text,
    const Vec3f & position,
    const Color & c = Color::White(),
    const double & size = 0.6,
    const int & id = -1)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = stamp;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.ns = ns.c_str();
    if (id >= 0) {
      marker.id = id;
    } else {
      static int id = 0;
      marker.id = id++;
    }
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.scale.z = size;
    marker.color = c;
    marker.text = text;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.w = 1.0;
    mkr_arr.markers.push_back(marker);
  };

  static void visualizePoint(visualization_msgs::msg::MarkerArray & mkr_arr,
    const rclcpp::Time & stamp,
    const Vec3f & pt,
    Color color = Color::Pink(),
    std::string ns = "pt",
    double size = 0.1,
    int id = -1,
    const bool & print_ns = true)
  {
    visualization_msgs::msg::Marker marker_ball;
    static int cnt = 0;
    Vec3f cur_pos = pt;
    if (isnan(pt.x()) || isnan(pt.y()) || isnan(pt.z())) {
      return;
    }
    marker_ball.header.frame_id = "world";
    marker_ball.header.stamp = stamp;
    marker_ball.ns = ns.c_str();
    marker_ball.id = id >= 0 ? id : cnt++;
    marker_ball.action = visualization_msgs::msg::Marker::ADD;
    marker_ball.pose.orientation.w = 1.0;
    marker_ball.type = visualization_msgs::msg::Marker::SPHERE;
    marker_ball.scale.x = size;
    marker_ball.scale.y = size;
    marker_ball.scale.z = size;
    marker_ball.color = color;

    geometry_msgs::msg::Point p;
    p.x = cur_pos.x();
    p.y = cur_pos.y();
    p.z = cur_pos.z();

    marker_ball.pose.position = p;
    mkr_arr.markers.push_back(marker_ball);

    // add test
    if (print_ns) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = stamp;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.ns = ns + "_text";
      if (id >= 0) {
        marker.id = id;
      } else {
        static int id = 0;
        marker.id = id++;
      }
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.scale.z = 0.6;
      marker.color = color;
      marker.text = ns;
      marker.pose.position.x = cur_pos.x();
      marker.pose.position.y = cur_pos.y();
      marker.pose.position.z = cur_pos.z() + 0.5;
      marker.pose.orientation.w = 1.0;
      mkr_arr.markers.push_back(marker);
    }
  }
};
}  // namespace rog_map
#endif  // ROG_MAP_ROS_HPP
