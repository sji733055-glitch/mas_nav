/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 *
 * 双雷达融合的"一台掉线"支撑：后雷达→前雷达的刚体变换、IMU 换源换算，
 * 以及"某台雷达是否还在线"的判据。这里只放纯计算/纯状态，方便单测，
 * ROS 侧（发布、日志、队列）都在 mid360_driver_node.cpp。
 */

#pragma once

#include "mid360_driver/mid360_driver.hpp"
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace mid360_driver {

    // 后雷达系 → 前雷达（参考）系的刚体变换。点云与 IMU 共用同一份外参。
    struct MergeTransform {
        float r00, r01, r02;
        float r10, r11, r12;
        float r20, r21, r22;
        float tx, ty, tz;
    };

    // xyz_rpy = [x, y, z, roll, pitch, yaw]（米/弧度），语义同 merge_extrinsic_back_to_front。
    MergeTransform make_merge_transform(const std::vector<double> &xyz_rpy);

    // 把一帧点云从后雷达系变到前雷达系；除 xyz 外其余字段（含 timestamp）原样保留。
    std::vector<Point> transform_merge_points(const std::vector<Point> &points, const MergeTransform &tf);

    // 把后雷达 IMU 的测量换算到前雷达 IMU 系。前提：每台雷达的 IMU 轴与它自己的雷达系对齐
    // （Point-LIO 的 extrinsic_R 为单位阵，正是这个前提）。
    //   ω_front = R · ω_back
    //   a_front = R · a_back + ω_front × (ω_front × d)
    // d 为"后 IMU 指向前 IMU"的杆臂（前系），≈ -外参平移；d 项即旋转时两安装点之间的
    // 离心加速度。角加速度项 α×d 需要微分陀螺，噪声大于收益，不补偿。
    ImuMsg rotate_imu_to_front(const ImuMsg &imu, const MergeTransform &tf);

    // 单台雷达的在线判据。点云和 IMU 各用一个实例（两路可以独立掉线）。
    struct LidarHealthConfig {
        // 超过这么久没收到任何数据即判为掉线；<=0 表示永不判掉线（等于关掉降级）。
        double stale_timeout_s = 0.5;
        // 掉线后恢复的数据需连续新鲜这么久，才重新参与双雷达配对，避免边界抖动来回切。
        // 只约束"掉线后回来"：自启动以来第一次收到该雷达数据时立即上线，不额外等保持期。
        double recover_hold_s = 0.5;
    };

    // 选 IMU 来源：参考雷达（front）在线就用它；参考雷达掉线而另一台在线就切过去；
    // 两台都不在线时按"谁新鲜用谁"兜底——避免"参考雷达已坏、另一台刚回来还在恢复保持期"
    // 这种两头都不用的空档。返回 true = 用参考雷达的 IMU。
    inline bool select_front_imu_source(bool front_online, bool back_online, bool front_fresh, bool back_fresh) {
        if (front_online) {
            return true;
        }
        if (back_online) {
            return false;
        }
        return front_fresh || !back_fresh;
    }

    class LidarHealthGate {
    private:
        struct State {
            double last_seen_s = std::numeric_limits<double>::lowest();
            double recover_since_s = std::numeric_limits<double>::lowest();
            bool online = false;
            bool ever_online = false;
        };

        LidarHealthConfig config_;
        State front_;
        State back_;

        static const State &select(const State &front, const State &back, bool is_front) { return is_front ? front : back; }
        static State &select(State &front, State &back, bool is_front) { return is_front ? front : back; }

    public:
        LidarHealthGate() = default;
        explicit LidarHealthGate(const LidarHealthConfig &config) : config_(config) {}

        void configure(const LidarHealthConfig &config) { config_ = config; }
        const LidarHealthConfig &config() const { return config_; }

        // 收到该路数据时调用（is_front = 参考雷达）。
        void mark_seen(bool is_front, double now_s) {
            select(front_, back_, is_front).last_seen_s = now_s;
        }

        bool ever_seen(bool is_front) const { return select(front_, back_, is_front).last_seen_s > std::numeric_limits<double>::lowest(); }

        double last_seen_s(bool is_front) const { return select(front_, back_, is_front).last_seen_s; }

        // 纯超时判据：数据是否新鲜，不改状态。
        bool is_fresh(bool is_front, double now_s) const {
            const State &state = select(front_, back_, is_front);
            if (state.last_seen_s <= std::numeric_limits<double>::lowest()) {
                return false;
            }
            if (!(config_.stale_timeout_s > 0.0)) {
                return true;
            }
            return now_s - state.last_seen_s <= config_.stale_timeout_s;
        }

        // 每个发布 tick 调一次：推进"恢复保持"计时，返回该雷达当前是否可参与双雷达配对。
        bool update_online(bool is_front, double now_s) {
            State &state = select(front_, back_, is_front);
            if (!is_fresh(is_front, now_s)) {
                state.online = false;
                state.recover_since_s = std::numeric_limits<double>::lowest();
                return false;
            }
            if (state.online) {
                return true;
            }
            // 自启动以来第一次收到该雷达的数据：立即上线，保持期只用于"掉线后回来"。
            if (!state.ever_online) {
                state.online = true;
                state.ever_online = true;
                return true;
            }
            if (state.recover_since_s <= std::numeric_limits<double>::lowest()) {
                state.recover_since_s = now_s;
            }
            if (now_s - state.recover_since_s >= config_.recover_hold_s) {
                state.online = true;
                state.recover_since_s = std::numeric_limits<double>::lowest();
            }
            return state.online;
        }

        bool is_online(bool is_front) const { return select(front_, back_, is_front).online; }

        void reset() {
            front_ = State{};
            back_ = State{};
        }
    };

}// namespace mid360_driver
