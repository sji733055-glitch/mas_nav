/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 *
 * 双雷达"一台掉线"的纯逻辑单测：外参变换、IMU 换源换算、在线/恢复判据。
 * 不含 ROS，端到端（假雷达 + 真实节点）见 test/integration_degrade_check.py。
 */

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "mid360_driver/merge_failover.hpp"

namespace {

    using mid360_driver::ImuMsg;
    using mid360_driver::LidarHealthConfig;
    using mid360_driver::LidarHealthGate;
    using mid360_driver::MergeTransform;
    using mid360_driver::Point;

    // 现场配置里的后雷达→前雷达外参（见 config/params.yaml）。
    const std::vector<double> kBackToFront = {0.0, 0.200484459, -0.223172538, -1.677800, 0.0, 0.0};

    bool near(const double lhs, const double rhs, const double tolerance) {
        return std::abs(lhs - rhs) <= tolerance;
    }

    void test_transform_matrix() {
        // 本工程的外参只有 roll（手性对称 R=Rx(-2α)，α=0.8389）。
        const MergeTransform tf = mid360_driver::make_merge_transform(kBackToFront);
        const double roll = kBackToFront[3];
        assert(near(tf.r00, 1.0, 1e-6));
        assert(near(tf.r01, 0.0, 1e-6));
        assert(near(tf.r02, 0.0, 1e-6));
        assert(near(tf.r11, std::cos(roll), 1e-6));
        assert(near(tf.r12, -std::sin(roll), 1e-6));
        assert(near(tf.r21, std::sin(roll), 1e-6));
        assert(near(tf.r22, std::cos(roll), 1e-6));
        assert(near(tf.tx, 0.0, 1e-6));
        assert(near(tf.ty, 0.200484459, 1e-6));
        assert(near(tf.tz, -0.223172538, 1e-6));
        // 旋转必须是正交的（点云变换不能缩放）。
        assert(near(tf.r11 * tf.r11 + tf.r12 * tf.r12, 1.0, 1e-6));
        assert(near(tf.r11 * tf.r21 + tf.r12 * tf.r22, 0.0, 1e-6));

        // 后雷达 z 轴上的点：R·(0,0,1) = (0, +sin(α')..., ) —— 用解析式核对，再加平移。
        std::vector<Point> points(1);
        points[0].x = 0.0F;
        points[0].y = 0.0F;
        points[0].z = 1.0F;
        points[0].intensity = 7.0F;
        points[0].timestamp = 12.5;
        const auto transformed = mid360_driver::transform_merge_points(points, tf);
        assert(transformed.size() == 1);
        assert(near(transformed[0].x, 0.0, 1e-5));
        assert(near(transformed[0].y, -std::sin(roll) + tf.ty, 1e-5));
        assert(near(transformed[0].z, std::cos(roll) + tf.tz, 1e-5));
        // 非坐标字段原样保留。
        assert(transformed[0].intensity == 7.0F);
        assert(transformed[0].timestamp == 12.5);

        // 纯 yaw 90°：验证一般性（x→y）。
        const MergeTransform yaw90 = mid360_driver::make_merge_transform({0.0, 0.0, 0.0, 0.0, 0.0, M_PI_2});
        std::vector<Point> axis(1);
        axis[0].x = 1.0F;
        axis[0].y = 0.0F;
        axis[0].z = 0.0F;
        const auto rotated = mid360_driver::transform_merge_points(axis, yaw90);
        assert(near(rotated[0].x, 0.0, 1e-6));
        assert(near(rotated[0].y, 1.0, 1e-6));
        assert(near(rotated[0].z, 0.0, 1e-6));
    }

    void test_imu_rotation() {
        // 单位变换：数据不变。
        const MergeTransform identity = mid360_driver::make_merge_transform({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        ImuMsg imu{};
        imu.timestamp = 3.0;
        imu.angular_velocity_x = 0.1F;
        imu.angular_velocity_y = -0.2F;
        imu.angular_velocity_z = 0.3F;
        imu.linear_acceleration_x = 0.0F;
        imu.linear_acceleration_y = 0.0F;
        imu.linear_acceleration_z = 1.0F;
        const ImuMsg unchanged = mid360_driver::rotate_imu_to_front(imu, identity);
        assert(near(unchanged.angular_velocity_x, 0.1, 1e-6));
        assert(near(unchanged.angular_velocity_y, -0.2, 1e-6));
        assert(near(unchanged.angular_velocity_z, 0.3, 1e-6));
        assert(near(unchanged.linear_acceleration_x, 0.0, 1e-6));
        assert(near(unchanged.linear_acceleration_y, 0.0, 1e-6));
        assert(near(unchanged.linear_acceleration_z, 1.0, 1e-6));

        // 静止时只剩旋转：现场外参把后雷达 +z 转到前系 (0, 0.9943, -0.1068)，即车体前向。
        const MergeTransform tf = mid360_driver::make_merge_transform(kBackToFront);
        ImuMsg still{};
        still.timestamp = 5.0;
        still.linear_acceleration_z = 1.0F;
        const ImuMsg rotated_still = mid360_driver::rotate_imu_to_front(still, tf);
        assert(near(rotated_still.linear_acceleration_x, 0.0, 1e-5));
        assert(near(rotated_still.linear_acceleration_y, 0.9943, 1e-4));
        assert(near(rotated_still.linear_acceleration_z, -0.1068, 1e-4));
        assert(near(rotated_still.timestamp, 5.0, 1e-12));

        // 离心项：绕后雷达系 z 轴以 1 rad/s 自转、输入加速度为 0，输出即 ω×(ω×d)，杆臂 d = −t（前系）。
        // 陀螺须同步转到前系：ω_front = R·(0,0,1) = (0, 0.9942806, −0.1067996)。
        ImuMsg spinning{};
        spinning.angular_velocity_z = 1.0F;
        const ImuMsg rotated_spinning = mid360_driver::rotate_imu_to_front(spinning, tf);
        assert(near(rotated_spinning.linear_acceleration_x, 0.0, 1e-6));
        assert(near(rotated_spinning.linear_acceleration_y, -0.0214117, 1e-5));
        assert(near(rotated_spinning.linear_acceleration_z, -0.1993378, 1e-5));
        assert(near(rotated_spinning.angular_velocity_y, 0.9942806, 1e-6));
        assert(near(rotated_spinning.angular_velocity_z, -0.1067996, 1e-6));
    }

    void test_health_gate_stale_and_recover() {
        LidarHealthGate gate{LidarHealthConfig{0.5, 0.3}};
        // 从没收到过数据 → 既不算新鲜也不算在线。
        assert(!gate.is_fresh(true, 0.0));
        assert(!gate.update_online(true, 0.0));
        assert(!gate.ever_seen(true));

        gate.mark_seen(true, 0.0);
        assert(gate.ever_seen(true));
        assert(gate.is_fresh(true, 0.5));      // 刚好等于超时仍算新鲜（<=）
        assert(!gate.is_fresh(true, 0.51));
        // 自启动以来第一次收到数据：立即上线，不等保持期（不拖慢启动）。
        assert(gate.update_online(true, 0.0));
        assert(gate.is_online(true));
        // 一路掉线不影响另一路。
        assert(!gate.update_online(false, 0.0));

        // 掉线：超时后立刻下线，且必须重新等满保持期才能恢复配对。
        gate.mark_seen(true, 1.0);
        assert(gate.update_online(true, 1.0));
        assert(!gate.update_online(true, 1.6));
        assert(!gate.is_online(true));
        gate.mark_seen(true, 1.7);
        assert(!gate.update_online(true, 1.7));      // 恢复计时开始
        assert(!gate.update_online(true, 1.95));     // 0.25 < 0.3
        assert(gate.update_online(true, 2.0));       // 0.3 → 重新上线
    }

    void test_health_gate_flapping_and_zero_hold() {
        // recover_hold = 0：恢复也立即生效（等于不做抖动保护）。
        LidarHealthGate instant{LidarHealthConfig{0.5, 0.0}};
        instant.mark_seen(false, 10.0);
        assert(instant.update_online(false, 10.0));
        assert(!instant.update_online(false, 11.0));  // 静默 1 s → 下线
        instant.mark_seen(false, 11.1);
        assert(instant.update_online(false, 11.1));

        // 边界抖动：0.5 s 超时、0.4 s 保持，短暂掉线后不能立刻重新配对。
        // 保持期从"第一个看到数据新鲜"的 tick 起算（本类不做定时，由调用方每 tick 推进）。
        LidarHealthGate flappy{LidarHealthConfig{0.5, 0.4}};
        flappy.mark_seen(true, 0.0);
        assert(flappy.update_online(true, 0.0));        // 首次上线立即生效
        flappy.mark_seen(true, 0.41);
        assert(flappy.update_online(true, 0.42));       // 仍在线
        assert(!flappy.update_online(true, 1.00));      // 静默 0.59 s → 下线
        // 恢复期里雷达持续发包，但配对要等满保持期。
        flappy.mark_seen(true, 1.01);
        assert(!flappy.update_online(true, 1.01));
        flappy.mark_seen(true, 1.11);
        assert(!flappy.update_online(true, 1.20));      // 1.20−1.01 = 0.19 < 0.4
        flappy.mark_seen(true, 1.21);
        assert(!flappy.update_online(true, 1.40));      // 1.40−1.01 = 0.39 < 0.4
        flappy.mark_seen(true, 1.41);
        assert(flappy.update_online(true, 1.42));       // 1.42−1.01 = 0.41 → 重新在线
        // 再掉一次：ever_online 已置位，不再享受"首次"豁免，恢复要重新等满保持期。
        assert(!flappy.update_online(true, 2.0));       // 静默 0.59 s → 下线
        flappy.mark_seen(true, 2.05);
        assert(!flappy.update_online(true, 2.05));      // 恢复计时开始
        assert(!flappy.update_online(true, 2.40));      // 0.35 < 0.4
        assert(flappy.update_online(true, 2.50));       // 0.45 ≥ 0.4 → 重新在线

        // stale_timeout = 0：永不判掉线（等于关掉降级），但仍要求"收到过数据"。
        LidarHealthGate never_stale{LidarHealthConfig{0.0, 0.0}};
        assert(!never_stale.update_online(true, 0.0));
        never_stale.mark_seen(true, 0.0);
        assert(never_stale.is_fresh(true, 1e6));
        assert(never_stale.update_online(true, 1e6));

        never_stale.reset();
        assert(!never_stale.ever_seen(true));
    }

    void test_imu_source_selection() {
        using mid360_driver::select_front_imu_source;
        // 参考雷达在线 → 用它（哪怕另一台也在线）。
        assert(select_front_imu_source(true, true, true, true));
        assert(select_front_imu_source(true, false, true, false));
        // 参考雷达掉线、另一台在线 → 立刻切过去（另一台一直在后台保持在线，不用等保持期）。
        assert(!select_front_imu_source(false, true, false, true));
        // 参考雷达刚回来但还没过保持期（fresh 但不在线）、另一台在线 → 继续用另一台，别来回切。
        assert(!select_front_imu_source(false, true, true, true));
        // 两台都不在线：按新鲜度兜底，不能两头都不用。
        assert(select_front_imu_source(false, false, true, false));
        assert(!select_front_imu_source(false, false, false, true));
        assert(select_front_imu_source(false, false, false, false));  // 都没数据 → 返回参考雷达（空）
    }

}// namespace

int main() {
    test_transform_matrix();
    test_imu_rotation();
    test_imu_source_selection();
    test_health_gate_stale_and_recover();
    test_health_gate_flapping_and_zero_hold();
    std::printf("merge_failover tests passed\n");
    return 0;
}
