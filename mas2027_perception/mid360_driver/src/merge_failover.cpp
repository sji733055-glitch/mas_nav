/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "mid360_driver/merge_failover.hpp"

namespace mid360_driver {

    MergeTransform make_merge_transform(const std::vector<double> &xyz_rpy) {
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

    std::vector<Point> transform_merge_points(const std::vector<Point> &points, const MergeTransform &tf) {
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

    ImuMsg rotate_imu_to_front(const ImuMsg &imu, const MergeTransform &tf) {
        ImuMsg output = imu;
        const float wx = tf.r00 * imu.angular_velocity_x + tf.r01 * imu.angular_velocity_y + tf.r02 * imu.angular_velocity_z;
        const float wy = tf.r10 * imu.angular_velocity_x + tf.r11 * imu.angular_velocity_y + tf.r12 * imu.angular_velocity_z;
        const float wz = tf.r20 * imu.angular_velocity_x + tf.r21 * imu.angular_velocity_y + tf.r22 * imu.angular_velocity_z;
        const float ax = tf.r00 * imu.linear_acceleration_x + tf.r01 * imu.linear_acceleration_y + tf.r02 * imu.linear_acceleration_z;
        const float ay = tf.r10 * imu.linear_acceleration_x + tf.r11 * imu.linear_acceleration_y + tf.r12 * imu.linear_acceleration_z;
        const float az = tf.r20 * imu.linear_acceleration_x + tf.r21 * imu.linear_acceleration_y + tf.r22 * imu.linear_acceleration_z;

        // 杆臂：后 IMU 指向前 IMU，前系 = 前原点在后系中的位置取反再转到前系 = -t。
        const float dx = -tf.tx;
        const float dy = -tf.ty;
        const float dz = -tf.tz;
        // ω×(ω×d) = ω(ω·d) − d|ω|²（离心项，随陀螺可算，无需微分）。
        const float omega_dot_d = wx * dx + wy * dy + wz * dz;
        const float omega_squared = wx * wx + wy * wy + wz * wz;

        output.angular_velocity_x = wx;
        output.angular_velocity_y = wy;
        output.angular_velocity_z = wz;
        output.linear_acceleration_x = ax + wx * omega_dot_d - dx * omega_squared;
        output.linear_acceleration_y = ay + wy * omega_dot_d - dy * omega_squared;
        output.linear_acceleration_z = az + wz * omega_dot_d - dz * omega_squared;
        return output;
    }

}// namespace mid360_driver
