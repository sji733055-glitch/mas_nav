#pragma once

#include <memory>
#include <Eigen/Dense>

namespace utils {
template<typename T>
class EMAFilter {
public:
    using Ptr = std::shared_ptr<EMAFilter<T>>;
    explicit EMAFilter(const double filter_ratio): filter_ratio_(filter_ratio) {}
    void initialize(const T& val) { value_ = val; initialized_ = true; }
    T value() const { return value_; };
    void reset() { initialized_ = false; }
    void update(const T& val) {
        if (!initialized_) {
            initialize(val);
            return;
        }
        if constexpr (std::is_base_of_v<Eigen::MatrixBase<std::decay_t<T>>, std::decay_t<T>>) { // 向量
            value_ = filter_ratio_ * value_ + (1 - filter_ratio_) * val;
        } else if constexpr (std::is_base_of_v<Eigen::QuaternionBase<std::decay_t<T>>, std::decay_t<T>>) { // 旋转
            value_ = value_.slerp(1 - filter_ratio_, val);
        } else if constexpr (std::is_same_v<Eigen::Isometry3d, std::decay_t<T>>) { // 3D位姿
            value_.translation() = filter_ratio_ * Eigen::Vector3d(value_.translation()) + (1 - filter_ratio_) * Eigen::Vector3d(val.translation());
            value_.linear() = Eigen::Quaterniond(value_.linear()).slerp(1 - filter_ratio_, Eigen::Quaterniond(val.linear())).toRotationMatrix();
        } else {
            static_assert(false, "unsupported type");
        }
    }

private:
    T value_;
    bool initialized_ = false;
    const double filter_ratio_ = 0; // 介于0-1之间，越大越稳定
};
}
