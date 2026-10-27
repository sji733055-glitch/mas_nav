#include "small_point_lio/parameters.h"
#include "small_point_lio/preprocess.h"

#include <gtest/gtest.h>

namespace small_point_lio {
namespace {

common::Point point(double stamp, float distance = 1.0F) {
    return {stamp, Eigen::Vector3f(distance, 0.0F, 0.0F)};
}

TEST(DenseFrameBoundary, CombinesCallbackFragmentsIntoCompleteInputFrames) {
    Parameters params{};
    params.point_filter_num = 1;
    params.min_distance_squared = 0.25F;
    params.max_distance_squared = 100.0F;
    params.blind_center = Eigen::Vector3f::Zero();
    params.space_downsample = false;

    Preprocess preprocess;
    preprocess.parameters = &params;
    preprocess.on_point_cloud_callback({point(1.0), point(1.1), point(1.2)});
    preprocess.on_point_cloud_callback({point(2.0), point(2.1)});
    ASSERT_EQ(preprocess.dense_frame_remaining.size(), 2U);

    EXPECT_FALSE(preprocess.finish_dense_point());
    EXPECT_FALSE(preprocess.finish_dense_point());
    EXPECT_TRUE(preprocess.finish_dense_point());
    EXPECT_FALSE(preprocess.finish_dense_point());
    EXPECT_TRUE(preprocess.finish_dense_point());
    EXPECT_TRUE(preprocess.dense_frame_remaining.empty());
    EXPECT_TRUE(preprocess.dense_point_deque.empty());

    preprocess.on_point_cloud_callback({point(3.0, 0.1F), point(3.1)});
    ASSERT_EQ(preprocess.dense_frame_remaining.size(), 1U);
    EXPECT_TRUE(preprocess.finish_dense_point());
    preprocess.reset();
    EXPECT_TRUE(preprocess.dense_frame_remaining.empty());
}

}  // namespace
}  // namespace small_point_lio
