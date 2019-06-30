#include "myslam/algorithm.h"
#include "myslam/common_include.h"

#include <gtest/gtest.h>

TEST(MyslamTest, Triangulation)
{
    Vec3 p_w(30, 20, 10), p_w_estimate;
    std::vector<SE3> poses{
        SE3(Eigen::Quaterniond(0, 0, 0, 1), Vec3(0, 0, 0)),
        SE3(Eigen::Quaterniond(0, 0, 0, 1), Vec3(0, -10, 0)),
        SE3(Eigen::Quaterniond(0, 0, 0, 1), Vec3(0, 10, 0)),
    };
    std::vector<Vec3> points;
    for (size_t i = 0; i < poses.size(); ++i)
    {
        Vec3 p_c = poses[i] * p_w;
        p_c /= p_c[2];
        points.emplace_back(p_c);
    }

    EXPECT_TRUE(myslam::triangulation(poses, points, p_w_estimate));
    EXPECT_NEAR(p_w[0], p_w_estimate[0], 0.01);
    EXPECT_NEAR(p_w[1], p_w_estimate[1], 0.01);
    EXPECT_NEAR(p_w[2], p_w_estimate[2], 0.01);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}