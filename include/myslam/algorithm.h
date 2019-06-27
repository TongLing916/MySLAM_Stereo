#pragma once
#ifndef MYSLAM_ALGORITHM_H
#define MYSLAM_ALGORITHM_H

// algorithms used in myslam
#include "myslam/common_include.h"

namespace myslam
{

/**
 * linear triangulation with SVD 
 * @param poses   camera poses,
 * @param points  points in normalized image plane in camera coordinate system,
 * @param pos_w   triangulated point's position in world
 * @return true if success
 */
inline bool triangulation(const std::vector<SE3> &poses, const std::vector<Vec3> points, Vec3 &p_w)
{
    MatXX A(2 * poses.size(), 4);
    VecX b(2 * poses.size());
    b.setZero();
    for (size_t i = 0; i < poses.size(); ++i)
    {
        Mat34 m = poses[i].matrix3x4();

        // every point provides two equations (rows)
        // ref: <<MVG>> section 12.2 Page 312
        A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
        A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
    }

    // ref: <<MVG>> Algorithm A5.4 Page 593
    auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

    // recover (x,y,z) from (x_, y_, z_, w_) where x_^2 + y_^2 + z_^2 + w_^2 = 1
    p_w = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    // calculate the quality of the solution
    // the smaller sigma_4 / sigma_3 is, the better the result is.
    // ref: http://cmp.felk.cvut.cz/cmp/courses/TDV/2012W/lectures/tdv-2012-07-anot.pdf
    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2)
        return true;
    else
        return false;
}

// converters
inline Vec2 toVec2(const cv::Point2f p)
{
    return Vec2(p.x, p.y);
}

} // namespace myslam

#endif // MYSLAM_ALGORITHM_H
