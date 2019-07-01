#pragma once
#ifndef MYSLAM_FEATURE_H
#define MYSLAM_FEATURE_H

#include "myslam/common_include.h"

#include <memory>
#include <opencv2/features2d.hpp>

namespace myslam
{

class Frame;
class MapPoint;

/**
 * 2D feature point
 * associated with a map point after triangulation
 */
class Feature
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;

    Feature()
    {
    }

    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
        : frame_(frame), kp_(kp)
    {
    }

public:
    std::weak_ptr<Frame> frame_;        // frame which contains this feature
    cv::KeyPoint kp_;             // extracted 2D keypoint
    std::weak_ptr<MapPoint> map_point_; // corresponding map point
    bool is_outlier_ = false;           // flag to check if it is a outlier
    bool is_on_left_image_ = true;      // flag to check if extracted from left image
};
} // namespace myslam

#endif // MYSLAM_FEATURE_H
