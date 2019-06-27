#pragma once
#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include "myslam/camera.h"
#include "myslam/common_include.h"

namespace myslam
{

// forward declare
class MapPoint;
class Feature;

/**
 * Frame
 * Every frame gets a unique id
 * If it is a keyframe, it gets a keyframe id additionally
 */
class Frame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;

    Frame()
    {
    }

    Frame(long id, double time_stamp, const SE3 &pose, const Mat &left, const Mat &right);

    // set and get pose, thread safe
    SE3 Pose()
    {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        return pose_;
    }

    void SetPose(const SE3 &pose)
    {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        pose_ = pose;
    }

    // set keyframe and assign keyframe id
    void SetKeyFrame();

    // factory pattern to assign frame id
    static std::shared_ptr<Frame> CreateFrame();

public:
    unsigned long id_ = 0;          // id of this frame
    unsigned long keyframe_id_ = 0; // id of key frame
    bool is_keyframe_ = false;      // if it is a keyframe
    double time_stamp_;             // timestamp of this frame, no use for now
    SE3 pose_;                      // Camera pose: T_c_w
    std::mutex pose_mutex_;         // pose lock
    cv::Mat left_img_, right_img_;  // stereo images

    // features extracted in left image
    std::vector<std::shared_ptr<Feature>> features_left_;
    // corresponding features in right image, set to nullptr if no corresponding
    std::vector<std::shared_ptr<Feature>> features_right_;
};

} // namespace myslam

#endif // MYSLAM_FRAME_H
