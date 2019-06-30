#pragma once
#ifndef MYSLAM_DATASET_H
#define MYSLAM_DATASET_H

#include "myslam/camera.h"
#include "myslam/common_include.h"
#include "myslam/frame.h"

namespace myslam
{

/**
 * Dataset Reader
 * Passing configuration file path when constructed.
 * "dataset_dir" in the configuration is the path.
 * Able to obtain Camera and next frame after Init()
 */
class Dataset
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Dataset> Ptr;

    Dataset(const std::string &dataset_path);

    bool Init();

    // create and return the next frame containing the stereo images
    Frame::Ptr NextFrame();

    /// get camera by id
    Camera::Ptr GetCamera(int camera_id) const
    {
        return cameras_.at(camera_id);
    }

private:
    std::string dataset_path_;
    int current_index_ = 0;
    std::vector<Camera::Ptr> cameras_;
    std::vector<double> time_stamps_;
};

} // namespace myslam

#endif