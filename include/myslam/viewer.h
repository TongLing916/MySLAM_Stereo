#pragma once
#ifndef MYSLAM_VIEWER_H
#define MYSLAM_VIEWER_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"

#include <pangolin/pangolin.h>
#include <thread>

namespace myslam
{

/**
 * Visualization of camera, keyframe and landmark
 */
class Viewer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Viewer> Ptr;

    Viewer();

    void SetMap(Map::Ptr map)
    {
        map_ = map;
    }

    void Close();

    void AddCurrentFrame(Frame::Ptr current_frame);

    void UpdateMap();

private:
    void ThreadLoop();

    void DrawFrame(Frame::Ptr frame, const float *color);

    void DrawMapPoints();

    void FollowCurrentFrame(pangolin::OpenGlRenderState &vis_camera);

    /// plot the features in current frame into an image
    cv::Mat PlotFrameImage();

    Frame::Ptr current_frame_ = nullptr;
    Map::Ptr map_ = nullptr;

    std::thread viewer_thread_;
    bool viewer_running_ = true;

    std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;
    std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks_;
    bool map_updated_ = false;

    std::mutex viewer_data_mutex_;
};
} // namespace myslam

#endif // MYSLAM_VIEWER_H
