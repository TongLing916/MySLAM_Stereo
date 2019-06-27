#pragma once
#ifndef MYSLAM_BACKEND_H
#define MYSLAM_BACKEND_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"

namespace myslam
{

class Map;

/**
 * Backend
 * Own thread to update map
 * Map update decided by Frontend
 */
class Backend
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Backend> Ptr;

    // start thread for optimization
    Backend();

    // set left and right cameras for acquisition of intrinsics
    void SetCameras(Camera::Ptr left, Camera::Ptr right)
    {
        cam_left_ = left;
        cam_right_ = right;
    }

    void SetMap(std::shared_ptr<Map> map)
    {
        map_ = map;
    }

    // update map, start optmization
    void UpdateMap();

    // stop thread of backend
    void Stop();

private:
    // thread backend
    void BackendLoop();

    // optimize given keyframes and landmarks
    void Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks);

    std::shared_ptr<Map> map_;
    std::thread backend_thread_;
    std::mutex data_mutex_;

    std::condition_variable map_update_;
    std::atomic<bool> backend_running_;

    Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;
};

} // namespace myslam

#endif // MYSLAM_BACKEND_H