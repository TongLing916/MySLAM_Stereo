#pragma once
#ifndef MYSLAM_MAP_H
#define MYSLAM_MAP_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"

namespace myslam
{

/**
 * Map
 * Interaction with Map：
 * 1) Frontend invokes InsertKeyframe and InsertMapPoint to add new frame and map point
 * 2) Backend matintains Map's structure，determine and remove outlier etc.
 */
class Map
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Map> Ptr;
    typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
    typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;

    Map()
    {
    }

    void InsertKeyFrame(Frame::Ptr frame);

    void InsertMapPoint(MapPoint::Ptr map_point);

    LandmarksType GetAllMapPoints()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return landmarks_;
    }

    KeyframesType GetAllKeyFrames()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return keyframes_;
    }

    LandmarksType GetActiveMapPoints()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_landmarks_;
    }

    KeyframesType GetActiveKeyFrames()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_keyframes_;
    }

    // clean the map points which was observed 0 times.
    void CleanMap();

private:
    // set old keyframes to inactive
    void RemoveOldKeyframe();

    std::mutex data_mutex_;
    LandmarksType landmarks_;        // all landmarks
    LandmarksType active_landmarks_; // active landmarks
    KeyframesType keyframes_;        // all key-frames
    KeyframesType active_keyframes_; // all key-frames

    Frame::Ptr current_frame_ = nullptr;

    // settings
    size_t num_active_keyframes_ = 7; // number of keyframes which can be activated.
};

} // namespace myslam

#endif // MYSLAM_MAP_H
