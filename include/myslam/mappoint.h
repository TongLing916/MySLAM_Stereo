#pragma once
#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H

#include "myslam/common_include.h"

namespace myslam
{

class Frame;
class Feature;

/**
 * Map Point (Landmark)
 * obtained by triangulation of keypoints
 */
class MapPoint
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<MapPoint> Ptr;

    MapPoint()
    {
    }

    MapPoint(long id, Vec3 position);

    Vec3 Pos()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return pos_;
    }

    void SetPos(const Vec3 &pos)
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        pos_ = pos;
    };

    void AddObservation(std::shared_ptr<Feature> feature)
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        observations_.emplace_back(feature);
        ++observed_times_;
    }

    void RemoveObservation(std::shared_ptr<Feature> feat);

    std::list<std::weak_ptr<Feature>> GetObs()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return observations_;
    }

    // factory function
    static MapPoint::Ptr CreateNewMappoint();

public:
    unsigned long id_ = 0; // ID
    bool is_outlier_ = false;
    Vec3 pos_ = Vec3::Zero(); // position in world frame
    std::mutex data_mutex_;
    int observed_times_ = 0; // times of being observed by feature matching algorithm
    std::list<std::weak_ptr<Feature>> observations_;
};

} // namespace myslam

#endif // MYSLAM_MAPPOINT_H
