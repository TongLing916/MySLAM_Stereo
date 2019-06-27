#pragma once
#ifndef MYSLAM_CAMERA_H
#define MYSLAM_CAMERA_H

#include "myslam/common_include.h"

namespace myslam
{

// Pinhole stereo camera model
class Camera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Camera> Ptr;

    Camera();

    Camera(double fx, double fy, double cx, double cy, double baseline, const SE3 &pose)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose)
    {
        pose_inv_ = pose_.inverse();
    }

    SE3 pose() const
    {
        return pose_;
    }

    // return intrinsic matrix
    Mat33 K() const
    {
        Mat33 k;
        k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
        return k;
    }

    // coordinate transform: world, camera, pixel
    Vec3 World2Camera(const Vec3 &p_w, const SE3 &T_c_w);

    Vec3 Camera2World(const Vec3 &p_c, const SE3 &T_c_w);

    Vec2 Camera2Pixel(const Vec3 &p_c);

    Vec3 Pixel2Camera(const Vec2 &p_p, double depth = 1);

    Vec3 Pixel2World(const Vec2 &p_p, const SE3 &T_c_w, double depth = 1);

    Vec2 World2Pixel(const Vec3 &p_w, const SE3 &T_c_w);

public:
    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0,
           baseline_ = 0; // Camera intrinsics
    SE3 pose_;            // extrinsic, from stereo camera to single camera
    SE3 pose_inv_;        // inverse of extrinsics
};

} // namespace myslam
#endif // MYSLAM_CAMERA_H
