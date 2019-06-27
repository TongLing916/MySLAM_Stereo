#include "myslam/camera.h"

namespace myslam
{

Camera::Camera()
{
}

Vec3 Camera::World2Camera(const Vec3 &p_w, const SE3 &T_c_w)
{
    return pose_ * T_c_w * p_w;
}

Vec3 Camera::Camera2World(const Vec3 &p_c, const SE3 &T_c_w)
{
    return T_c_w.inverse() * pose_inv_ * p_c;
}

Vec2 Camera::Camera2Pixel(const Vec3 &p_c)
{
    return Vec2(
        fx_ * p_c(0, 0) / p_c(2, 0) + cx_,
        fy_ * p_c(1, 0) / p_c(2, 0) + cy_);
}

Vec3 Camera::Pixel2Camera(const Vec2 &p_p, double depth)
{
    return Vec3(
        (p_p(0, 0) - cx_) * depth / fx_,
        (p_p(1, 0) - cy_) * depth / fy_,
        depth);
}

Vec2 Camera::World2Pixel(const Vec3 &p_w, const SE3 &T_c_w)
{
    return Camera2Pixel(World2Camera(p_w, T_c_w));
}

Vec3 Camera::Pixel2World(const Vec2 &p_p, const SE3 &T_c_w, double depth)
{
    return Camera2World(Pixel2Camera(p_p, depth), T_c_w);
}

} // namespace myslam
