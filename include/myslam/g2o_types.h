#pragma once
#ifndef MYSLAM_G2O_TYPES_H
#define MYSLAM_G2O_TYPES_H

#include "myslam/common_include.h"

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

namespace myslam
{

// Vertex representing camera pose for optimization
class VertexPose : public g2o::BaseVertex<6, SE3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override
    {
        _estimate = SE3();
    }

    // left multiplication on SE3
    virtual void oplusImpl(const double *update) override
    {
        Vec6 update_eigen;
        update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = SE3::exp(update_eigen) * _estimate;
    }

    virtual bool read(std::istream &in) override
    {
        return true;
    }

    virtual bool write(std::ostream &out) const override
    {
        return true;
    }
};

// Vertex representing landmark's position for optimization
class VertexXYZ : public g2o::BaseVertex<3, Vec3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override
    {
        _estimate = Vec3::Zero();
    }

    virtual void oplusImpl(const double *update) override
    {
        _estimate[0] += update[0];
        _estimate[1] += update[1];
        _estimate[2] += update[2];
    }

    virtual bool read(std::istream &in) override
    {
        return true;
    }

    virtual bool write(std::ostream &out) const override
    {
        return true;
    }
};

/**
 * Edge to optimize camera pose
 * Member: (1) observed landmark's position (Vec3).
 *         (2) camera intrinsic matrix. (Mat33)
 * Error: reprojection error (Vec2)
 * */
class EdgeProjectionPoseOnly : public g2o::BaseUnaryEdge<2, Vec2, VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjectionPoseOnly(const Vec3 &p_w, const Mat33 &K)
        : _p_w(p_w), _K(K)
    {
    }

    // compute reprojection error
    virtual void computeError() override
    {
        const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
        SE3 T = v->estimate();
        Vec3 pos_pixel = _K * (T * _p_w);
        pos_pixel /= pos_pixel[2];
        _error = _measurement - pos_pixel.head<2>();
    }

    // Compute Jacobian matrix of reprojection error [delta_u delta_v] with respect to vertices
    virtual void linearizeOplus() override
    {
        const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
        SE3 T = v->estimate();
        Vec3 p_c = T * _p_w;
        double fx = _K(0, 0);
        double fy = _K(1, 1);
        double X_c = p_c[0];
        double Y_c = p_c[1];
        double Z_c = p_c[2];
        double Z_c_inv = 1.0 / (Z_c + 1e-18);
        double Z_c_inv2 = Z_c_inv * Z_c_inv;

        // Compute Jacobian with respect to the first vertex (camera pose)
        _jacobianOplusXi << -fx * Z_c_inv, 0, fx * X_c * Z_c_inv2, fx * X_c * Y_c * Z_c_inv2, -fx - fx * X_c * X_c * Z_c_inv2, fx * Y_c * Z_c_inv,
            0, -fy * Z_c_inv, fy * Y_c * Z_c_inv2, fy + fy * Y_c * Y_c * Z_c_inv2, -fy * X_c * Y_c * Z_c_inv2, -fy * X_c * Z_c_inv;
    }

    virtual bool read(std::istream &in) override
    {
        return true;
    }

    virtual bool write(std::ostream &out) const override
    {
        return true;
    }

private:
    Vec3 _p_w;
    Mat33 _K;
};

/**
 * Edge to optimize camera pose (SE3) and landmark's position (Vec3)
 * Member: (1) camera intrinsic matrix (Mat33)
 *         (2) camera extrinsic matrix (SE3)
 * Error: reprojection error (Vec2)
 * */
class EdgeProjection : public g2o::BaseBinaryEdge<2, Vec2, VertexPose, VertexXYZ>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /// assign camera parameters during construction
    EdgeProjection(const Mat33 &K, const SE3 &cam_ext)
        : _K(K), _cam_ext(cam_ext)
    {
    }

    // compute reprojection error
    virtual void computeError() override
    {
        const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
        const VertexXYZ *v1 = static_cast<VertexXYZ *>(_vertices[1]);
        SE3 T = v0->estimate();
        Vec3 pos_pixel = _K * (_cam_ext * (T * v1->estimate()));
        pos_pixel /= pos_pixel[2];
        _error = _measurement - pos_pixel.head<2>();
    }

    // compute Jacobian matrix of reprojection error [delta_u delta_v] with respect to vertices
    virtual void linearizeOplus() override
    {
        const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
        const VertexXYZ *v1 = static_cast<VertexXYZ *>(_vertices[1]);
        SE3 T = v0->estimate();
        Vec3 p_w = v1->estimate();
        Vec3 p_c = _cam_ext * T * p_w;
        double fx = _K(0, 0);
        double fy = _K(1, 1);
        double X_c = p_c[0];
        double Y_c = p_c[1];
        double Z_c = p_c[2];
        double Z_c_inv = 1.0 / (Z_c + 1e-18);
        double Z_c_inv2 = Z_c_inv * Z_c_inv;

        // <<Visual SLAM 14 lecture>> Page 167
        // Compute Jacobian with respect to the first vertex (camera pose)
        _jacobianOplusXi << -fx * Z_c_inv, 0, fx * X_c * Z_c_inv2, fx * X_c * Y_c * Z_c_inv2, -fx - fx * X_c * X_c * Z_c_inv2, fx * Y_c * Z_c_inv,
            0, -fy * Z_c_inv, fy * Y_c * Z_c_inv2, fy + fy * Y_c * Y_c * Z_c_inv2, -fy * X_c * Y_c * Z_c_inv2, -fy * X_c * Z_c_inv;

        // Compute Jacobian with respect to the second vertex (landmark's position)
        _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) * _cam_ext.rotationMatrix() * T.rotationMatrix();
    }

    virtual bool read(std::istream &in) override
    {
        return true;
    }

    virtual bool write(std::ostream &out) const override
    {
        return true;
    }

private:
    Mat33 _K;
    SE3 _cam_ext;
};

} // namespace myslam

#endif // MYSLAM_G2O_TYPES_H
