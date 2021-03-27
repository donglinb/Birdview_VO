//
// Created by mrwhite on 2021/3/26.
//

#ifndef BIRDVIEW_VO_COSTFUNCTORS_H
#define BIRDVIEW_VO_COSTFUNCTORS_H

#include <cmath>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <utility>
#include <ceres/jet.h>
#include <ceres/autodiff_local_parameterization.h>
#include <ceres/autodiff_cost_function.h>

// Normalizes the angle in radians between [-pi and pi).
template <typename T>
inline T NormalizeAngle(const T& angle_radians)
{
    // Use ceres::floor because it is specialized for double and Jet types.
    T two_pi(2.0 * M_PI);
    return angle_radians -
           two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}

template <typename T>
Eigen::Matrix<T, 2, 2> RotationMatrix2D(T yaw_radians)
{
    const T cos_yaw = ceres::cos(yaw_radians);
    const T sin_yaw = ceres::sin(yaw_radians);

    Eigen::Matrix<T, 2, 2> rotation;
    rotation << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;
    return rotation;
}

//template<typename T>
//using Vector3 =  Eigen::Matrix<T,3,1>;
//
//template<typename T>
//static Vector3<T> cross(const Vector3<T>& a, const Vector3<T>& b)
//{
//    // s1 = a2 * b3 - a3 * b2
//    // s2 = a3 * b1 - a1 * b3
//    // s3 = a1 * b2 - a2 * b1
//    T x = a[1] * b[2] - a[2] * b[1];
//    T y = a[2] * b[0] - a[0] * b[2];
//    T z = a[0] * b[1] - a[1] * b[0];
//    return {x, y, z};
//}
//
//template<typename T>
//static T norm(const Vector3<T>& x)
//{
//    T r = x[0] * x[0] + x[1] * x[1] + x[2] * x[2];
//    return ceres::sqrt(r);
//}

// Defines a local parameterization for updating the angle to be constrained in
// [-pi to pi).
class AngleLocalParameterization
{
public:
    template <typename T>
    bool operator()(const T* theta_radians, const T* delta_theta_radians, T* theta_radians_plus_delta) const
    {
        *theta_radians_plus_delta = NormalizeAngle(*theta_radians + *delta_theta_radians);
        return true;
    }

    static ceres::LocalParameterization* Create()
    {
        return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,1,1>);
    }
};

struct ReprojectionErrorFunctor
{
    explicit ReprojectionErrorFunctor(const cv::Point2f& pt)
            : pt(pt)
    {
        ;
    }

    template<typename T>
    bool operator()(const T* const pose_bw_x, const T* const pose_bw_y, const T* const pose_bw_theta,
                    const T* const point_w, T* residuals) const
    {
        Eigen::Map<Eigen::Matrix<T,2,1>> err(residuals);
        Eigen::Map<const Eigen::Matrix<T,2,1>> p_w(point_w);
        Eigen::Matrix<T,2,1> t_bw(*pose_bw_x, *pose_bw_y);

        Eigen::Matrix<T,2,1> p_b = RotationMatrix2D(*pose_bw_theta) * p_w + t_bw;
        err[0] = T(pt.x) - p_b[0];
        err[1] = T(pt.y) - p_b[1];

        return true;
    }

    static ceres::CostFunction* Create(const cv::Point2f& pt)
    {
        return (new ceres::AutoDiffCostFunction<ReprojectionErrorFunctor,2,1,1,1,2>(new ReprojectionErrorFunctor(pt)));
    }

protected:
    cv::Point2f pt;
};

struct ManhattanErrorFunctor
{
    ManhattanErrorFunctor(const cv::Point2f& normal_obs, double weight)
      : a(normal_obs.x), b(normal_obs.y), w(weight)
    {
        ab = sqrt(a * a + b * b);
    }

    template<typename T>
    bool operator()(const T* const pose_bw_theta, const T* const line_w, T* residuals) const
    {
        Eigen::Map<const Eigen::Matrix<T,2,1>> normal_w(line_w);
        Eigen::Matrix<T,2,1> normal_b = RotationMatrix2D(*pose_bw_theta) * normal_w;
        T len = ceres::sqrt(normal_b[0] * normal_b[0] + normal_b[1] * normal_b[1]);
        residuals[0] = T(1.0) - (T(a) * normal_b[0] + T(b) * normal_b[1]) / (T(ab) * len);
        residuals[0] = w * residuals[0];

        return true;
    }

    static ceres::CostFunction* Create(const cv::Point2f& normal_obs, double weight)
    {
        return (new ceres::AutoDiffCostFunction<ManhattanErrorFunctor,1,1,2>(new ManhattanErrorFunctor(normal_obs, weight)));
    }

protected:
    double a, b, ab;
    double w;
};

struct ParallelLineCostFunctor
{
    ParallelLineCostFunctor(double a, double b)
      : a(a), b(b)
    {
        ;
    }

    template<typename T>
    bool operator()(const T* const line, T* residuals) const
    {
        Eigen::Matrix<T,2,1> le(line[1], -line[0]);
        residuals[0] = ceres::abs(T(a) * le[0] + T(b) * le[1]) / ceres::sqrt(le[0] * le[0] + le[1] * le[1]);
        return true;
    }

    static ceres::CostFunction* Create(double a, double b)
    {
        return (new ceres::AutoDiffCostFunction<ParallelLineCostFunctor,1,2>(new ParallelLineCostFunctor(a,b)));
    }

protected:

    double a, b;
};

struct OrthogonalLineCostFunctor
{
    OrthogonalLineCostFunctor(double a, double b)
      : a(a), b(b)
    {
        ;
    }

    template<typename T>
    bool operator()(const T* const line, T* residuals) const
    {
        Eigen::Matrix<T,2,1> le(line[0], line[1]);
        residuals[0] = ceres::abs(T(a) * le[0] + T(b) * le[1]) / ceres::sqrt(le[0] * le[0] + le[1] * le[1]);
        return true;
    }

    static ceres::CostFunction* Create(double a, double b)
    {
        return (new ceres::AutoDiffCostFunction<OrthogonalLineCostFunctor,1,2>(new OrthogonalLineCostFunctor(a,b)));
    }

protected:

    double a,b;
};

#endif //BIRDVIEW_VO_COSTFUNCTORS_H
