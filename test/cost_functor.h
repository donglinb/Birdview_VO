//
// Created by mrwhite on 2021/3/26.
//

#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Core>
#include <ceres/autodiff_cost_function.h>
#include <ceres/autodiff_local_parameterization.h>

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

struct ReprojectionError
{
    explicit ReprojectionError(const cv::Point2f& pt)
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
        return (new ceres::AutoDiffCostFunction<ReprojectionError,2,1,1,1,2>(new ReprojectionError(pt)));
    }

protected:

    cv::Point2f pt;
};
