#pragma once
#ifndef BIRDVIEW_VO_SE2_H
#define BIRDVIEW_VO_SE2_H

#include <eigen3/Eigen/Core>
#include <opencv2/core/core.hpp>

#include "se3Quat/se3quat.h"

namespace birdview
{

struct SE2
{
    SE2();
    SE2(double _x, double _y ,double _theta);
    ~SE2();

    SE2 inv() const;
    SE2 operator+(const SE2& that) const;
    SE2 operator-(const SE2& that) const;
    cv::Point2f operator*(const cv::Point2f& pt) const;

    cv::Mat toCvSE2() const;
    cv::Mat toCvSE3() const;

    static SE2 fromCvSE2(const cv::Mat& mat);
    static SE2 fromCvSE3(const cv::Mat& mat);
    static SE2 fromCvSE2Rt(const cv::Mat& R, const cv::Mat& t);

    Eigen::Matrix3d toMatrix3d() const;
    Eigen::Matrix4d toMatrix4d() const;

    static SE2 fromMatrix3d(const Eigen::Matrix3d& mat);
    static SE2 fromMatrix4d(const Eigen::Matrix4d& mat);

    SE3::SE3Quat toSE3Quat() const;

    static double normalize_angle(double theta);

    double x;
    double y;
    double theta;
};

}  // namespace birdview

#endif  // BIRDVIEW_VO_SE2_H