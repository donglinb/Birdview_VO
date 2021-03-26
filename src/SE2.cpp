#include "SE2.h"

namespace birdview
{

double SE2::normalize_angle(double theta)
{
  if (theta >= -M_PI && theta < M_PI)
    return theta;

  double multiplier = floor(theta / (2*M_PI));
  theta = theta - multiplier*2*M_PI;
  if (theta >= M_PI)
    theta -= 2*M_PI;
  if (theta < -M_PI)
    theta += 2*M_PI;

  return theta;
}

SE2::SE2()
  : x(0.0), y(0.0), theta(0.0)
{
    ;
}

SE2::SE2(double _x, double _y ,double _theta)
  : x(_x), y(_y), theta(normalize_angle(_theta))
{
    ;
}

SE2::~SE2()
{
    ;
}

SE2 SE2::inv() const
{
    double c = std::cos(theta);
    double s = std::sin(theta);
    return SE2(-c*x-s*y, s*x-c*y, -theta);
}

SE2 SE2::operator+(const SE2& that) const
{
    double c = std::cos(theta);
    double s = std::sin(theta);
    double _x = x + that.x*c - that.y*s;
    double _y = y + that.x*s + that.y*c;
    double _theta = normalize_angle(theta + that.theta);
    return SE2(_x, _y, _theta);
}

// that.inv() + *this
SE2 SE2::operator-(const SE2& that) const
{
    double dx = x - that.x;
    double dy = y - that.y;
    double dth = normalize_angle(theta - that.theta);

    double c = std::cos(that.theta);
    double s = std::sin(that.theta);
    return SE2(c*dx+s*dy, -s*dx+c*dy, dth);
}

// current -> world
cv::Point2f SE2::operator*(const cv::Point2f &pt) const
{
    double c = std::cos(theta);
    double s = std::sin(theta);

    double xw = c * pt.x - s * pt.y + x;
    double yw = s * pt.x + c * pt.y + y;

    return cv::Point2f(xw, yw);
}

cv::Mat SE2::toCvSE2() const
{
    double c = cos(theta);
    double s = sin(theta);

    return (cv::Mat_<float>(3,3) <<
            c,-s, x,
            s, c, y,
            0, 0, 1);
}

cv::Mat SE2::toCvSE3() const
{
    double c = cos(theta);
    double s = sin(theta);

    return (cv::Mat_<float>(4,4) <<
            c,-s, 0, x,
            s, c, 0, y,
            0, 0, 1, 0,
            0, 0, 0, 1);
}

SE2 SE2::fromCvSE2(const cv::Mat& mat)
{
    double yaw = std::atan2(mat.at<float>(1,0), mat.at<float>(0,0));
    double theta = normalize_angle(yaw);
    double x = mat.at<float>(0,2);
    double y = mat.at<float>(1,2);
    return SE2(x,y,theta);
}

SE2 SE2::fromCvSE3(const cv::Mat &mat)
{
    double yaw = std::atan2(mat.at<float>(1,0), mat.at<float>(0,0));
    double theta = normalize_angle(yaw);
    double x = mat.at<float>(0,3);
    double y = mat.at<float>(1,3);
    return SE2(x,y,theta);
}

SE2 SE2::fromCvSE2Rt(const cv::Mat& R, const cv::Mat& t)
{
    double yaw = std::atan2(R.at<float>(1,0), R.at<float>(0,0));
    double theta = normalize_angle(yaw);
    double x = t.at<float>(0);
    double y = t.at<float>(1);
    return SE2(x,y,theta);
}

Eigen::Matrix3d SE2::toMatrix3d() const
{
    double c = cos(theta);
    double s = sin(theta);

    Eigen::Matrix3d mat;
    mat <<  c,-s, x,
            s, c, y,
            0, 0, 1;
    return mat;
}

Eigen::Matrix4d SE2::toMatrix4d() const
{
    double c = cos(theta);
    double s = sin(theta);

    Eigen::Matrix4d mat;
    mat <<  c,-s, 0, x,
            s, c, 0, y,
            0, 0, 1, 0,
            0, 0, 0, 1;
    return mat;
}

SE2 SE2::fromMatrix3d(const Eigen::Matrix3d& mat)
{
    double yaw = std::atan2(mat(1,0), mat(0,0));
    double theta = normalize_angle(yaw);
    double x = mat(0,2);
    double y = mat(1,2);
    return SE2(x,y,theta);
}

SE2 SE2::fromMatrix4d(const Eigen::Matrix4d& mat)
{
    double yaw = std::atan2(mat(1,0), mat(0,0));
    double theta = normalize_angle(yaw);
    double x = mat(0,3);
    double y = mat(1,3);
    return SE2(x,y,theta);
}

SE3::SE3Quat SE2::toSE3Quat() const
{
    double c = cos(theta);
    double s = sin(theta);

    Eigen::Matrix3d R;
    R <<    c,-s, 0,
            s, c, 0,
            0, 0, 1;
    Eigen::Vector3d t(x, y, 0.0);

    return SE3::SE3Quat(R, t);
}

}  // namespace birdview