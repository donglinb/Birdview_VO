#pragma once
#ifndef BIRDVIEW_VO_LINE_GEOMETRY_H
#define BIRDVIEW_VO_LINE_GEOMETRY_H

#include <eigen3/Eigen/Dense>

namespace birdview
{

using namespace Eigen;

typedef Matrix<double,6,1> Vector6d;
typedef Matrix<double,8,1> Vector8d;
typedef Matrix<double,6,6> Matrix6d;

class LineGeometry
{
public:
    static Vector4d line_to_orth(Vector6d line);
    static Vector6d orth_to_line(Vector4d orth);
    static Vector4d plk_to_orth(Vector6d plk);
    static Vector6d orth_to_plk(Vector4d orth);

    static Vector4d pi_from_ppp(Vector3d x1, Vector3d x2, Vector3d x3);
    static Vector6d pipi_plk( Vector4d pi1, Vector4d pi2);
    static Vector3d plucker_origin(Vector3d n, Vector3d v);
    static Matrix3d skew_symmetric( Vector3d v );

    static Vector3d point_from_pose( Eigen::Matrix3d Rcw, Eigen::Vector3d tcw, Vector3d pt_c );
    static Vector3d point_to_pose( Eigen::Matrix3d Rcw, Eigen::Vector3d tcw , Vector3d pt_w );
    static Vector6d line_to_pose(Vector6d line_w, Eigen::Matrix3d Rcw, Eigen::Vector3d tcw);
    static Vector6d line_from_pose(Vector6d line_c, Eigen::Matrix3d Rcw, Eigen::Vector3d tcw);

    static Vector6d plk_to_pose( Vector6d plk_w, Eigen::Matrix3d Rcw, Eigen::Vector3d tcw );
    static Vector6d plk_from_pose( Vector6d plk_c, Eigen::Matrix3d Rcw, Eigen::Vector3d tcw );

};


}  // namespace birdview

#endif // BIRDVIEW_VO_LINE_GEOMETRY_H
