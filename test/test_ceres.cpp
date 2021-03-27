//
// Created by mrwhite on 2021/3/25.
//

#include <iostream>
#include <random>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "SE2.h"
#include "cost_functor.h"

using namespace ceres;

struct CostFunctor
{
    template<typename T>
    bool operator()(const T* const x, T* residual) const
    {
        residual[0] = 10.0 - x[0];
        return true;
    }
};

struct NumericDiffCostFunctor
{
    bool operator()(const double* const x, double* residual) const
    {
        residual[0] = 10.0 - x[0];
        return true;
    }
};

class QuadraticCostFunction : public ceres::SizedCostFunction<1,1>
{
public:
    virtual ~QuadraticCostFunction(){}
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
    {
        const double x = parameters[0][0];
        residuals[0] = 10 - x;

        if(jacobians && jacobians[0])
        {
            jacobians[0][0] = -1;
        }

        return true;
    }
};

struct PowellCostFunctor
{
    template<typename T>
    bool operator()(const T* const x, T* residual) const
    {
        residual[0] = x[0] + T(10) * x[1];
        residual[1] = sqrt(5.0) * (x[2] - x[3]);
        residual[2] = (x[1] - T(2) * x[2]) * (x[1] - T(2) * x[2]);
        residual[3] = sqrt(10.0) * (x[0] - x[3]) * (x[0] - x[3]);
        return true;
    }
};

void GenerateData(int num_points, birdview::SE2& pose_bw, birdview::SE2& pose_bw_noise,  std::vector<cv::Point2f>& points_w, std::vector<cv::Point2f>& points_b)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_xy(0.0,5.0);
    std::uniform_real_distribution<> dis_theta(0.0, M_PI);
    std::uniform_real_distribution<> dis_point(-10.0,10.0);
    std::normal_distribution<> dis_noise(0.0,0.5);

    pose_bw = birdview::SE2(dis_xy(gen),dis_xy(gen),dis_theta(gen)).inv();
    pose_bw_noise = birdview::SE2(pose_bw.x + dis_noise(gen), pose_bw.y + dis_noise(gen), pose_bw.theta + dis_noise(gen));
    for(int i = 0; i < num_points; i++)
    {
        cv::Point2f p_w(dis_point(gen),dis_point(gen));
        points_w.push_back(p_w);
        points_b.push_back(pose_bw * p_w);
    }
}

std::ostream& operator<<(std::ostream& o, const birdview::SE2& pose)
{
    o << "x = " << pose.x << ", y = " << pose.y << ", theta = " << pose.theta;
}

int main(int argc, char** argv)
{
    const int num_points = 50;
    birdview::SE2 pose_bw, pose_bw_noise;
    std::vector<cv::Point2f> points_w, points_b;
    GenerateData(num_points,pose_bw,pose_bw_noise,points_w,points_b);

    std::cout << "Groundtruth: " << std::endl;
    std::cout << pose_bw << std::endl;
    std::cout << "Initial: " << std::endl;
    std::cout << pose_bw_noise << std::endl;

    double points[num_points][2]{};

    ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();

    Problem problem;
    for(int i = 0; i < num_points; i++)
    {
        points[i][0] = points_w[i].x;
        points[i][1] = points_w[i].y;

        CostFunction* cost_function = ReprojectionError::Create(points_b[i]);
        problem.AddResidualBlock(cost_function,nullptr,&pose_bw_noise.x,&pose_bw_noise.y,&pose_bw_noise.theta,points[i]);
        problem.SetParameterBlockConstant(points[i]);
    }
    problem.SetParameterization(&pose_bw_noise.theta, angle_local_parameterization);

    Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options,&problem,&summary);

    std::cout << summary.BriefReport() << std::endl;

    std::cout << "Final: " << std::endl;
    std::cout << pose_bw_noise << std::endl;

    return 0;
}

void print(int x,...)
{
    ;
}