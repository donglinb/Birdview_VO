//
// Created by mrwhite on 2021/3/25.
//

#include "Optimizer.h"

#include <map>
#include <eigen3/Eigen/Core>
#include <ceres/ceres.h>

#include "CostFunctors.h"
#include "SE2.h"
#include "KeyLineGeometry.h"

namespace birdview
{

using namespace ceres;

void Optimizer::PoseOptimization(const KeyFramePtr& pKF)
{
    SE2 Tbw = pKF->GetPoseTbw();
    const std::map<MapPointPtr, int> observations = pKF->GetObservations();
    const int N = observations.size();

    LocalParameterization* angle_local_parameter = AngleLocalParameterization::Create();
    LossFunction* huber_loss = new HuberLoss(1.0);
    Problem problem;

    std::vector<Eigen::Vector2d> points(N);
    int count = 0;
    for(const auto& it : observations)
    {
        const cv::Point2f pos = it.first->GetPos();
        const cv::Point2f pt = pKF->GetKeyPointXY(it.second);
        points[count] = Eigen::Vector2d(pos.x, pos.y);

        CostFunction* cost_function = ReprojectionErrorFunctor::Create(pt);
        problem.AddResidualBlock(cost_function, huber_loss, &Tbw.x, &Tbw.y, &Tbw.theta, points[count].data());
        problem.SetParameterBlockConstant(points[count].data());

        count++;
    }
    problem.SetParameterization(&Tbw.theta, angle_local_parameter);

    Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    Solver::Summary summary;
    Solve(options,&problem,&summary);

    if(summary.IsSolutionUsable())
    {
        pKF->SetPoseTbw(Tbw);
    }
}

void Optimizer::PoseOptimization(const FramePtr &pFrame, const std::vector<MapPointPtr> &vpLocalMapPoints,
                                 const std::vector<cv::DMatch> &vMatches)
{
    SE2 Tbw = pFrame->GetPoseTbw();
    int N = vMatches.size();

    LocalParameterization* angle_local_parameter = AngleLocalParameterization::Create();
    LossFunction* huber_loss = new HuberLoss(1.0);
    Problem problem;

    std::vector<Eigen::Vector2d> points(N);
    int count = 0;
    for(const cv::DMatch& m : vMatches)
    {
        const cv::Point2f pt = pFrame->GetKeyPointXY(m.queryIdx);
        const cv::Point2f pos = vpLocalMapPoints[m.trainIdx]->GetPos();
        points[count] = Eigen::Vector2d(pos.x, pos.y);

        CostFunction* cost_function = ReprojectionErrorFunctor::Create(pt);
        problem.AddResidualBlock(cost_function, huber_loss, &Tbw.x, &Tbw.y, &Tbw.theta, points[count].data());
        problem.SetParameterBlockConstant(points[count].data());
        count++;
    }
    problem.SetParameterization(&Tbw.theta, angle_local_parameter);

    Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    Solver::Summary summary;
    Solve(options,&problem,&summary);

    if(summary.IsSolutionUsable())
    {
        pFrame->SetPoseTbw(Tbw);
    }
}

void Optimizer::LocalBundleAdjustment(const MapPtr& pMap)
{
    std::map<KeyFramePtr, SE2, KeyFrame::IdLessThan> localKeyFramePoses;
    std::map<MapPointPtr, Eigen::Vector2d, MapPoint::IdLessThan,
        Eigen::aligned_allocator<std::pair<MapPointPtr, Eigen::Vector2d>>> localMapPoints;

    const std::vector<KeyFramePtr> vpLocalKFs = pMap->GetLocalKeyFrames();

    LocalParameterization* angle_local_parameter = AngleLocalParameterization::Create();
    LossFunction* huber_loss = new HuberLoss(1.0);
    Problem problem;

    for(const KeyFramePtr& kf : vpLocalKFs)
    {
        localKeyFramePoses[kf] = kf->GetPoseTbw();
        SE2& Tbw = localKeyFramePoses[kf];
        const std::map<MapPointPtr, int> observations = kf->GetObservations();

        for(const auto& it : observations)
        {
            if(localMapPoints.find(it.first) == localMapPoints.end())
            {
                const cv::Point2d pos = it.first->GetPos();
                localMapPoints[it.first] = Eigen::Vector2d(pos.x, pos.y);
            }

            const cv::Point2f pt = kf->GetKeyPointXY(it.second);
            CostFunction* cost_function = ReprojectionErrorFunctor::Create(pt);
            problem.AddResidualBlock(cost_function, huber_loss, &Tbw.x, &Tbw.y, &Tbw.theta, localMapPoints[it.first].data());
        }

        problem.SetParameterization(&Tbw.theta, angle_local_parameter);
    }

    Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    Solver::Summary summary;
    Solve(options,&problem,&summary);

    if(!summary.IsSolutionUsable())
    {
        return;
    }

    // update keyframe poses and map points
    for(auto& kf : localKeyFramePoses)
    {
        kf.first->SetPoseTbw(kf.second);
    }
    for(auto& mp : localMapPoints)
    {
        mp.first->SetPos(cv::Point2f(mp.second[0],mp.second[1]));
    }
}

void Optimizer::PoseOptimization(const KeyFramePtr &pKF, const Line& major_line)
{
    SE2 Tbw = pKF->GetPoseTbw();
    const std::map<MapPointPtr, int> observations = pKF->GetObservations();
    const int N = observations.size();

    LocalParameterization* angle_local_parameter = AngleLocalParameterization::Create();
    LossFunction* huber_loss = new HuberLoss(1.0);
    Problem problem;

    std::vector<Eigen::Vector2d> points(N);
    int count = 0;
    for(const auto& it : observations)
    {
        const cv::Point2f pos = it.first->GetPos();
        const cv::Point2f pt = pKF->GetKeyPointXY(it.second);
        points[count] = Eigen::Vector2d(pos.x, pos.y);

        CostFunction* cost_function = ReprojectionErrorFunctor::Create(pt);
        problem.AddResidualBlock(cost_function, huber_loss, &Tbw.x, &Tbw.y, &Tbw.theta, points[count].data());
        problem.SetParameterBlockConstant(points[count].data());

        count++;
    }
    problem.SetParameterization(&Tbw.theta, angle_local_parameter);

    // manhattan
    Eigen::Vector2d major_normal_w(major_line.le.x, major_line.le.y);
    Line major_line_c;
    pKF->GetMajorLine(major_line_c);
    CostFunction* cost_manhattan = ManhattanErrorFunctor::Create(cv::Point2f(major_line_c.le.x, major_line_c.le.y), 500.0);
    problem.AddResidualBlock(cost_manhattan, nullptr,&Tbw.theta,major_normal_w.data());
    problem.SetParameterBlockConstant(major_normal_w.data());

    Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    Solver::Summary summary;
    Solve(options,&problem,&summary);

    if(summary.IsSolutionUsable())
    {
        pKF->SetPoseTbw(Tbw);
    }
}

void Optimizer::LocalBundleAdjustment(const MapPtr &pMap, Line& major_line)
{
    std::map<KeyFramePtr, SE2, KeyFrame::IdLessThan> localKeyFramePoses;
    std::map<MapPointPtr, Eigen::Vector2d, MapPoint::IdLessThan,
            Eigen::aligned_allocator<std::pair<MapPointPtr, Eigen::Vector2d>>> localMapPoints;

    const std::vector<KeyFramePtr> vpLocalKFs = pMap->GetLocalKeyFrames();

    LocalParameterization* angle_local_parameter = AngleLocalParameterization::Create();
    LossFunction* huber_loss = new HuberLoss(1.0);
    Problem problem;

    for(const KeyFramePtr& kf : vpLocalKFs)
    {
        localKeyFramePoses[kf] = kf->GetPoseTbw();
        SE2& Tbw = localKeyFramePoses[kf];
        const std::map<MapPointPtr, int> observations = kf->GetObservations();

        for(const auto& it : observations)
        {
            if(localMapPoints.find(it.first) == localMapPoints.end())
            {
                const cv::Point2d pos = it.first->GetPos();
                localMapPoints[it.first] = Eigen::Vector2d(pos.x, pos.y);
            }

            const cv::Point2f pt = kf->GetKeyPointXY(it.second);
            CostFunction* cost_function = ReprojectionErrorFunctor::Create(pt);
            problem.AddResidualBlock(cost_function, huber_loss, &Tbw.x, &Tbw.y, &Tbw.theta, localMapPoints[it.first].data());
        }

        problem.SetParameterization(&Tbw.theta, angle_local_parameter);
    }

    // manhattan
    Eigen::Vector2d major_normal_w(major_line.le.x, major_line.le.y);
    for(auto& kf : localKeyFramePoses)
    {
        Line major_line_c;
        kf.first->GetMajorLine(major_line_c);
        CostFunction* cost_manhattan = ManhattanErrorFunctor::Create(cv::Point2f(major_line_c.le.x, major_line_c.le.y), 1100.0);
        problem.AddResidualBlock(cost_manhattan,nullptr,&kf.second.theta,major_normal_w.data());
    }
    problem.SetParameterBlockConstant(major_normal_w.data());

    Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    Solver::Summary summary;
    Solve(options,&problem,&summary);

    if(!summary.IsSolutionUsable())
    {
        return;
    }

    // update keyframe poses and map points
    for(auto& kf : localKeyFramePoses)
    {
        kf.first->SetPoseTbw(kf.second);
    }
    for(auto& mp : localMapPoints)
    {
        mp.first->SetPos(cv::Point2f(mp.second[0],mp.second[1]));
    }

//    // update major line
//    major_line.le.x = major_normal_w[0];
//    major_line.le.y = major_normal_w[1];
}

void Optimizer::OptimizeMajorLine(const std::vector<KeyLine> &vKeyLines, const std::vector<bool> &vIsParallel,
                                  cv::Point3f &le)
{
    assert(vKeyLines.size() == vIsParallel.size());
    int N = vKeyLines.size();

    Problem problem;

    Eigen::Vector2d normal(le.x, le.y);
    for(int i = 0; i < N; i++)
    {
        cv::Point3f le_i = KeyLineGeometry::GetKeyLineCoeff(vKeyLines[i]);
        if(vIsParallel[i])
        {
            CostFunction* cost_function = ParallelLineCostFunctor::Create(le_i.x, le_i.y);
            problem.AddResidualBlock(cost_function, nullptr,normal.data());
        }
        else
        {
            CostFunction* cost_function = OrthogonalLineCostFunctor::Create(le_i.x, le_i.y);
            problem.AddResidualBlock(cost_function, nullptr,normal.data());
        }
    }

    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    Solver::Summary summary;
    Solve(options,&problem,&summary);

    if(!summary.IsSolutionUsable())
    {
        return;
    }

    le.x = normal[0];
    le.y = normal[1];
}


}  // namespace birdview