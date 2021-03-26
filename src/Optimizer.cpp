//
// Created by mrwhite on 2021/3/25.
//

#include "Optimizer.h"

#include <map>
#include <eigen3/Eigen/Core>
#include <ceres/ceres.h>

#include "CostFunctors.h"
#include "SE2.h"

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

}  // namespace birdview