//
// Created by mrwhite on 2021/3/25.
//

#include "Tracking.h"

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <utility>

#include "Config.h"
#include "Optimizer.h"

namespace birdview
{

std::ostream& operator<<(std::ostream& o, const SE2& pose)
{
    o << "x = " << pose.x << ", y = " << pose.y << ", theta = " << pose.theta;
}

Tracker::Tracker(MapPtr pMap)
  : mpMap(std::move(pMap)), mpMapViewer(nullptr), mpFrame(nullptr), mpKF(nullptr)
{
    mpPointExtractor = std::make_shared<PointExtractor>();
    mpPointMatcher = std::make_shared<PointMatcher>();
    mpIcpSolver = std::make_shared<IcpSolver>(Config::MaxIterationsRansac());
}

void Tracker::SetMapViewer(MapViewerPtr pMapViewer)
{
    mpMapViewer = std::move(pMapViewer);
}

SE2 Tracker::Track(const cv::Mat& imageRaw, const cv::Mat& mask)
{
    mpFrame = std::make_shared<Frame>(imageRaw, mpPointExtractor, mask);

    if(!mpKF)
    {
        if(mpFrame->GetNumPoints() >= Config::KeyFrameFeature())
        {
            CreateKeyFrame();

            mpMap->AddKeyFrame(mpKF);
            mpMap->SetCurrentKF(mpKF);

            // create initial map point
            int N = mpKF->GetNumPoints();
            for(int i = 0; i < N; i++)
            {
                MapPointPtr pMP = std::make_shared<MapPoint>(mpKF->GetPoseTbw().inv() * mpKF->GetKeyPointXY(i));

                pMP->AddObservation(mpKF,i);
                mpKF->AddObservation(pMP,i);

                mpMap->AddMapPoint(pMP);
            }
        }

        return mpFrame->GetPoseTbw();
    }

    mvMatches12.clear();
    int nMatched = mpPointMatcher->Match(mpKF,mpFrame,mvMatches12,mvPrevMatched);
    ComputeCurrentPose();

    {
        cv::Mat matchImg;
        cv::drawMatches(mpKF->GetImageRaw(),mpKF->GetKeyPoints(),mpFrame->GetImageRaw(),mpFrame->GetKeyPoints(),mvMatches12,matchImg);
        cv::imshow("Matches", matchImg);
    }

    if(NeedNewKeyFrame(nMatched))
    {
        CreateKeyFrame();
        AssociateMapPoints();
        Optimizer::PoseOptimization(mpKF);

        mpMap->AddKeyFrame(mpKF);
        mpMap->SetCurrentKF(mpKF);
    }
    else
    {
        // TODO: PoseOptimization for frame
        // const std::vector<MapPointPtr>& vpLocalMapPoints = mpMap->GetLocalMapPoints();
    }

    {
        mpMapViewer->UpdateKeyFrames(mpMap->GetKeyFramesAll());
        mpMapViewer->UpdateMapPoints(mpMap->GetMapPointAll());
    }

    return mpFrame->GetPoseTbw();
}

void Tracker::ComputeCurrentPose()
{
    std::vector<IcpSolver::Match> vMatches;
    for(const cv::DMatch& m : mvMatches12)
    {
        vMatches.emplace_back(m.queryIdx,m.trainIdx);
    }
    std::vector<bool> vbMatchesInliers(vMatches.size(),false);

    cv::Mat R,t;
    int score = mpIcpSolver->FindRtICP2D(mpKF->GeyKeyPointsXY(),mpFrame->GeyKeyPointsXY(),
                             vMatches,vbMatchesInliers,R,t,0.03984);

    SE2 Tbr = SE2::fromCvSE2Rt(R,t).inv();
    mpFrame->SetPoseTbw(Tbr + mpKF->GetPoseTbw());

    // remove outliers
    reduceVector(mvMatches12, vbMatchesInliers);
}

bool Tracker::NeedNewKeyFrame(int nMatched)
{
    bool c1 = mpFrame->FrameId() - mpKF->FrameId() > Config::MaxFrames();
    bool c2 = mpFrame->GetNumPoints() > Config::KeyFrameFeature();
    bool c3 = nMatched < Config::MinTrackedRatio() * mpKF->GetNumPoints();
    bool c4 = nMatched < Config::MinTrackedPoints();

    bool NeedNew = c1 || c2 && (c3 || c4);

    return NeedNew;
}

void Tracker::CreateKeyFrame()
{
    std::cout << "Create New KeyFrame with Frame #" << mpFrame->FrameId() << std::endl;

    mpKF = std::make_shared<KeyFrame>(*mpFrame);

    mvPrevMatched.clear();
    cv::KeyPoint::convert(mpKF->GetKeyPoints(), mvPrevMatched);
}

void Tracker::AssociateMapPoints()
{
    int track_old = 0, local_map = 0, add_new = 0;

    // associate with last KF
    KeyFramePtr pLastKF = mpMap->GetCurrentKF();
    for(const cv::DMatch& m : mvMatches12)
    {
        if(pLastKF->hasObservation(m.queryIdx))
        {
            MapPointPtr pMP = pLastKF->GetObservation(m.queryIdx);

            pMP->AddObservation(mpKF, m.trainIdx);
            mpKF->AddObservation(pMP, m.trainIdx);

            track_old++;
        }
    }

    // associate with local map
    std::vector<cv::DMatch> vLocalMatches;
    const std::vector<MapPointPtr>& vpLocalMapPoints = mpMap->GetLocalMapPoints();
    mpPointMatcher->SearchByProjection(mpKF, vpLocalMapPoints, vLocalMatches);
    for(const cv::DMatch& m : vLocalMatches)
    {
        MapPointPtr pMP = vpLocalMapPoints[m.trainIdx];

        pMP->AddObservation(mpKF, m.queryIdx);
        mpKF->AddObservation(pMP, m.queryIdx);

        local_map++;
    }

    // add new map points
    for(const cv::DMatch& m : mvMatches12)
    {
        if(!mpKF->hasObservation(m.trainIdx))
        {
            MapPointPtr pMP = std::make_shared<MapPoint>(mpKF->GetPoseTbw().inv() * mpKF->GetKeyPointXY(m.trainIdx));

            pMP->AddObservation(pLastKF, m.queryIdx);
            pMP->AddObservation(mpKF, m.trainIdx);

            pLastKF->AddObservation(pMP, m.queryIdx);
            mpKF->AddObservation(pMP, m.trainIdx);

            mpMap->AddMapPoint(pMP);

            add_new++;
        }
    }

    std::cout << "KeyFrame #" << mpKF->Id() << ":" << std::endl;
    std::cout << "Tracked old: " << track_old << ", Local track: " << local_map
                << ", Add new: " << add_new << std::endl;
}

}  // namespace birdview