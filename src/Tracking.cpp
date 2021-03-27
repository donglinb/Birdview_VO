//
// Created by mrwhite on 2021/3/25.
//

#include "Tracking.h"

#include <iostream>
#include <utility>
#include <thread>
#include <opencv2/highgui/highgui.hpp>

#include "Config.h"
#include "Optimizer.h"
#include "KeyLineGeometry.h"
#include "Timer.h"

namespace birdview
{

std::ostream& operator<<(std::ostream& o, const SE2& pose)
{
    o << "x = " << pose.x << ", y = " << pose.y << ", theta = " << pose.theta;
}

Tracker::Tracker(MapPtr pMap)
  : mpMap(std::move(pMap)), mpMapViewer(nullptr), mpFrame(nullptr), mpKF(nullptr),
  mpLineExtractor(nullptr), mpLocalMapper(nullptr)
{
    mpPointExtractor = std::make_shared<PointExtractor>();
    mpPointMatcher = std::make_shared<PointMatcher>();
    mpIcpSolver = std::make_shared<IcpSolver>(Config::MaxIterationsRansac());

    if(Config::UseLines())
    {
        mpLineExtractor = std::make_shared<LineExtractor>();
    }
}

void Tracker::SetMapViewer(MapViewerPtr pMapViewer)
{
    mpMapViewer = std::move(pMapViewer);
}

void Tracker::SetLocalMapper(LocalMapperPtr pLocalMapper)
{
    mpLocalMapper = std::move(pLocalMapper);
}

SE2 Tracker::Track(const cv::Mat& imageRaw, const cv::Mat& mask)
{
    mpFrame = std::make_shared<Frame>(imageRaw, mpPointExtractor, mask);

    if(!mpKF)
    {
        if(mpFrame->GetNumPoints() >= Config::KeyFrameFeature())
        {
            CalculateMajorLine(mpFrame);
            CreateKeyFrame();

            mpMap->AddKeyFrame(mpKF);
            mpMap->SetCurrentKF(mpKF);

            // convert to world frame
            Line major_line;
            mpKF->GetMajorLine(major_line);
            SE2 Twb = mpKF->GetPoseTbw().inv();
            mpMap->SetMajorLine(Twb * major_line);

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

    std::thread tLines(&Tracker::CalculateMajorLine, this, ref(mpFrame));

    mvMatches12.clear();
    int nMatched = mpPointMatcher->Match(mpKF,mpFrame,mvMatches12,mvPrevMatched);
    ComputeCurrentPose();

    {
        cv::Mat matchImg;
        cv::drawMatches(mpKF->GetImageRaw(),mpKF->GetKeyPoints(),mpFrame->GetImageRaw(),mpFrame->GetKeyPoints(),mvMatches12,matchImg);
        cv::imshow("Matches", matchImg);
    }

    tLines.join();

    if(NeedNewKeyFrame(nMatched))
    {
        CreateKeyFrame();
        AssociateMapPoints();

        if(Config::UseLines())
        {
            Line major_line;
            mpMap->GetMajorLine(major_line);
            Optimizer::PoseOptimization(mpKF, major_line);
        }
        else
        {
            Optimizer::PoseOptimization(mpKF);
        }

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
    mpIcpSolver->FindRtICP2D(mpKF->GeyKeyPointsXY(),mpFrame->GeyKeyPointsXY(),
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

bool Tracker::CalculateMajorLine(const FramePtr& pFrame)
{
    static const float th = cos(M_PI / 4);
    static const float th2 = cos(M_PI / 10);
    static const float thDist = 0.05;
    static const float cx = 0.5 * pFrame->GetImageRaw().cols;
    static const float cy = 0.5 * pFrame->GetImageRaw().rows;

    if(!Config::UseLines())
        return false;

//    static cv::VideoWriter cap("frame.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, pFrame->GetImageRaw().size());

    const cv::Mat& imageRaw = pFrame->GetImageRaw();
    const cv::Mat& mask = pFrame->GetMaskRaw();
    cv::Point2f image_center(cx, cy);

    // extract lsd lines
    std::vector<KeyLine> vKeyLines;
    mpLineExtractor->extractLines(imageRaw, vKeyLines, mask);

    // find major direction from lsd lines
    std::vector<bool> status;
    KeyLine major_line;
    KeyLineGeometry::FindMajorDirection(vKeyLines, status, major_line);
    KeyLineGeometry::reduceVector(vKeyLines, status);

    // optimize direction
    cv::Point3f line = KeyLineGeometry::GetKeyLineCoeff(major_line);
    status = std::vector<bool>(vKeyLines.size(), false);
    cv::Point3f infinity(line.y, - line.x, 0.0);
    for(int i = 0; i < vKeyLines.size(); i++)
    {
        float dist = KeyLineGeometry::GetKeyLineCoeff(vKeyLines[i]).dot(infinity);
        if(fabs(dist) < thDist)
        {
            status[i] = true;
        }
    }
    Optimizer::OptimizeMajorLine(vKeyLines,status,line);

    // correct direction
    cv::Point2f dir(line.y, - line.x);
    if(!mpKF)
    {
        const cv::Point2f minus_y(0.0, -1.0);
        float cos_theta = fabs(dir.dot(minus_y)) / (cv::norm(dir) * cv::norm(minus_y));
        if(cos_theta < th)
        {
            dir = cv::Point2f(dir.y, - dir.x);
        }
        if(dir.y > 0)
        {
            dir = - dir;
        }

        mLastDir = dir / cv::norm(dir);
    }
    else
    {
        float cos_theta = fabs(dir.dot(mLastDir)) / (cv::norm(dir) * cv::norm(mLastDir));
        if(cos_theta < th2)
        {
            dir = cv::Point2f(dir.y, - dir.x);
        }
        if(dir.dot(mLastDir) < 0)
        {
            dir = - dir;
        }

        mLastDir = dir / cv::norm(dir);
    }
    dir = dir / cv::norm(dir);
    Line l(image_center, image_center + cy * dir);

    // convert to XY coordinate
    Line MajorLine = Line(Frame::BirdviewPT2XY(l.sP), Frame::BirdviewPT2XY(l.eP));
    pFrame->SetMajorLine(MajorLine);

    // visualization
    cv::Mat keylineImg;
    mpLineExtractor->drawKeylines(imageRaw, vKeyLines, keylineImg);

    cv::Point3f line2 = cv::Point3f(line.y, -line.x, 1.0);
    KeyLineGeometry::DrawLineDirection(keylineImg, line);
    KeyLineGeometry::DrawLineDirection(keylineImg, line2);
    cv::arrowedLine(keylineImg,l.sP,l.eP,cv::Scalar(0,0,255),4);

    if(mpKF)
    {
        Line global_line;
        mpMap->GetMajorLine(global_line);
        SE2 Tbw = mpKF->GetPoseTbw();
        Line local_line = Tbw * global_line;
        Line lc(Frame::ProjectXY2Birdview(local_line.sP), Frame::ProjectXY2Birdview(local_line.eP));
        cv::Point2f dir_c(lc.le.y, - lc.le.x);
        dir_c = dir_c / cv::norm(dir_c);
        lc = Line(image_center, image_center + cy * dir_c);

        cv::arrowedLine(keylineImg,lc.sP,lc.eP,cv::Scalar(0,255,0),4);
    }

    cv::imshow("KeyLines", keylineImg);
//    cap << keylineImg;

    return true;
}

}  // namespace birdview