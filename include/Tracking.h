//
// Created by mrwhite on 2021/3/25.
//

#ifndef BIRDVIEW_VO_TRACKING_H
#define BIRDVIEW_VO_TRACKING_H

#include <memory>
#include <vector>

#include <opencv2/core/core.hpp>

#include "PointExtractor.h"
#include "PointMatcher.h"
#include "LineExtractor.h"
#include "IcpSolver.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "Viewer.h"
#include "Map.h"
#include "SE2.h"

namespace birdview
{

class Tracker
{

public:

    explicit Tracker(MapPtr pMap);
    SE2 Track(const cv::Mat& imageRaw, const cv::Mat& mask);
    void ComputeCurrentPose();
    bool NeedNewKeyFrame(int nMatched);
    void CreateKeyFrame();
    void AssociateMapPoints();

    // manhattan lines
    bool CalculateLineMainDirs(const FramePtr& pFrame);

    void SetMapViewer(MapViewerPtr pMapViewer);

protected:

    MapPtr mpMap;
    MapViewerPtr mpMapViewer;

    PointExtractorPtr mpPointExtractor;
    PointMatcherPtr mpPointMatcher;
    IcpSolverPtr mpIcpSolver;
    LineExtractorPtr mpLineExtractor;

    FramePtr mpFrame;
    KeyFramePtr mpKF;

    std::vector<cv::DMatch> mvMatches12;
    std::vector<cv::Point2f> mvPrevMatched;

    template<class T>
    void reduceVector(std::vector<T> &v, const std::vector<bool>& status)
    {
        int N = v.size();
        int j = 0;
        for (int i = 0; i < N; i++)
            if (status[i])
                v[j++] = v[i];
        v.resize(j);
    }
};
typedef std::shared_ptr<Tracker> TrackerPtr;

}  // namespace birdview

#endif //BIRDVIEW_VO_TRACKING_H
