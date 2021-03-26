//
// Created by mrwhite on 2021/3/24.
//

#ifndef BIRDVIEW_VO_POINTMATCHER_H
#define BIRDVIEW_VO_POINTMATCHER_H

#include <memory>
#include <vector>

#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"

namespace birdview
{

class PointMatcher
{

public:

    explicit PointMatcher(float NNratio = 0.6, bool CheckOrientation = true);

    // Computes the Hamming distance between two ORB descriptors
    static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

    //match orb features for birdview, output Matches12, from 1 to 2. Previously matched points are tracked.
    int Match(const FramePtr& pF1, const FramePtr& pF2, std::vector<cv::DMatch> &vMatches12, std::vector<cv::Point2f> &vPrevMatched, int windowSize=10) const;

    //match orb features for birdview, output Matches12, from 1 to 2.
    int Match(const FramePtr& pF1, const FramePtr& pF2, std::vector<cv::DMatch> &vMatches12, int windowSize = 10) const;

    // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
    // Used to track the local map (Tracking)
    int SearchByProjection(const KeyFramePtr& pF, const std::vector<MapPointPtr> &vpMapPoints, std::vector<cv::DMatch>& vLocalMatches, const float r=4) const;

protected:

    static const int TH_LOW;
    static const int TH_HIGH;
    static const int HISTO_LENGTH;

    static void ComputeThreeMaxima(std::vector<int>* histo, int L, int &ind1, int &ind2, int &ind3);

    float mfNNratio;
    bool mbCheckOrientation;
};
typedef std::shared_ptr<PointMatcher> PointMatcherPtr;

}  // namespace birdview

#endif //BIRDVIEW_VO_POINTMATCHER_H
