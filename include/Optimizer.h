//
// Created by mrwhite on 2021/3/25.
//

#ifndef BIRDVIEW_VO_OPTIMIZER_H
#define BIRDVIEW_VO_OPTIMIZER_H

#include <vector>

#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Map.h"
#include "FeatureLine.h"

namespace birdview
{

class Optimizer
{

public:

    static void PoseOptimization(const KeyFramePtr& pKF);
    static void PoseOptimization(const KeyFramePtr& pKF, const Line& major_line);
    static void PoseOptimization(const FramePtr& pFrame, const std::vector<MapPointPtr>& vpLocalMapPoints,
                                 const std::vector<cv::DMatch>& vMatches);
    static void LocalBundleAdjustment(const MapPtr& pMap);
    static void LocalBundleAdjustment(const MapPtr& pMap, Line& major_line);

    static void OptimizeMajorLine(const std::vector<KeyLine>& vKeyLines, const std::vector<bool>& vIsParallel, cv::Point3f& le);
};

}  // namespace birdview

#endif //BIRDVIEW_VO_OPTIMIZER_H
