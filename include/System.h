//
// Created by mrwhite on 2021/3/26.
//

#ifndef BIRDVIEW_VO_SYSTEM_H
#define BIRDVIEW_VO_SYSTEM_H

#include <thread>
#include <memory>

#include <opencv2/core/core.hpp>

#include "Tracking.h"
#include "LocalMapping.h"
#include "Map.h"
#include "Viewer.h"
#include "SE2.h"

namespace birdview
{

class System
{

public:

    System();
    void Start();
    void RequestFinish();

    SE2 TrackImage(const cv::Mat& imageRaw, const cv::Mat& mask);

protected:

    void WaitAllFinish();

    MapPtr mpMap;
    MapViewerPtr mpMapViewer;
    TrackerPtr mpTracker;
    LocalMapperPtr mpLocalMapper;

    std::shared_ptr<std::thread> mptMapViewer;
    std::shared_ptr<std::thread> mptLocalMapper;

};
typedef std::shared_ptr<System> SystemPtr;

}  // namespace birdview

#endif //BIRDVIEW_VO_SYSTEM_H
