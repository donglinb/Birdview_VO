//
// Created by mrwhite on 2021/3/26.
//

#include "System.h"

namespace birdview
{

System::System()
  : mptMapViewer(nullptr), mptLocalMapper(nullptr)
{
    mpMap = std::make_shared<Map>();

    MapViewer::MapViewerConfig cfg;
    mpMapViewer = std::make_shared<MapViewer>("MapViewer", cfg);

    mpTracker = std::make_shared<Tracker>(mpMap);
    mpTracker->SetMapViewer(mpMapViewer);

    mpLocalMapper = std::make_shared<LocalMapper>(mpMap);
}

void System::Start()
{
    mptMapViewer = std::make_shared<std::thread>(&MapViewer::Run, mpMapViewer);
    mptLocalMapper = std::make_shared<std::thread>(&LocalMapper::Run, mpLocalMapper);
}

void System::RequestFinish()
{
    mpLocalMapper->RequestFinish();
    mpMapViewer->RequestQuit();

    WaitAllFinish();
}

void System::WaitAllFinish()
{
    mptLocalMapper->join();
    mptMapViewer->join();
}

SE2 System::TrackImage(const cv::Mat &imageRaw, const cv::Mat &mask)
{
    return mpTracker->Track(imageRaw, mask);
}


}  // namespace birdview