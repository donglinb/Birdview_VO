#pragma once
#ifndef BIRDVIEW_VO_VIEWER_H
#define BIRDVIEW_VO_VIEWER_H

#include <string>
#include <mutex>
#include <memory>

#include <eigen3/Eigen/Core>
#include <pangolin/pangolin.h>

#include "KeyFrame.h"
#include "MapPoint.h"

namespace birdview
{

class MapViewer
{

public:

    struct MapViewerConfig
    {
        MapViewerConfig()
        : ViewPointX(0.0), ViewPointY(-0.7), ViewPointZ(-1.8), ViewPointF(500),
        PointSize(2), LineWidth(1), KeyFrameSize(0.05), KeyFrameLineWidth(1), 
        GraphLineWidth(0.9), CameraSize(0.08), CameraLineWidth(3), 
        WindowWidth(1024), WindowHeight(768)
        {
            ;
        }
        int WindowWidth, WindowHeight;
        double ViewPointX, ViewPointY, ViewPointZ, ViewPointF;
        double PointSize, LineWidth;
        double KeyFrameSize, KeyFrameLineWidth, GraphLineWidth;
        double CameraSize, CameraLineWidth;
    };

    MapViewer(const std::string& windowName, const MapViewerConfig& cfg);
    void Run();
    void UpdateMapPoints(const std::vector<MapPointPtr>& vpMapPoints);
    void UpdateKeyFrames(const std::vector<KeyFramePtr>& vpKeyFrames);
    void RequestQuit();

protected:

    void CreateContext();
    void DrawMapPoints();
    // void DrawMapLines();
    void DrawKeyFrames();
    void DrawAxis(double lineWidth = 1.0, double lineLength = 1.0);

private:

    std::string mWindowName;
    MapViewerConfig cfg;

    std::vector<MapPointPtr> mvpMapPoints;
    std::vector<KeyFramePtr> mvpKeyFrames;

    std::mutex mMutexPoint, mMutexLine;
    std::mutex mMutexKeyFrame;

    pangolin::OpenGlRenderState* s_cam;
    pangolin::View* d_cam;
};
typedef std::shared_ptr<MapViewer> MapViewerPtr;

}  // namespace birdview

#endif  // BIRDVIEW_VO_VIEWER_H