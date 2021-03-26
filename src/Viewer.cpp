#include <iostream>

#include "Viewer.h"

namespace birdview
{

using namespace pangolin;

MapViewer::MapViewer(const std::string& windowName, const MapViewerConfig& cfg)
  : mWindowName(windowName),cfg(cfg), s_cam(nullptr), d_cam(nullptr)
{
   ;
}

void MapViewer::CreateContext()
{
    pangolin::CreateWindowAndBind(mWindowName,cfg.WindowWidth,cfg.WindowHeight);
    
    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // Define Camera Render Object (for view / scene browsing)
    s_cam = new pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(cfg.WindowWidth,cfg.WindowHeight,cfg.ViewPointF,cfg.ViewPointF,cfg.WindowWidth/2,cfg.WindowHeight/2,0.1,1000),
        pangolin::ModelViewLookAt(cfg.ViewPointX,cfg.ViewPointY,cfg.ViewPointZ, 0,0,0,0.0,-1.0, 0.0)
        );

    // Add named OpenGL viewport to window and provide 3D Handler
    d_cam = &pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, - double(cfg.WindowWidth)/double(cfg.WindowHeight))
            .SetHandler(new pangolin::Handler3D(*s_cam));
}

void MapViewer::Run()
{
    std::cout << "Viewer Running" << std::endl;
    // BindToContext(mWindowName);
    CreateContext();

    glClearColor(1.0f,1.0f,1.0f,1.0f);

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    while(!ShouldQuit())
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam->Activate(*s_cam);

        DrawMapPoints();
        DrawKeyFrames();

        pangolin::FinishFrame();
    }

    DestroyWindow(mWindowName);
}

void MapViewer::RequestQuit()
{
    // Quit();
    QuitAll();
}

void MapViewer::UpdateMapPoints(const std::vector<MapPointPtr>& vpMapPoints)
{
    std::unique_lock<std::mutex> lock(mMutexPoint);
    mvpMapPoints = vpMapPoints;
}

void MapViewer::UpdateKeyFrames(const std::vector<KeyFramePtr>& vpKeyFrames)
{
    std::unique_lock<std::mutex> lock(mMutexKeyFrame);
    mvpKeyFrames = vpKeyFrames;
}

void MapViewer::DrawMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexPoint);

    // std::cout << "Drawing " << mvpMapPoints.size() << " map points. " << std::endl;

    glPointSize(cfg.PointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(MapPointPtr pMP : mvpMapPoints)
    {
        const cv::Point2f& p = pMP->GetPos();
        glVertex3f(p.x,p.y,0);
    }
    glEnd();
}

// void MapViewer::DrawLines(std::vector<Eigen::Vector6d>& lines)
// {
//     glLineWidth(cfg.LineWidth);
//     glBegin(GL_LINES);
//     glColor3f(0.0,0.0,0.0);

//     for(size_t i=0, iend=lines.size(); i<iend;i++)
//     {
//         const Eigen::Vector3d& sp = lines[i].head(3);
//         const Eigen::Vector3d& ep = lines[i].tail(3);
//         glVertex3f(sp[0],sp[1],sp[2]);
//         glVertex3f(ep[0],ep[1],ep[2]);
//     }
//     glEnd();
// }

void MapViewer::DrawKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexKeyFrame);

    // std::cout << "Drawing " << mvpKeyFrames.size() << " keyframes. " << std::endl;

    const float &w = cfg.KeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    for(KeyFramePtr pKF : mvpKeyFrames)
    {
        const SE3::SE3Quat Tcw = pKF->GetPoseTbw().toSE3Quat();
        Eigen::Matrix4d Twc = Tcw.inverse().to_homogeneous_matrix();

        glPushMatrix();
        glMultMatrixd(Twc.data());



        glLineWidth(cfg.KeyFrameLineWidth);
        glColor3f(0.0f,0.0f,1.0f);
        glBegin(GL_LINES);

        glVertex3f(0,0,0); glVertex3f(w,h,z);
        glVertex3f(0,0,0); glVertex3f(w,-h,z);
        glVertex3f(0,0,0); glVertex3f(-w,-h,z);
        glVertex3f(0,0,0); glVertex3f(-w,h,z);

        glVertex3f(w,h,z); glVertex3f(w,-h,z);
        glVertex3f(-w,h,z); glVertex3f(-w,-h,z);
        glVertex3f(-w,h,z); glVertex3f(w,h,z);
        glVertex3f(-w,-h,z); glVertex3f(w,-h,z);

        glEnd();

        glPopMatrix();
    }

    // Draw graph
    glLineWidth(cfg.GraphLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    for(size_t i = 1; i < mvpKeyFrames.size(); i++)
    {
        Eigen::Vector3d begin = mvpKeyFrames[i - 1]->GetPoseTbw().toSE3Quat().inverse().translation();
        Eigen::Vector3d end = mvpKeyFrames[i]->GetPoseTbw().toSE3Quat().inverse().translation();
        glVertex3f(begin[0], begin[1], begin[2]);
        glVertex3f(end[0], end[1], end[2]);
    }
    glEnd();
}

void MapViewer::DrawAxis(double lineWidth, double lineLength)
{
    glLineWidth(lineWidth);
    // x axis red
    glColor3f(1.0,0.0,0.0);
    glBegin(GL_LINES);
    glVertex3f(0.0,0.0,0.0);
    glVertex3f(lineLength,0.0,0.0);
    glEnd();
    // y axis green
    glColor3f(0.0,1.0,0.0);
    glBegin(GL_LINES);
    glVertex3f(0.0,0.0,0.0);
    glVertex3f(0.0,lineLength,0.0);
    glEnd();
    // z axis blue
    glColor3f(0.0,0.0,1.0);
    glBegin(GL_LINES);
    glVertex3f(0.0,0.0,0.0);
    glVertex3f(0.0,0.0,lineLength);
    glEnd();
}


}  // namespace birdview