//
// Created by mrwhite on 2021/3/24.
//

#ifndef BIRDVIEW_VO_FRAME_H
#define BIRDVIEW_VO_FRAME_H

#include <memory>
#include <vector>
#include <opencv2/core/core.hpp>

#include "PointExtractor.h"
#include "LineExtractor.h"
#include "SE2.h"

namespace birdview
{

class Frame
{
public:
    Frame(const cv::Mat& imageRaw, PointExtractorPtr pPointExtractor, const cv::Mat& mask = cv::Mat());
    Frame(const cv::Mat& imageRaw, PointExtractorPtr pPointExtractor, LineExtractorPtr pLineExtractor, const cv::Mat& mask = cv::Mat());

    int FrameId() const { return mnFrameId; }
    int GetNumPoints() const { return mvKeyPoints.size(); }
    const cv::KeyPoint& GetKeyPoint(int idx) const { assert(idx < mvKeyPoints.size()); return mvKeyPoints[idx]; }
    const std::vector<cv::KeyPoint>& GetKeyPoints() const { return mvKeyPoints; }
    const cv::Point2f& GetKeyPointXY(int idx) const { assert(idx < mvKeyPointsXY.size()); return mvKeyPointsXY[idx]; }
    const std::vector<cv::Point2f>& GeyKeyPointsXY() const { return mvKeyPointsXY; }
    cv::Mat GetDescriptor(int idx) const { assert(idx < mDescriptors.rows); return mDescriptors.row(idx); }
    const cv::Mat& GetDescriptors() const { return mDescriptors; }
    const cv::Mat& GetImageRaw() const { return mImageRaw; }
    const cv::Mat& GetMaskRaw() const { return mMaskRaw; }

    // manhattan lines
    bool CalculateLineMainDirs();
    bool GetMainDirs(cv::Point3f& dir1, cv::Point3f& dir2) const;
    void SetMainDirs(const cv::Point3f& dir1, const cv::Point3f& dir2);

    static bool PosInGrid(const cv::Point2f& pt, int& posX, int& posY) ;
    void AssignKeyPointsInGrid();
    std::vector<size_t> GetFeaturesInArea(float x, float y, float r, int minLevel = 0, int maxLevel = -1) const;

    virtual SE2 GetPoseTbw() { return mTbw; }
    virtual void SetPoseTbw(const SE2& Tbw) { mTbw = Tbw; }

    static cv::Point2f BirdviewKP2XY(const cv::KeyPoint &kp);
    static cv::Point2f ProjectXY2Birdview(const cv::Point2f &p);
    static bool PosInImage(const cv::Point2f& pt);

    template<typename T>
    static bool ProjectXY2Birdview(const T X, const T Y, T& u, T& v)
    {
        if(!mbParamsSet)
        {
            return false;
        }
        u = T(birdviewCols) / T(2.0) - Y * T(meter2pixel);
        v = T(birdviewRows) / (2.0) - (X - T(rear_axle_to_center)) * (meter2pixel);
        return true;
    }

protected:

    int mnFrameId;
    static int mnNextFrameId;

    std::vector<cv::KeyPoint> mvKeyPoints;
    std::vector<cv::Point2f> mvKeyPointsXY;
    cv::Mat mDescriptors;
    PointExtractorPtr mpPointExtractor;

    // manhattan lines
    LineExtractorPtr mpLineExtractor;
    cv::Point3f mMainDir1, mMainDir2;
    bool mbIsMainDirSet;

    cv::Mat mImageRaw;
    cv::Mat mMaskRaw;

    typedef std::vector<std::size_t> GridElement;
    std::vector<std::vector<GridElement>> mGrid;

    SE2 mTbw;

    // vehicle parameters
    static const double pixel2meter;
    static const double meter2pixel;
    static const double rear_axle_to_center;
    static const double vehicle_length;
    static const double vehicle_width;

    // size and grid
    static bool mbParamsSet;
    static const int FRAME_GRID_COLS;
    static const int FRAME_GRID_ROWS;
    static int birdviewRows, birdviewCols;
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
};
typedef std::shared_ptr<Frame> FramePtr;

}  // namespace birdview


#endif //BIRDVIEW_VO_FRAME_H
