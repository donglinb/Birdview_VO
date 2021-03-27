//
// Created by mrwhite on 2021/3/24.
//

#include "Frame.h"

#include <utility>
#include <thread>
#include <opencv2/highgui/highgui.hpp>
#include "KeyLineGeometry.h"
#include "Timer.h"

namespace birdview
{

using namespace std;

const double Frame::pixel2meter = 0.03984;
const double Frame::meter2pixel = 25.1;
const double Frame::rear_axle_to_center = 1.393;
const double Frame::vehicle_length = 4.63;
const double Frame::vehicle_width = 1.901;

int Frame::mnNextFrameId = 0;

bool Frame::mbParamsSet = false;
const int Frame::FRAME_GRID_COLS = 64, Frame::FRAME_GRID_ROWS = 48;
int Frame::birdviewRows = 0, Frame::birdviewCols = 0;
float Frame::mfGridElementWidthInv = 0.0, Frame::mfGridElementHeightInv = 0.0;

Frame::Frame(const cv::Mat& imageRaw, PointExtractorPtr pPointExtractor, const cv::Mat& mask)
        : mpPointExtractor(std::move(pPointExtractor)) , mImageRaw(imageRaw.clone()), mMaskRaw(mask.clone()),
        mbIsMajorLineSet(false), mpLineExtractor(nullptr)
{
    mnFrameId = mnNextFrameId++;

    if(!mbParamsSet)
    {
        birdviewCols = imageRaw.cols;
        birdviewRows = imageRaw.rows;
        mfGridElementWidthInv = double(FRAME_GRID_COLS) / birdviewCols;
        mfGridElementHeightInv = double(FRAME_GRID_ROWS) / birdviewRows;

        mbParamsSet = true;
    }

    mpPointExtractor->ExtractKeyPoints(imageRaw, mvKeyPoints, mDescriptors, mask);
    for(const cv::KeyPoint& kp : mvKeyPoints)
    {
        mvKeyPointsXY.push_back(BirdviewKP2XY(kp));
    }
    assert(mvKeyPoints.size() == mvKeyPointsXY.size());

    AssignKeyPointsInGrid();
}

Frame::Frame(const cv::Mat &imageRaw, PointExtractorPtr pPointExtractor, LineExtractorPtr pLineExtractor, const cv::Mat &mask)
  : mpPointExtractor(std::move(pPointExtractor)) , mImageRaw(imageRaw.clone()), mMaskRaw(mask.clone()),
    mbIsMajorLineSet(false), mpLineExtractor(std::move(pLineExtractor))
{
    mnFrameId = mnNextFrameId++;

    if(!mbParamsSet)
    {
        birdviewCols = imageRaw.cols;
        birdviewRows = imageRaw.rows;
        mfGridElementWidthInv = double(FRAME_GRID_COLS) / birdviewCols;
        mfGridElementHeightInv = double(FRAME_GRID_ROWS) / birdviewRows;

        mbParamsSet = true;
    }

    std::thread tLineExtraction(&Frame::CalculateMajorLine, this);

    mpPointExtractor->ExtractKeyPoints(imageRaw, mvKeyPoints, mDescriptors, mask);
    for(const cv::KeyPoint& kp : mvKeyPoints)
    {
        mvKeyPointsXY.push_back(BirdviewKP2XY(kp));
    }
    assert(mvKeyPoints.size() == mvKeyPointsXY.size());

    AssignKeyPointsInGrid();

    tLineExtraction.join();
}

bool Frame::PosInGrid(const cv::Point2f& pt, int& posX, int& posY)
{
    posX = round(pt.x * mfGridElementWidthInv);
    posY = round(pt.y * mfGridElementHeightInv);

    if(posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 || posY >= FRAME_GRID_ROWS)
        return false;

    return true;
}

void Frame::AssignKeyPointsInGrid()
{
    mGrid.resize(FRAME_GRID_COLS, std::vector<GridElement>(FRAME_GRID_ROWS));
    int posX = 0, posY = 0;
    for(size_t i = 0; i < mvKeyPoints.size(); i++)
    {
        const cv::KeyPoint& kp = mvKeyPoints[i];
        if(PosInGrid(kp.pt, posX, posY))
        {
            mGrid[posX][posY].push_back(i);
        }
    }
}

vector<size_t> Frame::GetFeaturesInArea(const float x, const float y, const float r, const int minLevel, const int maxLevel) const
{
    int N = mvKeyPoints.size();

    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(unsigned long j : vCell)
            {
                const cv::KeyPoint &kp = mvKeyPoints[j];
                if(bCheckLevels)
                {
                    if(kp.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kp.octave>maxLevel)
                            continue;
                }

                const float distx = kp.pt.x-x;
                const float disty = kp.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(j);
            }
        }
    }

    return vIndices;
}

cv::Point2f Frame::BirdviewKP2XY(const cv::KeyPoint &kp)
{
    return BirdviewPT2XY(kp.pt);
}

cv::Point2f Frame::BirdviewPT2XY(const cv::Point2f &kp)
{
    if(!mbParamsSet)
    {
        throw std::runtime_error("Params not set yet.");
    }

    cv::Point2f p;
    p.x = (birdviewRows/2.0-kp.y)*pixel2meter+rear_axle_to_center;
    p.y = (birdviewCols/2.0-kp.x)*pixel2meter;
    // p.z = -0.32115;

    return p;
}

cv::Point2f Frame::ProjectXY2Birdview(const cv::Point2f &p)
{
    if(!mbParamsSet)
    {
        throw std::runtime_error("Params not set yet.");
    }

    cv::Point2f pt;
    pt.x = birdviewCols/2.0-p.y*meter2pixel;
    pt.y = birdviewRows/2.0-(p.x-rear_axle_to_center)*meter2pixel;
    return pt;
}

bool Frame::PosInImage(const cv::Point2f &pt)
{
    if(pt.x < 0 || pt.x >= birdviewCols || pt.y < 0 || pt.y >= birdviewRows)
        return false;
    return true;
}

bool Frame::GetMajorLine(Line& line) const
{
    if(!mbIsMajorLineSet)
    {
        return false;
    }
    line = mMajorLine;
    return true;
}

void Frame::SetMajorLine(const Line& line)
{
    mMajorLine = line;
    mbIsMajorLineSet = true;
}

bool Frame::CalculateMajorLine()
{
    if(!mpLineExtractor)
        return false;

    std::vector<KeyLine> vKeyLines;
    mpLineExtractor->extractLines(mImageRaw, vKeyLines, mMaskRaw);
    std::vector<bool> status;
    KeyLine major_line;
    KeyLineGeometry::FindMajorDirection(vKeyLines, status, major_line);
    mMajorLine = Line(BirdviewPT2XY(major_line.getStartPoint()), BirdviewPT2XY(major_line.getEndPoint()));

    mbIsMajorLineSet = true;

    return true;
}


}  // namespace birdview
