//
// Created by mrwhite on 2021/3/25.
//

#include "MapPoint.h"
#include "KeyFrame.h"
#include "PointMatcher.h"

namespace birdview
{

int MapPoint::mnNextId = 0;

MapPoint::MapPoint(const cv::Point2f &pos)
  : mPos(pos)
{
    mnId = mnNextId++;
}

cv::Point2f MapPoint::GetPos()
{
    std::unique_lock<std::mutex> lock(mMutexPos);
    return mPos;
}

void MapPoint::SetPos(const cv::Point2d& pos)
{
    std::unique_lock<std::mutex> lock(mMutexPos);
    mPos = pos;
}

void MapPoint::AddObservation(const KeyFramePtr& pKF, int idx)
{
    std::unique_lock<std::mutex> lock(mMutexObs);
    mObservations[pKF] = idx;
}

void MapPoint::EraseObservation(const KeyFramePtr& pKF)
{
    std::unique_lock<std::mutex> lock(mMutexObs);
    mObservations.erase(pKF);
}

const std::map<KeyFramePtr, int> &MapPoint::GetObservations()
{
    std::unique_lock<std::mutex> lock(mMutexObs);
    return mObservations;
}

const cv::Mat& MapPoint::GetDescriptor() const
{
    return mDescriptor;
}

void MapPoint::UpdateDescriptor()
{
    std::vector<cv::Mat> vDescriptors;

    {
        std::unique_lock<std::mutex> lock(mMutexObs);
        for(auto& it : mObservations)
        {
            vDescriptors.push_back(it.first->GetDescriptor(it.second));
        }
    }

    int N = vDescriptors.size();
    std::vector<std::vector<int>> distances(N, std::vector<int>(N, -1));
    for(int i = 0; i < N; i++)
    {
        distances[i][i] = 0;
        for(int j = i + 1; j < N; j++)
        {
            int dist = PointMatcher::DescriptorDistance(vDescriptors[i], vDescriptors[j]);
            distances[i][j] = dist;
            distances[j][i] = dist;
        }
    }

    // find the one with minimal median distance with others
    int bestMedian = INT_MAX;
    int bestIdx = 0;
    for(int i = 0; i < N; i++)
    {
        std::sort(distances[i].begin(),distances[i].end());
        int median = distances[i][(N - 1) / 2];
        if(median < bestMedian)
        {
            bestMedian = median;
            bestIdx = i;
        }
    }

    mDescriptor = vDescriptors[bestIdx].clone();
}

}  // namespace birdview