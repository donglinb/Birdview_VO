//
// Created by mrwhite on 2021/3/25.
//

#include "Map.h"

#include <utility>

namespace birdview
{

Map::Map()
  : mpCurrentKF(nullptr), mbNeedLocalBA(false), mbIsMajorLineSet(false)
{
    ;
}

void Map::AddKeyFrame(const KeyFramePtr& pKF)
{
    std::unique_lock<std::mutex> lock(mMutexKF);
    mspMapKeyFrames.insert(pKF);
}

void Map::AddMapPoint(const MapPointPtr& pMP)
{
    std::unique_lock<std::mutex> lock(mMutexMP);
    mspMapPoints.insert(pMP);
}

void Map::SetCurrentKF(KeyFramePtr pKF)
{
    std::unique_lock<std::mutex> lock(mMutexCurrentKF);
    mpCurrentKF = std::move(pKF);

    UpdateLocalMap();
}

KeyFramePtr Map::GetCurrentKF()
{
    std::unique_lock<std::mutex> lock(mMutexCurrentKF);
    return mpCurrentKF;
}

std::vector<KeyFramePtr> Map::GetKeyFramesAll()
{
    std::unique_lock<std::mutex> lock(mMutexKF);
    return std::vector<KeyFramePtr>(mspMapKeyFrames.begin(), mspMapKeyFrames.end());
}

std::vector<MapPointPtr> Map::GetMapPointAll()
{
    std::unique_lock<std::mutex> lock(mMutexMP);
    return std::vector<MapPointPtr>(mspMapPoints.begin(), mspMapPoints.end());
}

int Map::GetNumKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexKF);
    return mspMapKeyFrames.size();
}

int Map::GetNumMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexMP);
    return mspMapPoints.size();
}

void Map::UpdateLocalMap()
{
    std::unique_lock<std::mutex> lock(mMutexLocalMap);

    mvpLocalKeyFrames.clear();
    mvpLocalMapPoints.clear();

    std::set<KeyFramePtr, KeyFrame::IdLessThan> setKeyFrames;
    std::set<MapPointPtr, MapPoint::IdLessThan> setMapPoints;

    setKeyFrames.insert(mpCurrentKF);

    int n = 2;

    while(n >= 0)
    {
        for(auto& kf : setKeyFrames)
        {
            const std::map<MapPointPtr, int>& observations = kf->GetObservations();
            for(auto& pt : observations)
            {
                setMapPoints.insert(pt.first);
            }
        }

        if(n ==0)
            break;

        for(auto& pt : setMapPoints)
        {
            const std::map<KeyFramePtr, int> covisibleKfs = pt->GetObservations();
            for(auto& kf_it : covisibleKfs)
            {
                if(kf_it.first == mpCurrentKF)
                    continue;
                setKeyFrames.insert(kf_it.first);
            }
        }

        n--;
    }

    mvpLocalKeyFrames.insert(mvpLocalKeyFrames.begin(),setKeyFrames.begin(),setKeyFrames.end());
    mvpLocalMapPoints.insert(mvpLocalMapPoints.begin(),setMapPoints.begin(),setMapPoints.end());

    mbNeedLocalBA = true;
}

const std::vector<KeyFramePtr> &Map::GetLocalKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexLocalMap);
    return mvpLocalKeyFrames;
}

const std::vector<MapPointPtr> &Map::GetLocalMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexLocalMap);
    return mvpLocalMapPoints;
}

bool Map::NeedLocalBA()
{
    std::unique_lock<std::mutex> lock(mMutexLocalMap);
    return mbNeedLocalBA;
}
void Map::LocalBAFinished()
{
    std::unique_lock<std::mutex> lock(mMutexLocalMap);
    mbNeedLocalBA = false;
}

void Map::SetMajorLine(const Line &major_line)
{
    std::unique_lock<std::mutex> lock(mMutexMajorLine);
    mMajorLine = major_line;
    mbIsMajorLineSet = true;
}

bool Map::GetMajorLine(Line& major_line)
{
    std::unique_lock<std::mutex> lock(mMutexMajorLine);
    if(!mbIsMajorLineSet)
        return false;
    major_line = mMajorLine;
    return true;
}

bool Map::IsMajorLineSet()
{
    std::unique_lock<std::mutex> lock(mMutexMajorLine);
    return mbIsMajorLineSet;
}

}  // namespace birdview