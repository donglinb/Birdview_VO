//
// Created by mrwhite on 2021/3/25.
//

#include "KeyFrame.h"

namespace birdview
{

int KeyFrame::mnNextId = 0;

KeyFrame::KeyFrame(const Frame &frame)
  : Frame(frame)
{
    mnId = mnNextId++;
}

void KeyFrame::AddObservation(const MapPointPtr& pMP, int idx)
{
    std::unique_lock<std::mutex> lock(mMutexObs);
    mObservations[pMP] = idx;
    mDualObservations[idx] = pMP;
}

void KeyFrame::EraseObservation(const MapPointPtr& pMP)
{
    std::unique_lock<std::mutex> lock(mMutexObs);
    if(mObservations.find(pMP) == mObservations.end())
        return;
    int idx = mObservations[pMP];
    mObservations.erase(pMP);
    mDualObservations.erase(idx);
}

void KeyFrame::EraseObservation(int idx)
{
    std::unique_lock<std::mutex> lock(mMutexObs);
    if(mDualObservations.find(idx) == mDualObservations.end())
        return;
    MapPointPtr pMP = mDualObservations[idx];
    mObservations.erase(pMP);
    mDualObservations.erase(idx);
}

bool KeyFrame::hasObservation(int idx)
{
    std::unique_lock<std::mutex> lock(mMutexObs);
    return (mDualObservations.find(idx) != mDualObservations.end());
}

bool KeyFrame::hasObservation(const MapPointPtr& pMP)
{
    std::unique_lock<std::mutex> lock(mMutexObs);
    return (mObservations.find(pMP) != mObservations.end());
}

MapPointPtr KeyFrame::GetObservation(int idx)
{
    std::unique_lock<std::mutex> lock(mMutexObs);
    if(mDualObservations.find(idx) == mDualObservations.end())
        return nullptr;
    return mDualObservations[idx];
}

const std::map<MapPointPtr, int> &KeyFrame::GetObservations()
{
    std::unique_lock<std::mutex> lock(mMutexObs);
    return mObservations;
}

SE2 KeyFrame::GetPoseTbw()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return mTbw;
}

void KeyFrame::SetPoseTbw(const SE2& Tbw)
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    mTbw = Tbw;
}

}  // namespace birdview