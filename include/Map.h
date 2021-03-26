//
// Created by mrwhite on 2021/3/25.
//

#ifndef BIRDVIEW_VO_MAP_H
#define BIRDVIEW_VO_MAP_H

#include <memory>
#include <mutex>
#include <set>

#include "KeyFrame.h"
#include "MapPoint.h"

namespace birdview
{

class Map
{

public:

    Map();

    void AddKeyFrame(const KeyFramePtr& pKF);
    void AddMapPoint(const MapPointPtr& pMP);

    std::vector<KeyFramePtr> GetKeyFramesAll();
    std::vector<MapPointPtr> GetMapPointAll();

    int GetNumKeyFrames();
    int GetNumMapPoints();

    // local map
    void SetCurrentKF(KeyFramePtr pKF);
    KeyFramePtr GetCurrentKF();
    void UpdateLocalMap();
    bool NeedLocalBA();
    void LocalBAFinished();

    const std::vector<KeyFramePtr>& GetLocalKeyFrames();
    const std::vector<MapPointPtr>& GetLocalMapPoints();

protected:

    // global map
    std::set<KeyFramePtr, KeyFrame::IdLessThan> mspMapKeyFrames;
    std::set<MapPointPtr, MapPoint::IdLessThan> mspMapPoints;

    // local map
    KeyFramePtr mpCurrentKF;
    std::vector<KeyFramePtr> mvpLocalKeyFrames;
    std::vector<MapPointPtr> mvpLocalMapPoints;
    bool mbNeedLocalBA;

    std::mutex mMutexKF;
    std::mutex mMutexMP;
    std::mutex mMutexCurrentKF;
    std::mutex mMutexLocalMap;
};
typedef std::shared_ptr<Map> MapPtr;

}  // namespace birdview

#endif //BIRDVIEW_VO_MAP_H
