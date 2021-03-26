//
// Created by mrwhite on 2021/3/25.
//

#ifndef BIRDVIEW_VO_KEYFRAME_H
#define BIRDVIEW_VO_KEYFRAME_H

#include <memory>
#include <mutex>
#include <map>

#include "Frame.h"

namespace birdview
{

class MapPoint;
typedef std::shared_ptr<MapPoint> MapPointPtr;

class KeyFrame : public Frame
{

public:

    explicit KeyFrame(const Frame& frame);

    int Id() const { return mnId; }

    void AddObservation(const MapPointPtr& pMP, int idx);
    void EraseObservation(const MapPointPtr& pMP);
    void EraseObservation(int idx);
    bool hasObservation(int idx);
    bool hasObservation(const MapPointPtr& pMP);
    MapPointPtr GetObservation(int idx);
    const std::map<MapPointPtr, int>& GetObservations();

    SE2 GetPoseTbw() override;
    void SetPoseTbw(const SE2& Tbw) override;

    struct IdLessThan
    {
        bool operator() (const std::shared_ptr<KeyFrame>& lhs, const std::shared_ptr<KeyFrame>& rhs) const
        {
            return lhs->Id() < rhs->Id();
        }
    };

protected:

    int mnId;
    static int mnNextId;

    std::map<MapPointPtr, int> mObservations;
    std::map<int, MapPointPtr> mDualObservations;

    std::mutex mMutexObs;
    std::mutex mMutexPose;

};
typedef std::shared_ptr<KeyFrame> KeyFramePtr;

}  // namespace birdview

#endif //BIRDVIEW_VO_KEYFRAME_H
