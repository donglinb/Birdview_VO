//
// Created by mrwhite on 2021/3/25.
//

#ifndef BIRDVIEW_VO_MAPPOINT_H
#define BIRDVIEW_VO_MAPPOINT_H

#include <memory>
#include <mutex>
#include <map>

#include <opencv2/core/core.hpp>

namespace birdview
{

class KeyFrame;
typedef std::shared_ptr<KeyFrame> KeyFramePtr;

class MapPoint
{

public:

    explicit MapPoint(const cv::Point2f& pos);

    int Id() const { return mnId; }

    cv::Point2f GetPos();
    void SetPos(const cv::Point2d& pos);

    void AddObservation(const KeyFramePtr& pKF, int idx);
    void EraseObservation(const KeyFramePtr& pKF);
    const std::map<KeyFramePtr, int>& GetObservations();

    const cv::Mat& GetDescriptor() const;
    void UpdateDescriptor();

    struct IdLessThan
    {
        bool operator() (const std::shared_ptr<MapPoint>& lhs, const std::shared_ptr<MapPoint>& rhs) const
        {
            return lhs->Id() < rhs->Id();
        }
    };

protected:

    int mnId;
    static int mnNextId;

    cv::Point2f mPos;
    cv::Mat mDescriptor;

    std::map<KeyFramePtr, int> mObservations;

    std::mutex mMutexPos;
    std::mutex mMutexObs;
};
typedef std::shared_ptr<MapPoint> MapPointPtr;

}  // namespace birdview

#endif //BIRDVIEW_VO_MAPPOINT_H
