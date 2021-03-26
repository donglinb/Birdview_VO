//
// Created by mrwhite on 2021/3/24.
//

#ifndef BIRDVIEW_VO_POINTEXTRACTOR_H
#define BIRDVIEW_VO_POINTEXTRACTOR_H

#include <memory>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace birdview
{

class PointExtractor
{
public:
    PointExtractor();
    void ExtractKeyPoints(const cv::Mat& ImageRaw, std::vector<cv::KeyPoint>& vKeyPoints,
                          cv::Mat& Descriptors, const cv::Mat& mask = cv::Mat());
private:
    cv::Ptr<cv::Feature2D> mpExtractor;
};
typedef std::shared_ptr<PointExtractor> PointExtractorPtr;

}  // namespace birdview

#endif //BIRDVIEW_VO_POINTEXTRACTOR_H
