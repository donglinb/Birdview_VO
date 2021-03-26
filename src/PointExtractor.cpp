//
// Created by mrwhite on 2021/3/24.
//

#include "PointExtractor.h"

namespace birdview
{

PointExtractor::PointExtractor()
{
    mpExtractor = cv::ORB::create();
}

void PointExtractor::ExtractKeyPoints(const cv::Mat& ImageRaw, std::vector<cv::KeyPoint>& vKeyPoints,
                                      cv::Mat& Descriptors, const cv::Mat& mask)
{
    cv::Ptr<cv::GFTTDetector> gftt = cv::GFTTDetector::create(500,0.01,8);
    gftt->detect(ImageRaw,vKeyPoints,mask);
    mpExtractor->compute(ImageRaw,vKeyPoints,Descriptors);
//    mpExtractor->detectAndCompute(ImageRaw, mask, vKeyPoints, Descriptors);
}

}  // namespace birdview
