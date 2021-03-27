#pragma once
#ifndef BIRDVIEW_VO_LINE_EXTRACTOR_H_
#define BIRDVIEW_VO_LINE_EXTRACTOR_H_

#include <memory>
#include <vector>

#include <opencv2/core.hpp>
#include <line_descriptor_custom.hpp>

namespace birdview
{

using KeyLine = slam::line_descriptor::KeyLine;
using LSDDetector = slam::line_descriptor::LSDDetectorC;
using BinaryDescriptor = slam::line_descriptor::BinaryDescriptor;
using BinaryDescriptorMatcher = slam::line_descriptor::BinaryDescriptorMatcher;
using DrawLinesMatchesFlags = slam::line_descriptor::DrawLinesMatchesFlags;

class LineExtractor
{
public:
    LineExtractor();
    ~LineExtractor();
    void extractLines(const cv::Mat& img, std::vector<KeyLine>& keylines, 
                      cv::Mat& descriptors, const cv::Mat& mask = cv::Mat());
    void extractLines(const cv::Mat& img, std::vector<KeyLine>& keylines,
                      const cv::Mat& mask = cv::Mat());
    static void drawKeylines(const cv::Mat& image, const std::vector<KeyLine>& keylines, cv::Mat& outImage, 
                             const cv::Scalar& color = cv::Scalar::all( -1 ), int flags = DrawLinesMatchesFlags::DEFAULT );
    static int match(const cv::Mat& queryDescriptors, const cv::Mat& trainDescriptors, std::vector<cv::DMatch>& vMatches,
                     float fNNratio = 0.8, const cv::Mat& mask = cv::Mat(), bool mutualBest = true);
    static int matchNNR(const cv::Mat& queryDescriptors, const cv::Mat& trainDescriptors, std::vector<cv::DMatch>& vMatches,
                        float fNNratio = 0.8, const cv::Mat& mask = cv::Mat());
    static void drawLineMatches( const cv::Mat& img1, const std::vector<KeyLine>& keylines1, const cv::Mat& img2, const std::vector<KeyLine>& keylines2,
                                 const std::vector<cv::DMatch>& matches1to2, cv::Mat& outImg, const cv::Scalar& matchColor = cv::Scalar::all( -1 ),
                                 const cv::Scalar& singleLineColor = cv::Scalar::all( -1 ), const std::vector<char>& matchesMask = std::vector<char>(),
                                 int flags = DrawLinesMatchesFlags::DEFAULT );
private:
    LSDDetector::LSDOptions opts;
    cv::Ptr<LSDDetector> lsd;
    cv::Ptr<BinaryDescriptor> lbd;
};
typedef std::shared_ptr<LineExtractor> LineExtractorPtr;

}  // namespace birdview

#endif  // BIRDVIEW_VO_LINE_EXTRACTOR_H_
