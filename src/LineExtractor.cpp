#include "LineExtractor.h"

namespace birdview
{

LineExtractor::LineExtractor()
{
    opts.refine       = 0;
    opts.scale        = 1.2;
    opts.sigma_scale  = 0.6;
    opts.quant        = 2.0;
    opts.ang_th       = 22.5;
    opts.log_eps      = 1.0;
    opts.density_th   = 0.6;
    opts.n_bins       = 1024;

    lsd = LSDDetector::createLSDDetectorC();
    lbd = BinaryDescriptor::createBinaryDescriptor();
}

LineExtractor::~LineExtractor()
{
    ;
}

void LineExtractor::extractLines(const cv::Mat& img, std::vector<KeyLine>& keylines, cv::Mat& descriptors, const cv::Mat& mask)
{
    opts.min_length   = 0.025 * std::max(img.rows, img.cols);

    std::vector<KeyLine> keylines_;
    lsd->detect(img,keylines_,1,1,opts);

    keylines.clear();
    for(auto & keyline : keylines_)
    {
        if( keyline.octave == 0 && keyline.lineLength >= 20)
        {
            keylines.push_back(keyline);
        }
    }
    
    lbd->compute(img, keylines, descriptors);
}

void LineExtractor::extractLines(const cv::Mat &img, std::vector<KeyLine> &keylines, const cv::Mat &mask)
{
    opts.min_length   = 0.025 * std::max(img.rows, img.cols);

    std::vector<KeyLine> keylines_;
    lsd->detect(img,keylines_,1,1,opts);

    keylines.clear();
    for(auto & keyline : keylines_)
    {
        if( keyline.octave == 0 && keyline.lineLength >= 20)
        {
            keylines.push_back(keyline);
        }
    }
}


void LineExtractor::drawKeylines(const cv::Mat& image, const std::vector<KeyLine>& keylines, 
                                 cv::Mat& outImage, const cv::Scalar& color, int flags)
{
    slam::line_descriptor::drawKeylines(image, keylines,outImage, color, flags);
}

int LineExtractor::matchNNR(const cv::Mat& queryDescriptors, const cv::Mat& trainDescriptors,
                    std::vector<cv::DMatch>& vMatches, const float fNNratio, const cv::Mat& mask)
{
    std::vector<std::vector<cv::DMatch>> matches;
    BinaryDescriptorMatcher matcher;
    matcher.knnMatch(queryDescriptors, trainDescriptors, matches, 2, mask);

    vMatches.clear();

    // ratio test
    CV_Assert(queryDescriptors.rows == matches.size());
    for(int i = 0; i < queryDescriptors.rows; i++)
    {
        if(matches[i].empty())
        {
            continue;
        }
        else if(matches[i].size() == 1)
        {
            vMatches.push_back(matches[i][0]);
        }
        else
        {
            double dist1 = matches[i][0].distance;
            double dist2 = matches[i][1].distance;
            if(dist1 < fNNratio * dist2)
            {
                vMatches.push_back(matches[i][0]);
            }
        }
    }
    
    return vMatches.size();
}

int LineExtractor::match(const cv::Mat& queryDescriptors, const cv::Mat& trainDescriptors, std::vector<cv::DMatch>& vMatches,
                         const float fNNratio, const cv::Mat& mask, bool mutualBest)
{
    if(!mutualBest)
    {
        return matchNNR(queryDescriptors,trainDescriptors,vMatches,fNNratio,mask);
    }

    vMatches.clear();

    // mutual best
    std::vector<cv::DMatch> vMatches12, vMatches21;
    matchNNR(queryDescriptors,trainDescriptors,vMatches12,fNNratio,mask);
    matchNNR(trainDescriptors,queryDescriptors,vMatches21,fNNratio,mask);

    std::vector<int> vnMatches21(trainDescriptors.rows,-1);
    for(const cv::DMatch& m : vMatches21)
    {
        vnMatches21[m.queryIdx] = m.trainIdx;
    }

    for(const cv::DMatch& m : vMatches12)
    {
        if(m.trainIdx < vnMatches21.size() && vnMatches21[m.trainIdx] == m.queryIdx)
        {
            vMatches.push_back(m);
        }
    }

    return vMatches.size();
}

void LineExtractor::drawLineMatches(const cv::Mat& img1, const std::vector<KeyLine>& keylines1, const cv::Mat& img2, 
                                    const std::vector<KeyLine>& keylines2, const std::vector<cv::DMatch>& matches1to2, 
                                    cv::Mat& outImg, const cv::Scalar& matchColor, const cv::Scalar& singleLineColor, 
                                    const std::vector<char>& matchesMask, int flags)
{
    slam::line_descriptor::drawLineMatches(img1, keylines1, img2, keylines2, matches1to2, outImg,  matchColor, singleLineColor, matchesMask, flags);
}

}  // namespace birdview
