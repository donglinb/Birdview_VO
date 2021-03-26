#ifndef ICPSOLVER_H
#define ICPSOLVER_H

#include <iostream>
#include <vector>
#include <memory>

#include <opencv2/opencv.hpp>

namespace birdview
{

using namespace std;

class IcpSolver
{
public:

    typedef pair<int,int> Match;

    explicit IcpSolver(int MaxIterations=100)
    {
        mMaxIterations = MaxIterations;
    }

    int FindRtICP(const vector<cv::Point3f> &vKeysXYZ1, const vector<cv::Point3f> &vKeysXYZ2, const vector<Match> &vMatches,
                    vector<bool> &vbMatchesInliers, cv::Mat &R, cv::Mat &t, float sigma = 1.0) const;
    int FindRtICP2D(const vector<cv::Point2f> &vKeysXY1, const vector<cv::Point2f> &vKeysXY2, const vector<Match> &vMatches,
                    vector<bool> &vbMatchesInliers, cv::Mat &R, cv::Mat &t, float sigma = 1.0) const;

private:

    static bool ComputeRtICP(const vector<cv::Point3f> &vP1, const vector<cv::Point3f> &vP2, cv::Mat &R, cv::Mat &t);
    static bool ComputeRtICP2D(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2, cv::Mat &R, cv::Mat &t);

    static int CheckRtICP(const cv::Mat &R, const cv::Mat &t, const vector<cv::Point3f> &vP3D1, const vector<cv::Point3f> &vP3D2,
                const vector<Match> &vMatches12, vector<bool> &vbMatchesInliers, float sigma);
    static int CheckRtICP2D(const cv::Mat &R, const cv::Mat &t, const vector<cv::Point2f> &vP2D1, const vector<cv::Point2f> &vP2D2,
                const vector<Match> &vMatches12, vector<bool> &vbMatchesInliers, float sigma);

    
    int mMaxIterations;
};
typedef std::shared_ptr<IcpSolver> IcpSolverPtr;


}  // namespace ORB_SLAM2

#endif  //ICPSOLVER_H