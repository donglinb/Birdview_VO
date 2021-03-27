//
// Created by mrwhite on 2021/3/26.
//

#ifndef BIRDVIEW_VO_KEYLINEGEOMETRY_H
#define BIRDVIEW_VO_KEYLINEGEOMETRY_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <line_descriptor_custom.hpp>

namespace birdview
{

class KeyLineGeometry
{
public:
    using KeyLine =  slam::line_descriptor::KeyLine;

    static cv::Point3f GetKeyLineCoeff(const KeyLine& kl);
    static cv::Point3f GetKeyLineIntersect(const KeyLine& kl1, const KeyLine& kl2);
    static int FindMajorDirection(const std::vector<KeyLine>& vKeyLines, std::vector<bool>& status,
                                  cv::Point3f& dir1_out, cv::Point3f& dir2_out, float thDist = 0.05);
    static bool GenerateRansacIndices(int TotalNum, int SelectNum, int MaxIterations, std::vector<std::vector<size_t>>& vSets);

    template<class T>
    static void reduceVector(std::vector<T> &v, const std::vector<bool>& status)
    {
        int N = v.size();
        int j = 0;
        for (int i = 0; i < N; i++)
            if (status[i])
                v[j++] = v[i];
        v.resize(j);
    }
};

}  // namespace birdview

#endif //BIRDVIEW_VO_KEYLINEGEOMETRY_H
