//
// Created by mrwhite on 2021/3/27.
//

#ifndef BIRDVIEW_VO_FEATURELINE_H
#define BIRDVIEW_VO_FEATURELINE_H

#include <cmath>
#include <opencv2/core/core.hpp>

#include "SE2.h"

namespace birdview
{

struct Line
{
    cv::Point2f sP;
    cv::Point2f eP;
    cv::Point3f le;
    Line()
    {
        ;
    }
    Line(const cv::Point2f& sp, const cv::Point2f& ep)
      : sP(sp), eP(ep)
    {
        cv::Point3f s(sp.x, sp.y, 1.0), e(ep.x, ep.y, 1.0);
        le = s.cross(e);
        le = le / std::sqrt(le.x * le.x + le.y * le.y);
    }
};

Line operator*(const SE2& pose, const Line& line);

}  // namespace birdview

#endif //BIRDVIEW_VO_FEATURELINE_H
