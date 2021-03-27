//
// Created by mrwhite on 2021/3/27.
//

#include "FeatureLine.h"

namespace birdview
{

Line operator*(const SE2& pose, const Line& line)
{
    cv::Point2f sP = pose * line.sP;
    cv::Point2f eP = pose * line.eP;
    return {sP, eP};
}


}  // namespace birdview