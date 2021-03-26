//
// Created by mrwhite on 2021/3/25.
//

#include "Config.h"

namespace birdview
{

int Config::mnMaxFrames = 10;
int Config::mnKeyFrameFeatures = 100;
float Config::mfMinTrackedRatio = 0.4;
int Config::mnMinTrackedPoints = 30;

int Config::mnMaxIterationsRansac = 100;

}  // namespace birdview