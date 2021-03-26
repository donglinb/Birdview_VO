//
// Created by mrwhite on 2021/3/25.
//

#ifndef BIRDVIEW_VO_CONFIG_H
#define BIRDVIEW_VO_CONFIG_H

namespace birdview
{

class Config
{

public:

    static int MaxFrames() { return mnMaxFrames; }
    static int KeyFrameFeature() { return mnKeyFrameFeatures; }
    static float MinTrackedRatio() { return mfMinTrackedRatio; }
    static int MinTrackedPoints() { return mnMinTrackedPoints; }
    static int MaxIterationsRansac() { return mnMaxIterationsRansac; }

protected:

    static int mnMaxFrames;
    static int mnKeyFrameFeatures;
    static float mfMinTrackedRatio;
    static int mnMinTrackedPoints;

    static int mnMaxIterationsRansac;

};

}  // namespace birdview

#endif //BIRDVIEW_VO_CONFIG_H
