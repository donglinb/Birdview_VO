//
// Created by mrwhite on 2021/3/26.
//

#include "LocalMapping.h"

#include <utility>
#include <unistd.h>

#include "Optimizer.h"
#include "Config.h"

namespace birdview
{

LocalMapper::LocalMapper(MapPtr pMap)
  : mpMap(std::move(pMap)), mbFinishRequested(false), mbFinished(false)
{
    ;
}

void LocalMapper::Run()
{
    mbFinished = false;
    mbFinishRequested = false;

    while(!mbFinishRequested)
    {
        if(mpMap->NeedLocalBA())
        {
            // local BA
            if(Config::UseLines() && mpMap->IsMajorLineSet())
            {
                Line major_line;
                mpMap->GetMajorLine(major_line);
                Optimizer::LocalBundleAdjustment(mpMap, major_line);
                mpMap->SetMajorLine(major_line);
            }
            else
            {
                Optimizer::LocalBundleAdjustment(mpMap);
            }
            mpMap->LocalBAFinished();
        }
        else
        {
            usleep(1000);
        }
    }

    mbFinished = true;
}

void LocalMapper::RequestFinish()
{
    mbFinishRequested = true;
}

bool LocalMapper::IsFinished() const
{
    return mbFinished;
}


}  // namespace birdview