//
// Created by mrwhite on 2021/3/26.
//

#ifndef BIRDVIEW_VO_LOCALMAPPING_H
#define BIRDVIEW_VO_LOCALMAPPING_H

#include <memory>

#include "Map.h"

namespace birdview
{

class LocalMapper
{

public:

    explicit LocalMapper(MapPtr pMap);
    void Run();

    void RequestFinish();
    bool IsFinished() const;

protected:

    MapPtr mpMap;

    bool mbFinishRequested;
    bool mbFinished;
};
typedef std::shared_ptr<LocalMapper> LocalMapperPtr;


}  // namespace birdview

#endif //BIRDVIEW_VO_LOCALMAPPING_H
