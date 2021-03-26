//
// Created by mrwhite on 2021/3/24.
//

#include "PointMatcher.h"

namespace birdview
{

const int PointMatcher::TH_HIGH = 100;
const int PointMatcher::TH_LOW = 50;
const int PointMatcher::HISTO_LENGTH = 30;

using namespace std;

PointMatcher::PointMatcher(float NNratio, bool CheckOrientation)
  : mfNNratio(NNratio), mbCheckOrientation(CheckOrientation)
{
    ;
}

// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int PointMatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

void PointMatcher::ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
{
    int max1=0;
    int max2=0;
    int max3=0;

    for(int i=0; i<L; i++)
    {
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    if(max2<0.1f*(float)max1)
    {
        ind2=-1;
        ind3=-1;
    }
    else if(max3<0.1f*(float)max1)
    {
        ind3=-1;
    }
}

// F1: Reference frame, F2: Current frame;
int PointMatcher::Match(const FramePtr& pF1, const FramePtr& pF2, std::vector<cv::DMatch> &vMatches12, std::vector<cv::Point2f> &vPrevMatched, int windowSize) const
{
    int nmatches=0;
    int N1 = pF1->GetNumPoints();
    int N2 = pF2->GetNumPoints();

    std::vector<int> vnMatches12 = vector<int>(N1,-1);

    vector<int> rotHist[HISTO_LENGTH];
    for(auto & i : rotHist)
        i.reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    vector<int> vMatchedDistance(N2,INT_MAX);
    vector<int> vnMatches21(N2,-1);

    for(size_t i1 = 0; i1 < N1; i1++)
    {
        const cv::KeyPoint& kp1 = pF1->GetKeyPoint(i1);
        int level1 = kp1.octave;
        if(level1>0)
            continue;

        vector<size_t> vIndices2 = pF2->GetFeaturesInArea(vPrevMatched[i1].x,vPrevMatched[i1].y, windowSize,level1,level1);

        if(vIndices2.empty())
            continue;

        cv::Mat d1 = pF1->GetDescriptor(i1);

        int bestDist = INT_MAX;
        int bestDist2 = INT_MAX;
        int bestIdx2 = -1;

        for(size_t i2 : vIndices2)
        {
            cv::Mat d2 = pF2->GetDescriptor(i2);

            int dist = DescriptorDistance(d1,d2);

            if(vMatchedDistance[i2]<=dist)
                continue;

            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestIdx2=i2;
            }
            else if(dist<bestDist2)
            {
                bestDist2=dist;
            }
        }

        if(bestDist<=TH_LOW)
        {
            if(bestDist<(float)bestDist2*mfNNratio)
            {
                if(vnMatches21[bestIdx2]>=0)
                {
                    vnMatches12[vnMatches21[bestIdx2]]=-1;
                    nmatches--;
                }
                vnMatches12[i1]=bestIdx2;
                vnMatches21[bestIdx2]=i1;
                vMatchedDistance[bestIdx2]=bestDist;
                nmatches++;

                if(mbCheckOrientation)
                {
                    float rot = kp1.angle - pF2->GetKeyPoint(bestIdx2).angle;
                    if(rot<0.0)
                        rot+=360.0f;
                    int bin = round(rot*factor);
                    if(bin==HISTO_LENGTH)
                        bin=0;
                    assert(bin>=0 && bin<HISTO_LENGTH);
                    rotHist[bin].push_back(i1);
                }
            }
        }

    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(int idx1 : rotHist[i])
            {
                if(vnMatches12[idx1]>=0)
                {
                    vnMatches21[vnMatches12[idx1]]=-1;
                    vnMatches12[idx1]=-1;
                    nmatches--;
                }
            }
        }
    }

    // convert to DMatch
    vMatches12.clear();
    for(int i = 0; i < vnMatches12.size(); i++)
    {
        if(vnMatches12[i] < 0)
            continue;
        int distance = DescriptorDistance(pF1->GetDescriptor(i), pF2->GetDescriptor(vnMatches12[i]));
        vMatches12.emplace_back(i, vnMatches12[i], distance);
    }


    //Update prev matched
    for(size_t i1=0, iend1=vnMatches12.size(); i1<iend1; i1++)
        if(vnMatches12[i1]>=0)
            vPrevMatched[i1] = pF2->GetKeyPoint(vnMatches12[i1]).pt;

    return nmatches;
}

int PointMatcher::Match(const FramePtr& pF1, const FramePtr& pF2, std::vector<cv::DMatch> &vMatches12, int windowSize) const
{
    int nmatches=0;
    int N1 = pF1->GetNumPoints();
    int N2 = pF2->GetNumPoints();

    std::vector<int> vnMatches12 = vector<int>(N1,-1);

    vector<int> rotHist[HISTO_LENGTH];
    for(auto & i : rotHist)
        i.reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    vector<int> vMatchedDistance(N2,INT_MAX);
    vector<int> vnMatches21(N2,-1);

    for(size_t i1 = 0; i1 < N1; i1++)
    {
        const cv::KeyPoint& kp1 = pF1->GetKeyPoint(i1);
        int level1 = kp1.octave;
        if(level1>0)
            continue;

        vector<size_t> vIndices2 = pF2->GetFeaturesInArea(kp1.pt.x,kp1.pt.y, windowSize,level1,level1);

        if(vIndices2.empty())
            continue;

        cv::Mat d1 = pF1->GetDescriptor(i1);

        int bestDist = INT_MAX;
        int bestDist2 = INT_MAX;
        int bestIdx2 = -1;

        for(size_t i2 : vIndices2)
        {
            cv::Mat d2 = pF2->GetDescriptor(i2);

            int dist = DescriptorDistance(d1,d2);

            if(vMatchedDistance[i2]<=dist)
                continue;

            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestIdx2=i2;
            }
            else if(dist<bestDist2)
            {
                bestDist2=dist;
            }
        }

        if(bestDist<=TH_LOW)
        {
            if(bestDist<(float)bestDist2*mfNNratio)
            {
                if(vnMatches21[bestIdx2]>=0)
                {
                    vnMatches12[vnMatches21[bestIdx2]]=-1;
                    nmatches--;
                }
                vnMatches12[i1]=bestIdx2;
                vnMatches21[bestIdx2]=i1;
                vMatchedDistance[bestIdx2]=bestDist;
                nmatches++;

                if(mbCheckOrientation)
                {
                    float rot = kp1.angle - pF2->GetKeyPoint(bestIdx2).angle;
                    if(rot<0.0)
                        rot+=360.0f;
                    int bin = round(rot*factor);
                    if(bin==HISTO_LENGTH)
                        bin=0;
                    assert(bin>=0 && bin<HISTO_LENGTH);
                    rotHist[bin].push_back(i1);
                }
            }
        }

    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(int idx1 : rotHist[i])
            {
                if(vnMatches12[idx1]>=0)
                {
                    vnMatches21[vnMatches12[idx1]]=-1;
                    vnMatches12[idx1]=-1;
                    nmatches--;
                }
            }
        }
    }

    // convert to DMatch
    vMatches12.clear();
    for(int i = 0; i < vnMatches12.size(); i++)
    {
        if(vnMatches12[i] < 0)
            continue;
        int distance = DescriptorDistance(pF1->GetDescriptor(i), pF2->GetDescriptor(vnMatches12[i]));
        vMatches12.emplace_back(i, vnMatches12[i], distance);
    }

    return nmatches;
}

int PointMatcher::SearchByProjection(const KeyFramePtr& pKF, const std::vector<MapPointPtr> &vpMapPoints, std::vector<cv::DMatch>& vLocalMatches, const float r) const
{
    // 1->KF, 2->LocalMP
    std::vector<int> vnMatches12(pKF->GetNumPoints(),-1);
    // distance for KF
    vector<int> vnMatchedDistance(pKF->GetNumPoints(),INT_MAX);

    const SE2 Tbw = pKF->GetPoseTbw();

    for(int i = 0; i < vpMapPoints.size(); i++)
    {
        MapPointPtr pMP = vpMapPoints[i];

        if(pKF->hasObservation(pMP))
            continue;

        cv::Point2f pt = Frame::ProjectXY2Birdview(Tbw * pMP->GetPos());

        if(!Frame::PosInImage(pt))
            continue;

        const vector<size_t> vIndices = pKF->GetFeaturesInArea(pt.x,pt.y,r);

        if(vIndices.empty())
            continue;

        pMP->UpdateDescriptor();
        const cv::Mat MPdescriptor = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestLevel = -1;
        int bestDist2 = INT_MAX;
        int bestLevel2 = -1;
        int bestIdx = -1 ;

        // Get best and second matches with near keypoints
        for(int idx : vIndices)
        {
            if(pKF->hasObservation(idx))
                continue;

            const cv::Mat d = pKF->GetDescriptor(idx);
            const int dist = DescriptorDistance(MPdescriptor,d);

            if(dist >= vnMatchedDistance[idx])
                continue;

            if(dist < bestDist)
            {
                bestDist2 = bestDist;
                bestDist = dist;
                bestLevel2 = bestLevel;
                bestLevel = pKF->GetKeyPoint(idx).octave;
                bestIdx = idx;
            }
            else if(dist < bestDist2)
            {
                bestLevel2 = pKF->GetKeyPoint(idx).octave;
                bestDist2 = dist;
            }
        }

        // Apply ratio to second match (only if best and second are in the same scale level)
        if(bestDist <= TH_HIGH)
        {
            if(bestLevel == bestLevel2 && bestDist > mfNNratio * bestDist2)
                continue;

            vnMatches12[bestIdx] = i;
            vnMatchedDistance[bestIdx] = bestDist;
        }
    }

    // convert to DMatch
    vLocalMatches.clear();
    for(int i = 0; i < vnMatches12.size(); i++)
    {
        if(vnMatches12[i] < 0)
            continue;
        vLocalMatches.emplace_back(i, vnMatches12[i], 0);
    }

    return vLocalMatches.size();
}

}  // namespace birdview