//
// Created by mrwhite on 2021/3/27.
//

#include "KeyLineGeometry.h"
#include "DUtils/Random.h"

namespace birdview
{

cv::Point3f KeyLineGeometry::GetKeyLineCoeff(const KeyLine& kl)
{
    cv::Point2f sp_ = kl.getStartPoint(), ep_ = kl.getEndPoint();
    cv::Point3f sp(sp_.x, sp_.y, 1.0), ep(ep_.x, ep_.y, 1.0);
    cv::Point3f le = sp.cross(ep);
    le = le / sqrt(le.x * le.x + le.y * le.y);
    return le;
}

cv::Point3f KeyLineGeometry::GetKeyLineIntersect(const KeyLine& kl1, const KeyLine& kl2)
{
    cv::Point3f le1 = GetKeyLineCoeff(kl1);
    cv::Point3f le2 = GetKeyLineCoeff(kl2);
    cv::Point3f intersect = le1.cross(le2);
    return intersect;
}

int KeyLineGeometry::FindMajorDirection(const std::vector<KeyLine>& vKeyLines, std::vector<bool>& status, cv::Point3f& dir1_out, cv::Point3f& dir2_out, const float thDist)
{
    int N = vKeyLines.size();
    std::vector<cv::Point3f> vKeyLineCoeffs(N);
    for(int i = 0; i < N; i++)
    {
        vKeyLineCoeffs[i] = GetKeyLineCoeff(vKeyLines[i]);
    }

    status = std::vector<bool>(N,false);

    int bestCount = 0;
    int bestIdx = -1;

    for(int i = 0; i < N; i++)
    {
        const cv::Point3f& le = vKeyLineCoeffs[i];  // (a, b, c)
        cv::Point3f dir1(le.x, le.y, 0.0);  // (a, b, 0) -> infinity of orthogonal line
        cv::Point3f dir2(le.y, -le.x, 0.0); // (b, -a, 0) -> infinity of line

        int count = 0;
        for(const cv::Point3f& le_i : vKeyLineCoeffs)
        {
            float dist1 = fabs(le_i.dot(dir1));
            float dist2 = fabs(le_i.dot(dir2));

            if(dist1 < thDist || dist2 < thDist)
            {
                count++;
            }
        }

        if(count > bestCount)
        {
            bestCount = count;
            bestIdx = i;
        }
    }

    const cv::Point3f& le = vKeyLineCoeffs[bestIdx];  // (a, b, c)
    cv::Point3f dir1(le.x, le.y, 0.0);  // (a, b, 0) -> infinity of orthogonal line
    cv::Point3f dir2(le.y, -le.x, 0.0); // (b, -a, 0) -> infinity of line

    for(int i = 0; i < N; i++)
    {
        const cv::Point3f& le_i = vKeyLineCoeffs[i];
        float dist1 = fabs(le_i.dot(dir1));
        float dist2 = fabs(le_i.dot(dir2));

        if(dist1 < thDist || dist2 < thDist)
        {
            status[i] = true;
        }
    }

    // correct major direction
    if(dir1.y < 0) dir1 *= -1;
    if(dir2.y < 0) dir2 *= -1;
    if(dir1.x > 0 || dir1.y == 0)
    {
        dir1_out = dir1;
        if(dir1.x < 0)
            dir1_out *= -1;
        dir2_out = dir2;
    }
    else
    {
        dir1_out = dir2;
        if(dir1_out.x < 0)
            dir1_out *= -1;
        dir2_out = dir1;
    }

    return bestCount;
}

bool KeyLineGeometry::GenerateRansacIndices(int TotalNum, int SelectNum, int MaxIterations, std::vector<std::vector<size_t>>& vSets)
{
    assert(SelectNum <= TotalNum);

    // Indices for minimum set selection
    std::vector<size_t> vAllIndices;
    vAllIndices.reserve(TotalNum);
    std::vector<size_t> vAvailableIndices;

    for(int i = 0; i < TotalNum; i++)
    {
        vAllIndices.push_back(i);
    }

    // Generate sets of n points for each RANSAC iteration
    vSets = std::vector<std::vector<size_t>>(MaxIterations,std::vector<size_t>(SelectNum,0));

    DUtils::Random::SeedRandOnce(0);

    for(int it = 0; it < MaxIterations; it++)
    {
        vAvailableIndices = vAllIndices;

        // Select a minimum set
        for(size_t j = 0; j < SelectNum; j++)
        {
            int randi = DUtils::Random::RandomInt(0,vAvailableIndices.size()-1);
            int idx = vAvailableIndices[randi];

            vSets[it][j] = idx;

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }
    }

    return true;
}

}  // namespace birdview