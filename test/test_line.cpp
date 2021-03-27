//
// Created by mrwhite on 2021/3/26.
//

#include <iostream>
#include <fstream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <file_system.hpp>
#include <line_descriptor_custom.hpp>
#include "cmdLine.h"
#include "DUtils/Random.h"
#include "KeyLineGeometry.h"

using namespace std;
using namespace slam::line_descriptor;
using namespace birdview;

bool LoadDataset(const string& strDatasetPath, vector<string>& vstrImageFiles);

int main(int argc, char** argv)
{
    string strDatasetPath;
    string strMaskPath;

    CmdLine cmd;
    cmd.add(make_option('i', strDatasetPath, "dataset"));
    cmd.add(make_option('m', strMaskPath, "mask"));
    cmd.process(argc, argv);

    if(strDatasetPath.empty() || !stlplus::folder_exists(strDatasetPath))
    {
        cerr << "Dataset path empty or does not exist : " << strDatasetPath << endl;
        return EXIT_FAILURE;
    }

    if(strMaskPath.empty() || !stlplus::folder_exists(strMaskPath))
    {
        cerr << "Mask path empty or does not exist : " << strMaskPath << endl;
        return EXIT_FAILURE;
    }

    vector<string> vstrImageFiles, vstrMaskFiles;
    LoadDataset(strDatasetPath, vstrImageFiles);
    LoadDataset(strMaskPath, vstrMaskFiles);
    cout << "Read " << vstrImageFiles.size() << " images." << endl;
    assert(vstrImageFiles.size() == vstrMaskFiles.size());

    cv::Ptr<LSDDetectorC> lsd = LSDDetectorC::createLSDDetectorC();
    cv::Ptr<BinaryDescriptor> lbd = BinaryDescriptor::createBinaryDescriptor();

    int count = 0;
    for(const string& image_file : vstrImageFiles)
    {
        cv::Mat image = cv::imread(image_file, cv::IMREAD_UNCHANGED);
        cv::Mat mask = cv::imread(vstrMaskFiles[count], cv::IMREAD_UNCHANGED);

        std::vector<KeyLine> vKeyLines;
        lsd->detect(image,vKeyLines,1,1,mask);
        vector<bool> length_status(vKeyLines.size(),false);
        for(int i = 0; i < vKeyLines.size(); i++)
        {
            if(vKeyLines[i].lineLength > 20)
                length_status[i] = true;
        }
        KeyLineGeometry::reduceVector(vKeyLines,length_status);
//         lbd->detect(image,vKeyLines,mask);

        cv::Mat keylineImg;
        drawKeylines(image,vKeyLines,keylineImg);
        cv::imshow("Original", keylineImg);

        vector<bool> status;
        cv::Point3f dir1, dir2;
        int major = KeyLineGeometry::FindMajorDirection(vKeyLines, status, dir1, dir2);
        cout << "Major lines = " << major << endl;
        cout << "dir1 = " << dir1 << endl;
        cout << "dir2 = " << dir2 << endl;

        vector<KeyLine> vMajorLies, vRestLines;
        for(int i = 0; i < status.size(); i++)
        {
            if(status[i])
            {
                vMajorLies.push_back(vKeyLines[i]);
            }
            else
            {
                vRestLines.push_back(vKeyLines[i]);
            }
        }

        cv::Mat majorImg;
        drawKeylines(image,vMajorLies,majorImg);
        cv::imshow("MajorLines", majorImg);

        cv::waitKey(0);
        count++;
    }

    return 0;
}


bool LoadDataset(const string& strDatasetPath, vector<string>& vstrImageFiles)
{
    if(strDatasetPath.empty() || !stlplus::folder_exists(strDatasetPath))
    {
        cerr << "Dataset path empty or does not exist : " << strDatasetPath << endl;
        return false;
    }

    vector<string> files = stlplus::folder_files(strDatasetPath);

    vstrImageFiles.clear();
    for(const string& file : files)
    {
        string ext = stlplus::extension_part(file);
        if(ext == "jpg" || ext == "png" || ext == "JPG" || ext == "PNG")
        {
            vstrImageFiles.push_back(stlplus::filespec_to_path(strDatasetPath, file));
        }
    }

    sort(vstrImageFiles.begin(), vstrImageFiles.end());

    return true;
}


int FindMajorDirection(const vector<KeyLine>& vKeyLines, vector<bool>& status)
{
    const double thPrl = 0.1;
    const double thDist = 0.5;
    const int MaxIterations = 200;
    const int N = vKeyLines.size();

    std::vector<cv::Point3f> vKeyLineCoeffs(N);
    for(int i = 0; i < N; i++)
    {
        vKeyLineCoeffs[i] = KeyLineGeometry::GetKeyLineCoeff(vKeyLines[i]);
    }

    vector<vector<size_t>> vSets;
    KeyLineGeometry::GenerateRansacIndices(N,2,MaxIterations,vSets);

    status = vector<bool>(N,false);
    int bestCount = 0;

    for(int iter = 0; iter < MaxIterations; iter++)
    {
        // step1. select 2 lines and compute intersection
        const cv::Point3f& le1 = vKeyLineCoeffs[vSets[iter][0]];
        const cv::Point3f& le2 = vKeyLineCoeffs[vSets[iter][1]];
        cv::Point3f intersect = le1.cross(le2);

        if(fabs(intersect.z) >= thPrl)  // parallel lines should intersect at infinity (z = 0)
            continue;

        // step2. count how many keylines pass through the intersect
        int count = 0;
        vector<bool> status_i(N,false);
        for(int i = 0; i < N; i++)
        {
            cv::Point3f le = KeyLineGeometry::GetKeyLineCoeff(vKeyLines[i]);
            float dot = le.dot(intersect);
            if(fabs(dot) < thDist)
            {
                count++;
                status_i[i] = true;
            }
        }

        // step3. compare with current best
        if(count > bestCount)
        {
            bestCount = count;
            status = status_i;
        }
    }

    return bestCount;
}

