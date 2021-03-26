#include <iostream>
#include <vector>
#include <thread>

#include <opencv2/opencv.hpp>

#include"System.h"

#include <file_system.hpp>
#include "cmdLine.h"

using namespace std;
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

    SystemPtr pSystem = std::make_shared<System>();
    pSystem->Start();

    int count = 0;
    for(const string& image_file : vstrImageFiles)
    {
        cv::Mat image = cv::imread(image_file, cv::IMREAD_UNCHANGED);
        cv::Mat mask = cv::imread(vstrMaskFiles[count], cv::IMREAD_UNCHANGED);

        pSystem->TrackImage(image, mask);

        cv::waitKey(30);
        count++;
    }

    pSystem->RequestFinish();

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
