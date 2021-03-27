#include <iostream>
#include <fstream>
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

    ofstream f("trajectory.txt");
    if(!f.is_open())
    {
        std::cerr << "Can not open file trajectory.txt" << std::endl;
    }
    f << fixed << setw(8);

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

        SE2 Tbw = pSystem->TrackImage(image, mask);

        cv::waitKey(30);
        count++;

        double time = stod(stlplus::basename_part(image_file));
        SE3::SE3Quat Twb = Tbw.toSE3Quat().inverse();
        const auto& q = Twb.rotation();
        const auto& t = Twb.translation();
        f << time << " " << t[0] << " " << t[1] << " " << t[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }

    f.close();

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

//void LoadDataset(const string &strFile, vector<string> &vstrImageFilenames, vector<string> &vstrMaskFilenames,
//                 vector<cv::Vec3d> &vodomPose, vector<double> &vTimestamps)
//{
//    ifstream f;
//    f.open(strFile.c_str());
//
//    if(!f.is_open())
//    {
//        std::cerr << "Can not open file " << strFile << std::endl;
//        return;
//    }
//
//    while(!f.eof())
//    {
//        string s;
//        getline(f,s);
//        if(!s.empty())
//        {
//            stringstream ss;
//            ss << s;
//            double t;
//            double x,y,theta;
//            string image;
//            ss >> t;
//            vTimestamps.push_back(t);
//            ss>>x>>y>>theta;
//            vodomPose.emplace_back(x,y,theta);
//            ss >> image;
//            vstrImageFilenames.push_back("birdview/"+image);
//            vstrMaskFilenames.push_back("mask/"+image);
//        }
//    }
//    // double t0=vTimestamps[0];
//    // for_each(vTimestamps.begin(),vTimestamps.end(),[t0](double &t){t-=t0;});
//}
