//
// Created by mrwhite on 2021/3/26.
//

#include <iostream>
#include <opencv2/core.hpp>
#include "KeyLineGeometry.h"
#include <line_descriptor_custom.hpp>

using namespace std;
using KeyLine =  slam::line_descriptor::KeyLine;

int main()
{
    cv::Point3f sp1(255.0,255.0,1.0), ep1(257.0,257.0,1.0);
    cv::Point3f sp2(2.0,0.0,1.0), ep2(3.0,1.0,1.0);

    KeyLine kl1, kl2;

    kl1.startPointX = sp1.x;
    kl1.startPointY = sp1.y;
    kl1.endPointX = ep1.x;
    kl1.endPointY = ep1.y;

    kl2.startPointX = sp2.x;
    kl2.startPointY = sp2.y;
    kl2.endPointX = ep2.x;
    kl2.endPointY = ep2.y;

    cout << "sp1 = " << sp1 << ", ep1 = " << ep1 << endl;
    cout << "sp2 = " << sp2 << ", ep2 = " << ep2 << endl;

    cv::Point3f le1 = KeyLineGeometry::GetKeyLineCoeff(kl1);
    cv::Point3f le2 = KeyLineGeometry::GetKeyLineCoeff(kl2);

    cout << "le1 = " << le1 << endl;
    cout << "le2 = " << le2 << endl;

    cv::Point3f inter = KeyLineGeometry::GetKeyLineIntersect(kl1,kl2);
    cout << "intersect = " << inter << endl;

    return 0;
}