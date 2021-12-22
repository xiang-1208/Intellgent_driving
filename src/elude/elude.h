#ifndef ELUDE_H
#define ELUDE_H

#include <iostream>
#include <opencv2/opencv.hpp> 
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "../serial/Serial.h"
#include <mutex>
// #include <chrono>

using namespace std;
using namespace cv;


class Eludeing
{
public:
    Eludeing(Mat);
    void run();

private:
    Mat Image;  
    std::mutex mtx;
    chrono::steady_clock::time_point t1;
    chrono::steady_clock::time_point t2;
    void pose_estimatiob_2d2d(
        vector<KeyPoint> keypoints_1,
        vector<KeyPoint> keypoints_2,
        vector<DMatch> matches,
        Mat &R, Mat &t);
    Point2d pixel2cam(const Point2d &p, const Mat &K);
};

#endif //DRIVE_H