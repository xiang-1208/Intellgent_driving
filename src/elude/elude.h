#ifndef ELUDE_H
#define ELUDE_H

#include <iostream>
#include <opencv2/opencv.hpp> 
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "../serial/Serial.h"
#include "../Frame/Frame.h"
#include "../ORBextractor/ORBextractor.h"
#include <mutex>
#include <string>
// #include <chrono>

using namespace std;
using namespace cv;

enum {WORKING=0,INiTAIL=1};

class Eludeing
{
public:
    Eludeing(VideoCapture);
    void run();

private:
    int mState=WORKING;
    Frame mCurrentFrame;
    Frame mLastFrame;
    cv::Mat mK;
    VideoCapture car_capture;
    Mat Image;  
    std::mutex mtx;
    ORBextractor* mpORBextractor;
    ORBextractor* mpIniORBextractor;
    std::vector<cv::Mat> mvImagePyramid;
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