#ifndef INITIAL_H
#define INITIAL_H

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


class Initial
{
public:
    Initial(VideoCapture);
    bool start();
    bool end(char*);

private:
    VideoCapture car_capture;
    Mat start_img;
    Mat end_img;
    bool intial_run(unsigned short);
    void pose_estimatiob_2d2d(
        vector<KeyPoint> keypoints_1,
        vector<KeyPoint> keypoints_2,
        vector<DMatch> matches,
        Mat &R, Mat &t);
    Point2d pixel2cam(const Point2d &p, const Mat &K);
};

#endif //INITIAL_H