#ifndef DRIVE_H
#define DRIVE_H

#include <opencv2/opencv.hpp>
#include "../serial/Serial.h"
#include "../extract/f_extract.h"
#include "../elude/elude.h"
#include <thread>

using namespace std;
using namespace cv;

enum STATUS {Follow=0,Finding=1,Parking=2,Nothing=3};

class Drive{
public:
    Drive();
    void run();
    
private:
    /*相机模式(T) or 图片测试模式(F)*/
    bool RUNING_MOD = false;
    STATUS state = Follow;

    VideoCapture car_capture;
    Serial car_serial;
    Mat Image;

    void doPark();
    void doNothing();
    void doFollowing();
    void sendto_car(Mat,double);
};

#endif //DRIVE_H