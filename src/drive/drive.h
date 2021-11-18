#ifndef DRIVE_H
#define DRIVE_H

#include <opencv2/opencv.hpp>
#include "../serial/Serial.h"

using namespace std;
using namespace cv;

enum STATUS {Follow=0,Finding=1,Parking=2,Nothing=3};

class Drive{
public:
    Drive();
    void run();
    
private:
    STATUS state = Parking;

    VideoCapture car_capture;
    Serial car_serial;
    Mat Image;

    void doPark();
    void doNothing();
};

#endif //DRIVE_H