#include <iostream>
#include <opencv2/opencv.hpp> 
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <mutex>
// #include <chrono>

using namespace std;
using namespace cv;
// #include "drive/drive.h"


using namespace std;

int main() 
{
    cout<<"begin!"<<endl;
    Mat img_1;
    Mat img_2;
    img_1 = imread ("../data/start.jpg");
    img_2 = imread ("../data/end.jpg");

    Mat imageROI=image(Rect(260,30,45,75))

    imshow ("start.jpg",img_1);
    imshow ("end.jpg",img_2);
    cv::waitKey(0);
    return 0;
}
