#ifndef F_extract_H
#define F_extract_H

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <vector>

using namespace std;
using namespace cv;

#define pi 3.1415926


class Extract{
public:
    Extract(Mat);
    void result();
    double return_raw();
    Mat return_word();
private:
    Mat Image;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    Mat word;
    double raw;

    bool RectDetect (vector<vector<Point>>&);
    bool isRotationMatrix(Mat&);
    double rotationMatrixToEulerAngles(Mat);
};

#endif //F_extract_H