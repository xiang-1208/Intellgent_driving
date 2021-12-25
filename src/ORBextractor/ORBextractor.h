#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <iostream>
#include <opencv/cv.h>
#include <list>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

using namespace std;
using namespace cv;

class ORBextractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    ORBextractor(int nfeatures = 1000, float scaleFactor = 1.2f, int nlevels = 8, int scoreType=FAST_SCORE, int fastTh = 20);

    ~ORBextractor(){}

    // Compute the ORB features and descriptors on an image
    void operator()( cv::InputArray image,std::vector<cv::KeyPoint>& keypoints,cv::OutputArray descriptors);

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}


protected:

    void ComputePyramid(cv::Mat image);
    void ComputeKeyPoints(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);

    int nfeatures;
    double scaleFactor;
    int nlevels;
    int scoreType;
    int fastTh;

    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;

    std::vector<cv::Mat> mvImagePyramid;

};

#endif

