#ifndef F_extract_H
#define F_extract_H

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


class Extract{
public:
    Extract(Mat);
    void result();
private:
    Mat Image;

    bool RectDetect (Mat&);
};

#endif //F_extract_H