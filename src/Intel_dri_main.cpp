/********************************************************************************
Copyright 2018 Huazhong University of Science & XIANG
*********************************************************************************/

#include <iostream>
#include "drive/drive.h"
#include "extract/f_extract.h"

using namespace std;

int main() 
{
    cout<<"begin!"<<endl;
    Mat Image = imread("../data/Image2.png");  
    Extract my_extract(Image);
    // Drive car_drive;
    // car_drive.run();
    // // cv::waitKey(0);
    return 0;
}