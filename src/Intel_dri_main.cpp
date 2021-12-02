/********************************************************************************
Copyright 2018 Huazhong University of Science & XIANG
*********************************************************************************/

#include <iostream>
#include "drive/drive.h"


using namespace std;

int main() 
{
    cout<<"begin!"<<endl;
    Drive car_drive;
    car_drive.run();
    cv::waitKey(0);
    return 0;
}

// //逼近多边形曲线
// #include "opencv2/opencv.hpp"
// using namespace cv;

// int main()
// {
//     Mat srcImg = imread("../data/1.png");
//     imshow("src", srcImg);
//     Mat dstImg = srcImg.clone();
//     Mat dstImg2(srcImg.size(), CV_8UC3, Scalar::all(0));//全黑图像

//     GaussianBlur(srcImg, srcImg, Size(3, 3), 0, 0);
//     cvtColor(srcImg, srcImg, CV_BGR2GRAY);
//     //Canny(srcImg, srcImg, 100, 200);
//     threshold(srcImg, srcImg, 200, 255, CV_THRESH_BINARY_INV);
//     imshow("threshold", srcImg);

//     vector<vector<Point>> contours;
//     vector<Vec4i> hierarcy;
//     findContours(srcImg, contours, hierarcy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
//     vector<vector<Point>> contours_poly(contours.size());

//     for (int i = 0; i<contours.size(); i++)
//     {
//         approxPolyDP(Mat(contours[i]), contours_poly[i],1, true);//true曲线封闭，反之不封闭
//         drawContours(dstImg, contours, i, Scalar(0, 255, 0), 2, 8);//绘制轮廓
//         drawContours(dstImg2, contours_poly, i, Scalar(0, 255, 255), 2, 8);  //绘制多边形逼近
//     }
//     imshow("dst", dstImg);
//     imshow("approx", dstImg2);

//     waitKey(0);
//     return 0;
// }