#include <opencv2/opencv.hpp> 
#include "serial/Serial.h"
#include <string>
#include <cmath>

using namespace cv;
using namespace std;

bool find_elude(vector<vector<Point>> contours,double& distance);
void sendto_car(bool,double,double,Serial&);
void lane_detect(Mat img,double&,double&);
void elude(Mat img,bool&,double&);

#define SERIAL

int main ()
{
    Mat img;
    double distance;
    double dis_x, angle;
    VideoCapture car_capture;

    #ifdef SERIAL
    Serial car_serial;
    if(!car_serial.open_port()) {
        cout << "open port fail" << endl;
        exit(-1);
    }
    #endif // !end SERIAL
    
    bool flag = false;
    if(!car_capture.open(0))
    //if(!car_capture.open("../data/test.mp4"))
        cout << "video open fail!" << endl;
    else
        cout << "video open success!" << endl;

    while (true) 
    {
        static int num = 0;
        num ++ ;
        car_capture>>img;
        if(img.empty()){
            break;;
        }
        if (num == 6)
        {   
            num = 0;
            lane_detect(img,dis_x,angle);
            elude(img,flag,distance);
        }

        #ifdef SERIAL
        sendto_car(flag,dis_x,angle,car_serial);
        #endif

        // waitKey(10);
    }    
    return 0;
}


bool find_elude(vector<vector<Point>> contours,double &distance)
{

    int x_max,y_max,x_min,y_min;
    int x_max_temp,y_max_temp,x_min_temp,y_min_temp;
    for(int i = 0; i < contours.size(); i++)
    {  
        vector<int> box_int_y;
        vector<int> box_int_x;
        for (int j=0; j<contours[i].size(); j++)
        {
            box_int_y.push_back(contours[i][j].y);
            // cout << contours[i][j].y << endl;
            // cout << contours[i][j].x << endl;
            box_int_x.push_back(contours[i][j].x);
        }
        vector<int> vec = {-7, 1, 10, 7, 2, 1};
        vector<int>::iterator x_itMax = max_element(box_int_x.begin(), box_int_x.end());
        vector<int>::iterator x_itMin = min_element(box_int_x.begin(), box_int_x.end());
        // cout << "最大值为：" << *x_itMax << " " << "所在位置：" << distance(box_int_x.begin(), x_itMax) << endl;
        // cout << "最小值为：" << *x_itMin << " " << "所在位置：" << distance(box_int_x.begin(), x_itMin) << endl;
        vector<int>::iterator y_itMax = max_element(box_int_y.begin(), box_int_y.end());
        vector<int>::iterator y_itMin = min_element(box_int_y.begin(), box_int_y.end());
        y_min_temp =  *y_itMin;
        x_min_temp =  *x_itMin;
        y_max_temp =  *y_itMax;
        x_max_temp =  *x_itMax;
        if (i == 0)
        {
            y_min = y_min_temp;
            x_min = x_min_temp;
            y_max = y_max_temp;
            x_max = x_max_temp;     
        }  
        else    
        {
            if (y_min_temp < y_min) y_min = y_min_temp;
            if (x_min_temp < x_min) x_min = x_min_temp;
            if (y_max_temp > y_max) y_max = y_max_temp;
            if (x_max_temp > x_max) x_max = x_max_temp;
        }
        vector<int>().swap(box_int_y);
        vector<int>().swap(box_int_x);
    }

    // cout << "y_min" << y_min << endl
    //     << "x_min" << x_min << endl
    //     << "y_max" << y_max << endl
    //     << "x_max" << x_max << endl;
    //12000为人工测试距离，其意指当达到预判位置时路障色块所像素最小外接矩形大小
    distance = sqrt(12000)/(sqrt(((x_max - x_min) * (y_max - y_min))));
    cout << "distance: " << distance<< endl;
    if (distance <= 1.0)
        return true;
    else
        return false;
}

void sendto_car(bool flag,double dis_x,double angle,Serial &car_serial)
{
    unsigned short send_angle = angle + 32768;
    unsigned short send_dis_x = dis_x + 32768;

    unsigned char sendBuffer[8];
    sendBuffer[0] = '#';
    if (flag)
        // 到达需要转向的距离
        sendBuffer[1] = 0x01;
    else
        // 未到达需要转向的距离
        sendBuffer[1] = 0x00;
    sendBuffer[2] = 0xff;
    sendBuffer[3] = (send_dis_x >> 8) & 0xff;
    sendBuffer[4] = send_dis_x & 0xff;
    sendBuffer[5] = (send_angle >> 8) & 0xff;
    sendBuffer[6] = send_angle & 0xff;
    sendBuffer[7] = '!';
    car_serial.send(sendBuffer, sizeof(sendBuffer));
}

void lane_detect(Mat img,double& dis_x,double& angle)
{
    vector<Mat> channels;
    split(img,channels);
    Mat stencil =  channels.at(0);
    Mat mask = Mat::zeros(stencil.size(),CV_8UC1);
    Rect r(50,300,500,180);//    Rect r(mask.cols*0.25,mask.rows*0.4,400,100);
    rectangle(mask,r,Scalar(255),-1);
    Mat m_out;
    bitwise_and(stencil,stencil,m_out,mask);
    imshow("m_out",m_out);

    // 阈值判断
    Mat thresh;
    threshold(m_out,thresh, 60, 1, THRESH_BINARY);
    Mat mask_1 = Mat::ones(stencil.size(),CV_8UC1);
    thresh = mask_1 - thresh;
    Mat thresh_end;
    bitwise_and(thresh, thresh,thresh_end, mask);
    thresh_end.copyTo(thresh);

    Mat kernel_erode = getStructuringElement(MORPH_RECT, cv::Size(7, 7));
    morphologyEx(thresh, thresh,MORPH_ERODE, kernel_erode);

    //Mat p1 = np.float32([[137,258], [417,250], [501,446], [55,472]])
    Mat p1 = (Mat_<float>(4, 2) <<137,258,417,250,501,446,55,472);
    Mat p2 = (Mat_<float>(4, 2) <<0,0,63,0,63,50,0,50);
    p2 *= 6; 
    p2 += 100;
    

    Mat M = getPerspectiveTransform(p1, p2);
    cout << M << endl;
    Mat dst_thresh,img_dst;
    warpPerspective(thresh,dst_thresh, M,thresh.size());
    warpPerspective(img, img_dst,M, img.size());

    vector<Vec4f> plines;
    HoughLinesP(dst_thresh, plines,1, CV_PI/180, 10);
    Mat dmy;
    img_dst.copyTo(dmy);

    static int x1 ,  y1 , x2 , y2 ;
    static int x3 ,  y3 , x4 , y4 ;
    
    int cnt = 0;
    for (auto lines : plines)
    {   
        x1 = (int)lines[0];
        y1 = (int)lines[1];
        x2 = (int)lines[2];
        y2 = (int)lines[3];
        line(dmy, Point(x1, y1), Point(x2, y2), (0,255,255), 3,8);
        if (cnt == 0)
        {
            x3 = x1;
            y3 = y1;
            x4 = x2;
            y4 = y2;
        }
        cnt += 1;
    }

    cout << cnt << endl;

    double k1 = (y2 - y1) / (x2 - x1);  // 如果有一个斜率为负无穷，就代表角度没偏
    double k2 = (y4 - y3) / (x4 - x3);

    double phi;
    if (x2 == x1 and x4 != x3)
        phi = atan(k2/2);
    else if (x4 == x3 and x2 != x1)  // 有一条垂直，另一条不垂直
        phi = atan(k1/2);
    else if (x1 == x2 and x3 == x4)  // 两条都精准垂直
        phi = CV_PI/2;
    else
        phi = atan((k1+k2)/2);  // 传参数：角度. 这个角度的值是弧度制的，比如np.arctan(1) = pi/4 = 0.785
    double x_mid = (x2+x4)/2;

    // 弧度制转化成角度制，如果phi大于0表示往右边偏多少角度，小于0表示往左边偏
    // 照片水平底为横轴，中点且垂直底部的线为纵轴（90°），来算车载摄像头目前与这条线偏多少
    angle = 90 - (phi/CV_PI)*180; // 角度制
    dis_x = ((320-x_mid)/abs(x2-x4))*63;

    // imwrite("..out/result.png",dmy);
}

void elude(Mat img,bool & flag,double& distance)
{
    static int img_num = 0;
    RNG rng(0);
    Mat img_end,dst;
    int HSV_int[4] = {120,210,140,50};
    int HSVrange[6];
    vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
    //cout <<  img.size() <<endl;
    //因为原图上方有相似色域的衣服的人，故去之调试
    img=img(Rect(0,45,640,435));
    // img = imread("../data/Image2.png");
    if(img.channels()==3)
        cvtColor(img, img_end, CV_RGB2HSV);
    // imshow ("init",img);
    HSVrange[0] = HSV_int[0] - HSV_int[3];
    HSVrange[1] = HSV_int[0] + HSV_int[3];
    HSVrange[2] = HSV_int[1] - HSV_int[3];
    HSVrange[3] = HSV_int[1] + HSV_int[3];
    HSVrange[4] = HSV_int[2] - HSV_int[3];
    HSVrange[5] = HSV_int[2] + HSV_int[3];
    inRange(img_end,Scalar(HSVrange[0],HSVrange[2],HSVrange[4]),
            Scalar(HSVrange[1],HSVrange[3],HSVrange[5]),img_end);
    Mat kernel = getStructuringElement(MORPH_RECT, cv::Size(8, 8));
    Mat kernel_open = getStructuringElement(MORPH_RECT, cv::Size(2, 2));
    morphologyEx(img_end, img_end,MORPH_OPEN, kernel_open);
    morphologyEx(img_end, img_end,MORPH_CLOSE, kernel);
    // imshow ("end",img_end);
    // waitKey(10)
    findContours(img_end,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
	// Mat imageContours=Mat::zeros(img_HSV.size(),CV_8UC1)
    // cout << contours.size() <<endl;
    img_end.copyTo(dst);
    if (contours.size() == 3)
    {
        for(int i = 0; i < contours.size(); i++)
        {
            Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
            drawContours(dst, contours, i, color, 2, 8, hierarchy, 0, Point(0,0));
        }
        imshow("output",dst);
        
        flag = find_elude(contours,distance);
        img_num ++;
        putText(img,"distance"+to_string(distance),Point(50,60),FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,0),4,8);//在图片上写文字
        // imwrite("../out/"+to_string(img_num)+".jpg",img);
        // waitKey(1000);
    }
}