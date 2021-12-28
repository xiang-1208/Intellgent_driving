#include <opencv2/opencv.hpp> 
#include "serial/Serial.h"

using namespace cv;
using namespace std;

bool find_elude(vector<vector<Point>> contours);
void sendto_car(bool,double,double,Serial);

// #define SERIAL

int main ()
{
    Mat img,img_end,dst;
    double dis_x, angle;
    VideoCapture car_capture;
    vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
    #ifdef SERIAL
    Serial car_serial;
    if(!car_serial.open("/dev/ttyUSB0",115200,0,8,1)) {
        cout << "open port fail" << endl;
        exit(-1);
    }
    #endif // !end SERIAL
    RNG rng(0);
    bool flag = false;
    if(!car_capture.open("../data/test.mp4"))
        cout << "video open fail!" << endl;
    else
        cout << "video open success!" << endl;
    int HSV_int[4] = {120,210,140,50};
    int HSVrange[6];
    // img = imread("../data/end.jpg");
    // imshow ("init",img);
    // setMouseCallback("init", on_mouse, &img);
    while (true) 
    {
        static int num = 0;
        num ++ ;
        car_capture>>img;
        if(img.empty()){
            break;;
        }
        if (num == 30)
        {   
            num = 0;

            //cout <<  img.size() <<endl;
            //因为原图上方有相似色域的衣服的人，故去之调试
            img=img(Rect(0,45,640,435));
            // img = imread("../data/Image2.png");
            if(img.channels()==3)
                cvtColor(img, img, CV_RGB2HSV);
            // imshow ("init",img);
            HSVrange[0] = HSV_int[0] - HSV_int[3];
            HSVrange[1] = HSV_int[0] + HSV_int[3];
            HSVrange[2] = HSV_int[1] - HSV_int[3];
            HSVrange[3] = HSV_int[1] + HSV_int[3];
            HSVrange[4] = HSV_int[2] - HSV_int[3];
            HSVrange[5] = HSV_int[2] + HSV_int[3];

            inRange(img,Scalar(HSVrange[0],HSVrange[2],HSVrange[4]),
                    Scalar(HSVrange[1],HSVrange[3],HSVrange[5]),img_end);
            Mat kernel = getStructuringElement(MORPH_RECT, cv::Size(8, 8));
            Mat kernel_open = getStructuringElement(MORPH_RECT, cv::Size(2, 2));
            morphologyEx(img_end, img_end,MORPH_OPEN, kernel_open);
            morphologyEx(img_end, img_end,MORPH_CLOSE, kernel);

            // imshow ("end",img_end);
            // waitKey(10);


            findContours(img_end,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
	        // Mat imageContours=Mat::zeros(img_HSV.size(),CV_8UC1);

            // cout << contours.size() <<endl;
            img_end.copyTo(dst);
            if (contours.size() == 3)
            {
                for(int i = 0; i < contours.size(); i++)
                {
                    Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
                    drawContours(dst, contours, i, color, 2, 8, hierarchy, 0, Point(0,0));
                }
                //imshow("output",dst);
                flag = find_elude(contours);
                    
                //waitKey(10);
            }
        }

        /*
        //此处填入车道线识别代码
        */

        #ifdef SERIAL
        sendto_car(flag,dis_x,angle,car_serial);
        #endif

        //waitKey(10);
    }
    
    //waitKey(0);
}


bool find_elude(vector<vector<Point>> contours)
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
    float distance = sqrt(12000)/(sqrt(((x_max - x_min) * (y_max - y_min))));
    cout << "distance: " << distance<< endl;
    if (distance <= 1)
        return true;
    else
        return false;
}

void sendto_car(bool flag,double dis_x,double angle,Serial car_serial)
{
    unsigned short send_angle = angle + 32768;
    unsigned short send_dis_x = dis_x + 32768;

    unsigned char sendBuffer[7];
    sendBuffer[0] = '#';
    if (flag)
        // 到达需要转向的距离
        sendBuffer[1] = 'T';
    else
        // 未到达需要转向的距离
        sendBuffer[1] = 'F';
    sendBuffer[2] = (send_dis_x >> 8) & 0xff;
    sendBuffer[3] = send_dis_x & 0xff;
    sendBuffer[4] = (send_angle >> 8) & 0xff;
    sendBuffer[5] = send_angle & 0xff;
    sendBuffer[6] = '!';
    car_serial.send(sendBuffer, sizeof(sendBuffer));
}


