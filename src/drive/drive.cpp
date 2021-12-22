#include "drive.h"

// #define SERIAL

Drive::Drive()
{
    if (RUNING_MOD)
    {
        //摄像头0
        if(!car_capture.open(0))
            cout << "capture open fail!" << endl;
        else
            cout << "capture open success!" << endl;
    }
    else
    {
        //测试视频
        if(!car_capture.open("../data/test.mp4"))
            cout << "video open fail!" << endl;
        else
            cout << "video open success!" << endl;       
    }
    //串口
    #ifdef SERIAL
    if(!car_serial.open("/dev/ttyUSB0",115200,0,8,1)) {
        cout << "open port fail" << endl;
        exit(-1);
    }
    #endif // !end SERIAL
    cout<<"initial OK!!!"<<endl;
}

void Drive::run()
{
    // while(waitKey(1)!= 27) //esc
    {
        if(state==Follow){
            cout<<"state= Follow"<<endl;
            doFollowing();
        }
        else if(state==Finding){
            cout<<"state= Finding"<<endl;
        }
        else if(state==Parking){
            cout<<"state= Parking"<<endl;
            doPark();
        }
        else if(state==Nothing) {
            cout<<"state= Nothing"<<endl;
            doNothing();
        }
        else{
            cout<<"state error"<<endl;
            exit(-1);
        }
    }
}

void Drive::doPark()
{
    // /*显示调试内容*/
    // bool DEBUGGING_MOD = false or not RUNING_MOD;

    while (waitKey(1)!=27) {
        #ifdef SERIAL
        int n = car_serial.receive(100);
        if ((n > 0) && (car_serial.buf[0] == 'd')) {
            state = Nothing;
            doNothing()
            break;
        }  
        #endif // !end SERIAL

        if (RUNING_MOD)
        {
            car_capture >> Image;
            if (Image.empty()) {
                cout << "image empty!" << endl;
                break;
            }
        }
        else
        
        //打开单张图片
        Image = imread("../data/Image2.png");  
        Extract my_extract(Image);
        Mat word = my_extract.return_word();
        double raw = my_extract.return_raw();
        sendto_car(word,raw);
    }
}

void Drive::doFollowing()
{   
    static bool thread_flag = true;
    if (thread_flag)
    {
        Eludeing Eluder(car_capture);
        thread elude_thread(&Eludeing::run,&Eluder);
        elude_thread.join();
        thread_flag = false;
    }

    // /*显示调试内容*/
    // bool DEBUGGING_MOD = false or not RUNING_MOD;

    // while (waitKey(1)!=27) {
    //     #ifdef SERIAL
    //     int n = car_serial.receive(100);
    //     if ((n > 0) && (car_serial.buf[0] == 'd')) {
    //         state = Nothing;
    //         doNothing()
    //         break;
    //     }  
    //     #endif // !end SERIAL

    //     if (RUNING_MOD)
    //     {
    //         car_capture >> Image;
    //         if (Image.empty()) {
    //             cout << "image empty!" << endl;
    //             break;
    //         }
    //     }
    //     else
    //         //打开单张图片
    //         Image = imread("../data/Image2.png");  
    //     Extract my_extract(Image);
    //     Mat word = my_extract.return_word();
    //     double raw = my_extract.return_raw();
    //     sendto_car(word,raw);
    // }
}

void Drive::doNothing()
{
    while(true){
        std::cout<<"waiting for command from stm32"<<std::endl;
        #ifdef SERIAL
        int n=car_serial.receive(1);
        if(n>0 && (car_serial.buf[0]=='c')){
            state=Parking;
            break;
        }
        #endif // !end SERIAL
    }   
}

void Drive::sendto_car(Mat word,double angle)
{
    int dis_x = int(word.at<double>(0,0));
    int dis_y = int(word.at<double>(1,0));
    unsigned short send_angle = angle + 32768;
    unsigned short send_dis_x = dis_x + 32768;
    unsigned short send_dis_y = dis_y + 32768;

    unsigned char sendBuffer[8];
    sendBuffer[0] = '#';
    sendBuffer[1] = (send_angle >> 8) & 0xff;
    sendBuffer[2] = send_angle & 0xff;
    sendBuffer[3] = (send_dis_x >> 8) & 0xff;
    sendBuffer[4] = send_dis_x & 0xff;
    sendBuffer[5] = (send_dis_y >> 8) & 0xff;
    sendBuffer[6] = send_dis_y & 0xff;
    sendBuffer[7] = '!';
    car_serial.send(sendBuffer, sizeof(sendBuffer));

}