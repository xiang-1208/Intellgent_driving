#include "drive.h"

// #define SERIAL

Drive::Drive()
{
    //摄像头
    if(!car_capture.open(0)){
        if(!car_capture.open(1)) {
            cout << "capture open fail" << endl;
            exit(-1);
        }
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
    while(waitKey(1)!= 27) //esc
    {
        if(state==Follow){
            cout<<"state= Parking"<<endl;
        }
        else if(state==Finding){
            cout<<"state= Parking"<<endl;
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
    while (waitKey(1)!=27) {
        #ifdef SERIAL
        int n = car_serial.receive(100);
        if ((n > 0) && (car_serial.buf[0] == 'd')) {
            state = Nothing;
            doNothing()
            break;
        }  
        #endif // !end SERIAL
        car_capture >> Image;
        if (Image.empty()) {
            cout << "image empty!" << endl;
            break;
        }
        imshow("video test", Image);    
    }
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