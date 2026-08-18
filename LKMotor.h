//檔名LKMotor.h
#pragma once
#include "Arduino.h"
#include <vector>

#define BAUDRATE 4000000 //Use motor's MAX baudrate as default
#define SERIAL_PORT 3

class LKMotor
{
  public:
    LKMotor();
    LKMotor(int id, int reduction_ratio, int serial_port);
    void set_value(int id, int reduction_ratio, int serial_port);
    void Serial_Init();
    void sendFrame(uint8_t cmd, uint8_t dataLen, const uint8_t* data);            // 命令封包
    void Read_Motor_Status2();                                                  //(3)讀取馬達狀態2
    void Set_Motor_Origin();                                                    //(8)設置馬達零點
    void Read_Angle_MultiRound();                                               //(9)讀取多圈角度命令
    void Write_angularvel_MultiRound(double angularvel);                        //(11)速度閉環控制命令1
    void Write_Motor_Disable();                                                 //(15)電機關機命令
    void Write_Motor_Enable();                                                  //(17)電機運行命令                                                  
    void Write_Angle_MultiRound(double angle);                                  //(21)多圈位置閉環控制命令1
    void Write_Angle_MultiRound(double angle, long max_speed);                  //(22)多圈位置閉環控制命令2
    void Set_Motor_MultiOrigin();                                               //(24)設多圈原點
    double anglecheck(double angle, double pre_angle);

    double motor_angle = 0.0;     //馬達角度
    double motor_current = 0 ;    //馬達轉矩電流
    double motor_temperature = 0;   //馬達溫度
    double motor_dspeed = 0;   //馬達轉速

  private:
    Stream* MOTOR_SERIAL = &Serial1;
    int _id = 0;              //馬達設定的id
    int _reduction_ratio = 0; //馬達的減速比
    int _serial_port = 0;     //馬達使用的serial port
    int _baudrate = BAUDRATE; //馬達設定的baudrate

    byte _buffer[80];         //要傳送之封包

    void Unpack();
    void parseFrame();
    void parseAngle();
    void parseStatus2();
    byte readin[50];  //收到之封包
    int count_rx = 0; //計算當前封包之長度
    unsigned long RXWaitingStartTime = 0; // Start time of waiting motor package sand back.
    unsigned long RXWaitTime = 400; // unit: microseconds
};



