//檔名LKMotor.h
#pragma once
#include "Arduino.h"
#include <vector>

#define BAUDRATE 4000000 //Use motor's MAX baudrate as default
#define SERIAL_PORT 3

// LK 馬達通訊協定中，Write_Torque_MultiRound() 的電流指令欄位是用
// int16 (-2048~2048) 表示，對應到 -33A~+33A（這是LK驅動器「這個系列」
// 通用的原始數值滿刻度，不代表任何一顆馬達實際能承受這麼大電流）。
// !!重要!! 這個 33A 只是「數值編碼」的滿刻度，不是安全上限。
// 真正的安全上限必須依「馬達規格書」的 Max Current 另外設定，
// 見建構子的 max_current_A 參數與 currentSafetyLimitA()。
constexpr double LKMOTOR_PROTOCOL_MAX_CURRENT_A = 33.0;

class LKMotor
{
  public:
    LKMotor();

    // max_current_A: 這顆馬達允許送出的最大電流（安全上限，單位A）。
    //   Write_Torque_MultiRound() 內部會自動把指令夾限在 ±max_current_A，
    //   不管上層(PID/MPC)算出多大的扭矩指令，都不會超過這顆馬達的實際規格。
    //   預設等於協定滿刻度(33A) = 不設安全上限，等同舊行為；
    //   請務必在建立輪馬達物件時明確傳入馬達規格書上的 Max Current。
    LKMotor(int id, int reduction_ratio, int serial_port,
            double max_current_A = LKMOTOR_PROTOCOL_MAX_CURRENT_A);

    void set_value(int id, int reduction_ratio, int serial_port,
                    double max_current_A = LKMOTOR_PROTOCOL_MAX_CURRENT_A);

    void Serial_Init();

    // ---- 底層封包 ----
    void sendFrame(uint8_t cmd, uint8_t dataLen, const uint8_t* data);            // 命令封包

    // ---- 讀取類指令 ----
    void Read_Motor_Status2();                                                  //(3)讀取馬達狀態2
    void Read_Angle_MultiRound();                                               //(9)讀取多圈角度命令

    // ---- 控制類指令 ----
    void Set_Motor_Origin();                                                    //(8)設置馬達零點
    void Write_Torque_MultiRound(double cur);                                   //(10)轉矩閉環控制命令 (電流會被自動夾限在 ±max_current_A)
    void Write_angularvel_MultiRound(double angularvel);                        //(11)速度閉環控制命令1
    void Write_Motor_Disable();                                                 //(15)電機關機命令
    void Write_Motor_Enable();                                                  //(17)電機運行命令
    void Write_Angle_MultiRound(double angle);                                  //(21)多圈位置閉環控制命令1
    void Write_Angle_MultiRound(double angle, long max_speed);                  //(22)多圈位置閉環控制命令2
    void Set_Motor_MultiOrigin();                                               //(24)設多圈原點

    double anglecheck(double angle, double pre_angle);

    // 目前安全電流上限 (A)，供外部（例如除錯輸出）查詢用
    double currentSafetyLimitA() const { return _maxCurrentA; }

    // ---- 由 parseXxx() 更新，主控迴圈可直接讀取 ----
    double motor_angle = 0.0;       // 馬達角度 (deg，輸出軸)
    double motor_current = 0.0;     // 馬達實際回授轉矩電流 (A)
    double motor_temperature = 0;   // 馬達溫度 (°C)
    double motor_dspeed = 0.0;      // 馬達轉速 (deg/s，輸出軸)

  private:
    Stream* MOTOR_SERIAL = &Serial1;
    int _id = 0;                 // 馬達設定的id
    int _reduction_ratio = 0;    // 馬達的減速比
    int _serial_port = 0;        // 馬達使用的serial port
    int _baudrate = BAUDRATE;    // 馬達設定的baudrate
    double _maxCurrentA = LKMOTOR_PROTOCOL_MAX_CURRENT_A; // 安全電流上限(A)

    byte _buffer[80];            // 要傳送之封包

    // ---- 封包接收/解析 ----
    void Unpack();
    void parseFrame();
    void parseAngle();
    void parseStatus2();
    bool verifyFrameChecksum(uint8_t expectedLen) const;

    byte readin[50];   // 收到之封包
    int count_rx = 0;  // 目前已收到的封包長度(bytes)
    unsigned long RXWaitingStartTime = 0; // 開始等待回覆封包的時間戳
    unsigned long RXWaitTime = 400;       // 等待回覆封包的逾時時間 (單位: 微秒)
};