#pragma once
#include <Arduino.h>
#include "HWT906_Parser.h"


class HWT906{
  public:
    HWT906();

    void begin(HardwareSerial &serialPort, uint32_t baud);

    bool update();

    const IMUData& getData() const;

    // ==== 控制與校正命令 API ====
    void zeroXY();               // XY軸角度歸零
    void zeroYaw();              // Z 軸偏航角歸零
    void calibrateAcc();         // 加速度校準
    // void startMagCalibration();  // 進入磁力計校正
    // void stopMagCalibration();   // 退出磁力計校正並儲存
    void switchTo6Axis();        // 切換至 6軸模式
    void switchTo9Axis();        // 切換至 9軸模式
    void readAngle();            // 主動詢問當前角度 (Polling Mode 用) 
  
  private:
    HardwareSerial* serial = nullptr;
    HWT906_Parser   parser;

    // 底層私有封包控制
    void sendCommand(const uint8_t cmd[5]);
    void unlock();
    void save();
};