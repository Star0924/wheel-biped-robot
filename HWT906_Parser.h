#pragma once
#include <Arduino.h>

namespace HWT906Cmd {
  constexpr uint8_t UNLOCK[5]     = {0xFF, 0xAA, 0x69, 0x88, 0xB5}; //解鎖
  constexpr uint8_t SAVE[5]       = {0xFF, 0xAA, 0x00, 0x00, 0x00}; //保存
  constexpr uint8_t EXIT_CALI[5]  = {0xFF, 0xAA, 0x01, 0x00, 0x00}; //離開校正(某些需要用)
  constexpr uint8_t ANGLE_CALI[5] = {0xFF, 0xAA, 0x01, 0x08, 0x00}; //XY軸角歸零
  constexpr uint8_t ACC_CALI[5]   = {0xFF, 0xAA, 0x01, 0x01, 0x00}; //加速度歸零
  constexpr uint8_t YAW_CALI[5]   = {0xFF, 0xAA, 0x01, 0x04, 0x00}; //航向角歸零
  constexpr uint8_t AXIS6[5]      = {0xFF, 0xAA, 0x24, 0x01, 0x00}; //6軸切換指令
  constexpr uint8_t AXIS9[5]      = {0xFF, 0xAA, 0x24, 0x00, 0x00};  //9軸切換指令
  constexpr uint8_t READ_ANGLE[5] = {0xFF, 0xAA, 0x27, 0x3D, 0x00}; // 主動查詢姿態角度
}; //指令集

namespace HWT906Scale {
  constexpr float ACC   = 16.0f / 32768.0f;
  constexpr float GYRO   = 2000.0f / 32768.0f;
  constexpr float ANGLE = 180.0f / 32768.0f;
}; //換算

struct IMUData {
  float acc[3]   = {0};
  float gyro[3]  = {0};
  float angle[3] = {0};
  float temperature = 0.0f;
};

enum class FrameType : uint8_t{
  ACC_F = 0x51,
  GYRO_F = 0x52,
  ANGLE_F = 0x53
};




class HWT906_Parser{
  public:
    IMUData data;

    HWT906_Parser();

    bool parseByte(uint8_t b);

    void reset();
  
  private:
    enum class State {
        WAIT_HEADER,
        WAIT_TYPE,
        RECEIVE_DATA
    };

    State state = State::WAIT_HEADER;
    uint8_t buffer[11];
    uint8_t count = 0;

    bool verifyChecksum();
    void decode();
};