#pragma once
#include <Arduino.h>

namespace HWT906Cmd {
  constexpr uint8_t UNLOCK[5]     = {0xFF, 0xAA, 0x69, 0x88, 0xB5}; // 解鎖(進入設定模式)
  constexpr uint8_t SAVE[5]       = {0xFF, 0xAA, 0x00, 0x00, 0x00}; // 保存設定
  constexpr uint8_t EXIT_CALI[5]  = {0xFF, 0xAA, 0x01, 0x00, 0x00}; // 離開校正(部分校正流程需要)
  constexpr uint8_t ANGLE_CALI[5] = {0xFF, 0xAA, 0x01, 0x08, 0x00}; // XY軸角度歸零
  constexpr uint8_t ACC_CALI[5]   = {0xFF, 0xAA, 0x01, 0x01, 0x00}; // 加速度歸零
  constexpr uint8_t YAW_CALI[5]   = {0xFF, 0xAA, 0x01, 0x04, 0x00}; // 航向角歸零
  constexpr uint8_t AXIS6[5]      = {0xFF, 0xAA, 0x24, 0x01, 0x00}; // 切換6軸模式
  constexpr uint8_t AXIS9[5]      = {0xFF, 0xAA, 0x24, 0x00, 0x00}; // 切換9軸模式
  constexpr uint8_t READ_ANGLE[5] = {0xFF, 0xAA, 0x27, 0x3D, 0x00}; // 主動查詢姿態角度
}

namespace HWT906Scale {
  constexpr float ACC   = 16.0f   / 32768.0f;  // 加速度 LSB -> g
  constexpr float GYRO  = 2000.0f / 32768.0f;  // 角速度 LSB -> deg/s
  constexpr float ANGLE = 180.0f  / 32768.0f;  // 角度   LSB -> deg
}

struct IMUData {
  float acc[3]   = {0};   // ax, ay, az (g)
  float gyro[3]  = {0};   // gx, gy, gz (deg/s)
  float angle[3] = {0};   // roll, pitch, yaw (deg)
  float temperature = 0.0f;
};

enum class FrameType : uint8_t {
  ACC_F   = 0x51,
  GYRO_F  = 0x52,
  ANGLE_F = 0x53
};

// 負責把 WitMotion 協定的原始 byte stream 解析成 IMUData
class HWT906_Parser {
  public:
    IMUData data;

    HWT906_Parser();

    bool parseByte(uint8_t b);  // 每收到一個byte就餵進來；解出完整一幀時回傳true
    void reset();

  private:
    enum class State {
        WAIT_HEADER,   // 等待 0x55 (幀頭)
        WAIT_TYPE,     // 等待 frame type (0x51/0x52/0x53)
        RECEIVE_DATA   // 收集 8 bytes data + 1 byte checksum
    };

    State state = State::WAIT_HEADER;
    uint8_t buffer[11];   // 0x55 + type + 8 data bytes + checksum = 11 bytes
    uint8_t count = 0;

    bool verifyChecksum();
    void decode();
};

// 對外操作介面：初始化、輪詢更新、以及各種校正指令
class HWT906 {
  public:
    HWT906();

    void begin(HardwareSerial &serialPort, uint32_t baud);

    bool update();                     // 每個loop()呼叫一次，把序列埠收到的byte都餵給parser
    const IMUData& getData() const;

    // ==== 控制與校正命令 API (皆為阻塞式，內含 delay) ====
    void zeroXY();               // XY軸角度歸零
    void zeroYaw();               // Z 軸偏航角歸零
    void calibrateAcc();          // 加速度校準 (需保持模組靜止)
    void switchTo6Axis();         // 切換至 6軸模式
    void switchTo9Axis();         // 切換至 9軸模式
    void readAngle();             // 主動詢問當前角度 (Polling Mode 用)

  private:
    HardwareSerial* serial = nullptr;
    HWT906_Parser   parser;

    void sendCommand(const uint8_t cmd[5]);
    void unlock();
    void save();
};