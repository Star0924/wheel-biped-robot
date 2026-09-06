#include "LKMotor.h"

// ================================================================
// 建構 / 設定
// ================================================================
LKMotor::LKMotor(){
  _id = 0;
  _reduction_ratio = 0;
  _serial_port = 0;
  _maxCurrentA = LKMOTOR_PROTOCOL_MAX_CURRENT_A;
}

LKMotor::LKMotor(int id, int reduction_ratio, int serial_port, double max_current_A){
  _id = id;
  _reduction_ratio = reduction_ratio;
  _serial_port = serial_port;
  _maxCurrentA = max_current_A;
}

void LKMotor::set_value(int id, int reduction_ratio, int serial_port, double max_current_A){
  _id = id;
  _reduction_ratio = reduction_ratio;
  _serial_port = serial_port;
  _maxCurrentA = max_current_A;
}

void LKMotor::Serial_Init() {
  switch(_serial_port){
    case 1: MOTOR_SERIAL = &Serial1; Serial1.begin(_baudrate);break;
    case 2: MOTOR_SERIAL = &Serial2; Serial2.begin(_baudrate);break;
    case 3: MOTOR_SERIAL = &Serial3; Serial3.begin(_baudrate); Serial3.transmitterEnable(13);break;
    case 4: MOTOR_SERIAL = &Serial4; Serial4.begin(_baudrate);break;
    case 5: MOTOR_SERIAL = &Serial5; Serial5.begin(_baudrate); Serial5.transmitterEnable(2);break;
    case 6: MOTOR_SERIAL = &Serial6; Serial6.begin(_baudrate);break;
    case 7: MOTOR_SERIAL = &Serial7; Serial7.begin(_baudrate);break;
  }
}

// ================================================================
// 底層：封包發送
// ================================================================
void LKMotor::sendFrame(uint8_t cmd, uint8_t dataLen, const uint8_t* data) {
    if (MOTOR_SERIAL == nullptr)
        return;

    _buffer[0] = 0x3E; // 頭字節
    _buffer[1] = cmd;  // 命令
    _buffer[2] = _id;  // ID
    _buffer[3] = dataLen; // data字節長度

    // Header Checksum = buffer[0..3] 之和
    uint8_t headerCheckSum = 0;
    for (int i = 0; i < 4; i++) headerCheckSum += _buffer[i];
    _buffer[4] = headerCheckSum;

    if (dataLen > 0 && data != nullptr) {
        memcpy(&_buffer[5], data, dataLen);

        // Data Checksum = data 部分之和
        uint8_t dataCheckSum = 0;
        for (int i = 5; i < 5 + dataLen; i++) dataCheckSum += _buffer[i];
        _buffer[5 + dataLen] = dataCheckSum;
    }

    // 封包總長度：無資料時 5 bytes；有資料時 6+dataLen bytes (多一個data checksum)
    uint8_t totalLen = (dataLen == 0) ? 5 : (6 + dataLen);
    MOTOR_SERIAL->write(_buffer, totalLen);
}

// ================================================================
// 讀取類指令
// ================================================================

//(3)讀取馬達狀態2 (電流/溫度/轉速)
void LKMotor::Read_Motor_Status2(){
  sendFrame(0x9C, 0x00, nullptr);
  count_rx = 0;
  RXWaitingStartTime = micros();
  while ((micros() - RXWaitingStartTime) < RXWaitTime) {
    Unpack();
  }
}

//(9)讀取多圈角度命令
void LKMotor::Read_Angle_MultiRound(){
  sendFrame(0x92, 0x00, nullptr);
  count_rx = 0;
  RXWaitingStartTime = micros();
  while ((micros() - RXWaitingStartTime) < RXWaitTime) {
    Unpack();
  }
}

// ================================================================
// 控制類指令
// ================================================================

//(8)設置馬達零點
void LKMotor::Set_Motor_Origin(){
  sendFrame(0x19, 0x00, nullptr);
}

//(10)轉矩閉環控制命令
//    !! 安全性重點 !!
//    協定欄位是 int16 (-2048~2048) 對應 -33A~+33A(LKMOTOR_PROTOCOL_MAX_CURRENT_A)，
//    但這只是「數值編碼」的滿刻度，不代表馬達真的能吃到33A。
//    這裡一定要先用這顆馬達自己的 _maxCurrentA (建構時依規格書填入) 夾限，
//    這樣不論上層 PID / MPC 算出多離譜的扭矩指令，都不會超過馬達的實際額定電流。
void LKMotor::Write_Torque_MultiRound(double cur){
  double clampedCur = constrain(cur, -_maxCurrentA, _maxCurrentA);

  int16_t _current = (int16_t)(clampedCur * 2048.0 / LKMOTOR_PROTOCOL_MAX_CURRENT_A);

  uint8_t data[2];
  for(int i = 0; i < 2; i++)
    data[i] = (_current >> (8 * i)) & 0xFF;

  sendFrame(0xA1, 0x02, data);
  count_rx = 0;
  RXWaitingStartTime = micros();
  while ((micros() - RXWaitingStartTime) < RXWaitTime) {
    Unpack();
  }
}

//(11)速度閉環控制命令
void LKMotor::Write_angularvel_MultiRound(double angularvel){
  int32_t angular_velocity = (int32_t)(angularvel * 100 * _reduction_ratio); // 單位換算 + 乘上齒輪比

  uint8_t data[4];
  for(int i = 0; i < 4; i++)
    data[i] = (angular_velocity >> (8 * i)) & 0xFF;

  sendFrame(0xA2, 0x04, data);
  count_rx = 0;
  RXWaitingStartTime = micros();
  while ((micros() - RXWaitingStartTime) < RXWaitTime) {
    Unpack();
  }
}

//(15)電機關機命令
void LKMotor::Write_Motor_Disable(){
  sendFrame(0x80, 0x00, nullptr);
}

//(17)電機運行命令
void LKMotor::Write_Motor_Enable(){
  sendFrame(0x88, 0x00, nullptr);
}

//(21)多圈位置閉環控制命令1
void LKMotor::Write_Angle_MultiRound(double angle){
  int64_t angleControl = (int64_t)(angle * 100 * _reduction_ratio); // 單位換算 + 乘上齒輪比

  uint8_t data[8];
  for(int i = 0; i < 8; i++)
    data[i] = (angleControl >> (8 * i)) & 0xFF;

  sendFrame(0xA3, 0x08, data);
}

//(22)多圈位置閉環控制命令2 (含最大速度限制)
void LKMotor::Write_Angle_MultiRound(double angle, long max_speed){
  if (max_speed <= 1) {
    max_speed = 1;
  }

  int64_t angleControl = (int64_t)(angle * 100 * _reduction_ratio);
  uint32_t maxSpeed = (uint32_t)(max_speed * 100 * _reduction_ratio);

  uint8_t data[12];
  for(int i = 0; i < 8; i++)   // 角度
    data[i] = (angleControl >> (8 * i)) & 0xFF;
  for(int i = 0; i < 4; i++)   // 最大速度
    data[i + 8] = (maxSpeed >> (8 * i)) & 0xFF;

  sendFrame(0xA4, 0x0C, data);
  count_rx = 0;
  RXWaitingStartTime = micros();
  while ((micros() - RXWaitingStartTime) < RXWaitTime) {
    Unpack();
  }
}

//(24) 設多圈原點
void LKMotor::Set_Motor_MultiOrigin(){
  uint8_t data[7] = {0};
  sendFrame(0x95, 0x07, data);
}

// ================================================================
// 封包接收 / 解析
// ================================================================

// 接收封包
// 封包格式： [0]=0x3E header  [1]=cmd  [2]=id  [3]=dataLen  [4]=headerChecksum
//            [5..5+dataLen-1]=data (若dataLen>0)  [5+dataLen]=dataChecksum (若dataLen>0)
// 封包總長度 = 5 (dataLen==0時)  或  6+dataLen (dataLen>0時，別忘了最後的data checksum!)
void LKMotor::Unpack(){
    byte temp;

    while (MOTOR_SERIAL->available())
    {
        MOTOR_SERIAL->readBytes(&temp, 1);

        if (temp == 0x3E && count_rx == 0)
        {
            readin[count_rx++] = temp;
        }
        else if (count_rx >= 1)
        {
            readin[count_rx++] = temp;

            // 至少要收到 index 3 (dataLen) 才能算出這個封包完整需要幾個byte
            if (count_rx >= 4) {
                uint8_t dataLen = readin[3];
                // 修正：原本寫成 dataLen+5，漏算了 dataLen>0 時最後的 data checksum 那 1 byte，
                // 會導致每個帶資料的封包，最後一個 checksum byte 都沒被讀走(留在緩衝區裡)，
                // 且完全沒有驗證封包正確性 —— 資料若在傳輸中損毀也不會被發現。
                uint8_t expectedLen = (dataLen == 0) ? 5 : (dataLen + 6);

                if (count_rx == expectedLen)
                {
                    if (verifyFrameChecksum(expectedLen)) {
                        parseFrame();
                    }
                    // checksum 不對就直接丟棄這個封包，不更新任何馬達狀態，
                    // 好過用一筆可能損毀的數據去驅動平衡控制。
                    count_rx = 0;
                    return;
                }
            }

            if (count_rx > 20)
            {
                count_rx = 0;
                return;
            }
        }
    }
}

// 驗證 header checksum 與 (若有資料) data checksum
bool LKMotor::verifyFrameChecksum(uint8_t expectedLen) const {
    uint8_t headerSum = 0;
    for (int i = 0; i < 4; i++) headerSum += readin[i];
    if (headerSum != readin[4]) return false;

    uint8_t dataLen = readin[3];
    if (dataLen > 0) {
        uint8_t dataSum = 0;
        for (int i = 5; i < 5 + dataLen; i++) dataSum += readin[i];
        if (dataSum != readin[5 + dataLen]) return false;
    }
    (void)expectedLen;
    return true;
}

// 判斷封包命令並分派解析
void LKMotor::parseFrame(){
    switch(readin[1])
    {
        case 0x92: parseAngle();   break;
        case 0x9C: parseStatus2(); break;
        case 0xA1: parseStatus2(); break;
        case 0xA2: parseStatus2(); break;
        case 0xA4: parseStatus2(); break;
    }
}

// (9)讀取多圈角度命令解包
void LKMotor::parseAngle(){
    int motor_id = readin[2];
    if (motor_id != _id)
        return;

    int64_t motorAngle =
          ((int64_t)readin[12] << 56)
        | ((int64_t)readin[11] << 48)
        | ((int64_t)readin[10] << 40)
        | ((int64_t)readin[9]  << 32)
        | ((int64_t)readin[8]  << 24)
        | ((int64_t)readin[7]  << 16)
        | ((int64_t)readin[6]  << 8)
        | ((int64_t)readin[5]);

    motor_angle = (double)motorAngle / (100.0 * _reduction_ratio);
}

// (3)讀取馬達狀態2命令解包 (溫度 / 電流 / 轉速)
void LKMotor::parseStatus2(){
    int motor_id = readin[2];
    if (motor_id != _id)
        return;

    motor_temperature = readin[5];

    int16_t current = ((int16_t)readin[7] << 8) | readin[6];
    motor_current = current * 66.0 / 4096.0;

    int16_t speed = (int16_t)(((uint16_t)readin[9] << 8) | readin[8]);
    motor_dspeed = (double)speed / _reduction_ratio;
}