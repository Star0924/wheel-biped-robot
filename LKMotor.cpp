#include "LKMotor.h"

LKMotor::LKMotor(){
  _id = 0;
  _reduction_ratio = 0;
  _serial_port = 0;
}

LKMotor::LKMotor(int id, int reduction_ratio, int serial_port){
  _id = id;
  _reduction_ratio = reduction_ratio;
  _serial_port = serial_port;
}

void LKMotor::set_value(int id, int reduction_ratio, int serial_port){
  _id = id;
  _reduction_ratio = reduction_ratio;
  _serial_port = serial_port;
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

// 統一的發送封包私有函式
void LKMotor::sendFrame(uint8_t cmd, uint8_t dataLen, const uint8_t* data) {
    if (MOTOR_SERIAL == nullptr)
    return;

    _buffer[0] = 0x3E; // 頭字節
    _buffer[1] = cmd;  // 命令
    _buffer[2] = _id;   // ID
    _buffer[3] = dataLen; //data字節長度

    // 計算 Header Checksum
    uint8_t headerCheckSum = 0;
    for (int i = 0; i < 4; i++) headerCheckSum += _buffer[i];
    _buffer[4] = headerCheckSum;

    // 填入 Data
    if (dataLen > 0 && data != nullptr) {
        memcpy(&_buffer[5], data, dataLen);
        
        // 計算 Data Checksum
        uint8_t dataCheckSum = 0;
        for (int i = 5; i < 5 + dataLen; i++) dataCheckSum += _buffer[i];
        _buffer[5 + dataLen] = dataCheckSum;
    }

    uint8_t totalLen;
    if (dataLen == 0) {
        totalLen = 5;         // 無資料時，總封包長度為 5 Byte
    } else {
        totalLen = 6 + dataLen; // 有資料時，總封包長度為 6 + dataLen Byte
    }
    MOTOR_SERIAL->write(_buffer, totalLen);
}



//(3)讀取馬達狀態2
void LKMotor::Read_Motor_Status2(){
  sendFrame(0x9C,0x00,nullptr);
    // Read Package from motor
  count_rx = 0;
  RXWaitingStartTime = micros();
  while ((micros() - RXWaitingStartTime) < RXWaitTime)
  {
    Unpack();
  }
}   

//(8)設置馬達零點
void LKMotor::Set_Motor_Origin(){
  sendFrame(0x19,0x00,nullptr);
}

//(9)讀取多圈角度命令
void LKMotor::Read_Angle_MultiRound(){
  sendFrame(0x92,0x00,nullptr);
  // Read Package from motor
  count_rx = 0;
  RXWaitingStartTime = micros();
  while ((micros() - RXWaitingStartTime) < RXWaitTime)
  {
    Unpack();
  }
}

//(11)速度閉環控制命令
void LKMotor::Write_angularvel_MultiRound(double angularvel){

  int32_t angular_velocity = (int32_t)(angularvel * 100 * _reduction_ratio);   //單位換算與乘上齒輪比

  uint8_t data[4];

  for(int i=0;i<4;i++)
    data[i]=(angular_velocity>>(8*i))&0xFF;

  sendFrame(0xA2,0x04,data);
  
  // Read Package from motor
  count_rx = 0;
  RXWaitingStartTime = micros();
  while ((micros() - RXWaitingStartTime) < RXWaitTime)
  {
    Unpack();
  }
}

//(15)電機關機命令
void LKMotor::Write_Motor_Disable(){
  sendFrame(0x80,0x00,nullptr);
}

//(17)電機運行命令
void LKMotor::Write_Motor_Enable(){
  sendFrame(0x88,0x00,nullptr);
}

//(21)多圈位置閉環控制命令1
void LKMotor::Write_Angle_MultiRound(double angle){

  int64_t angleControl = (int64_t)(angle * 100 * _reduction_ratio);   //單位換算與乘上齒輪比
  
  uint8_t data[8];

  for(int i=0;i<8;i++)
    data[i]=(angleControl>>(8*i))&0xFF;

  sendFrame(0xA3,0x08,data);
}

//(22)多圈位置閉環控制命令2
void LKMotor::Write_Angle_MultiRound(double angle, long max_speed){
  if (max_speed <= 1)
  {
    max_speed = 1;
  }

  int64_t angleControl = (int64_t)(angle * 100 * _reduction_ratio);   //單位換算與乘上齒輪比
  uint32_t maxSpeed = (uint32_t)(max_speed * 100 * _reduction_ratio); //單位換算與乘上齒輪比
  
  uint8_t data[12];

  for(int i=0;i<8;i++)  //角度
    data[i]=(angleControl>>(8*i))&0xFF;
  
  for(int i=0;i<4;i++)  //最大速度
    data[i+8]=(maxSpeed>>(8*i))&0xFF;

  sendFrame(0xA4,0x0C,data);

  // Read Package from motor
  count_rx = 0;
  RXWaitingStartTime = micros();
  while ((micros() - RXWaitingStartTime) < RXWaitTime)
  {
    Unpack();
  }
}

//(24) 設多圈原點
void LKMotor::Set_Motor_MultiOrigin(){
  uint8_t data[7];

  for(int i=0;i<7;i++)  
    data[i]=(0x00>>(8*i));
  sendFrame(0x95,0x07,data);
}

// 接收封包
void LKMotor::Unpack(){
    byte temp;

    while (MOTOR_SERIAL->available())
    {
        MOTOR_SERIAL->readBytes(&temp,1);

        if (temp == 0x3E && count_rx == 0)
        {
            readin[count_rx++] = temp;
        }
        else if (count_rx >= 1)
        {
            readin[count_rx++] = temp;

            // 收到完整封包
            if (count_rx == readin[3] + 5)
            {
                parseFrame();      // <<<<<< 只呼叫解析

                count_rx = 0;
                return;
            }

            if (count_rx > 20)
            {
                count_rx = 0;
                return;
            }
        }
    }
}
// 判斷封包命令
void LKMotor::parseFrame(){
    switch(readin[1])
    {
        case 0x92:parseAngle();break;
        case 0x9C:parseStatus2();break;
        case 0xA2:parseStatus2();break;
        case 0xA4:parseStatus2();break;
    }
}
// (9)讀取多圈角度命令解包
void LKMotor::parseAngle(){
    int motor_id = readin[2];
    int64_t motorAngle =
          ((int64_t)readin[12]<<56)
        | ((int64_t)readin[11]<<48)
        | ((int64_t)readin[10]<<40)
        | ((int64_t)readin[9]<<32)
        | ((int64_t)readin[8]<<24)
        | ((int64_t)readin[7]<<16)
        | ((int64_t)readin[6]<<8)
        | ((int64_t)readin[5]);

    if (motor_id != _id)
        return;

    motor_angle = (double)motorAngle /
                  (100.0 * _reduction_ratio);
}
// (3)讀取馬達狀態2命令解包
void LKMotor::parseStatus2(){
    int motor_id = readin[2];

    if (motor_id != _id)
        return;

    int16_t current =
        ((int16_t)readin[7]<<8) |
        readin[6];

    motor_current =
        current * 66.0 / 4096.0;

    motor_temperature = readin[5];

    int16_t speed =
    (int16_t)(((uint16_t)readin[9] << 8) | readin[8]);

    motor_dspeed = (double)speed / _reduction_ratio;
    // Serial.println("在");
}

