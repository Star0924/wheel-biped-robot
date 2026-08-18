#include "HWT906_Parser.h"

HWT906_Parser::HWT906_Parser(){
  reset();
  for (int i = 0; i < 3; i++) {
      data.acc[i] = 0.0f;
      data.gyro[i] = 0.0f;
      data.angle[i] = 0.0f;
  }
  data.temperature = 0.0f;
}

void HWT906_Parser::reset(){
  state = State::WAIT_HEADER;
  count = 0;
}

bool HWT906_Parser::parseByte(uint8_t b){
  switch(state){
    case State::WAIT_HEADER:
      if(b == 0x55){
        buffer[0] = b;
        state = State::WAIT_TYPE;
      }
      break;
    
    case State::WAIT_TYPE:
      if(b >= (uint8_t)FrameType::ACC_F && b <= (uint8_t)FrameType::ANGLE_F){
        buffer[1] = b;
        count = 2;
        state = State::RECEIVE_DATA;
      }
      else {
        state = State::WAIT_HEADER;
      }
      break;
    
    case State::RECEIVE_DATA:
      buffer[count++] = b;
      if(count >= 11){
        state = State::WAIT_HEADER;
        if(verifyChecksum()){
          decode();
          return true;
        }
      }
      break;
  }
  return false;
}

bool HWT906_Parser::verifyChecksum() {
    uint8_t sum = 0;
    for (int i = 0; i < 10; ++i) {
        sum += buffer[i];
    }
    return (sum == buffer[10]);
}

void HWT906_Parser::decode() {
    uint8_t type = buffer[1];

    int16_t rawX = static_cast<int16_t>((buffer[3] << 8) | buffer[2]);
    int16_t rawY = static_cast<int16_t>((buffer[5] << 8) | buffer[4]);
    int16_t rawZ = static_cast<int16_t>((buffer[7] << 8) | buffer[6]);
    int16_t rawT = static_cast<int16_t>((buffer[9] << 8) | buffer[8]);

    if (type == static_cast<uint8_t>(FrameType::ACC_F)) {
        data.acc[0] = rawX * HWT906Scale::ACC;
        data.acc[1] = rawY * HWT906Scale::ACC;
        data.acc[2] = rawZ * HWT906Scale::ACC;
        data.temperature = rawT / 100.0f;
    } 
    else if (type == static_cast<uint8_t>(FrameType::GYRO_F)) {
        data.gyro[0] = rawX * HWT906Scale::GYRO;
        data.gyro[1] = rawY * HWT906Scale::GYRO;
        data.gyro[2] = rawZ * HWT906Scale::GYRO;
    } 
    else if (type == static_cast<uint8_t>(FrameType::ANGLE_F)) {
        data.angle[0] = rawX * HWT906Scale::ANGLE;
        data.angle[1] = rawY * HWT906Scale::ANGLE;
        data.angle[2] = rawZ * HWT906Scale::ANGLE;
    }
}