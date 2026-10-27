#include "MPCLink.h"
#include <string.h>
#include <stdlib.h>

MPCLink mpcLink;

void MPCLink::begin(unsigned long baud) {
  SerialUSB1.begin(baud);
  _rxLen = 0;
}

void MPCLink::sendState(double pitch_deg, double pitchRate_dps,
                        double vL_dps, double vR_dps,
                        double yaw_deg, double yawRate_dps, uint32_t timestamp_ms,
                        const MotorTelemetry* motor) {
  SerialUSB1.print('S'); SerialUSB1.print(',');
  SerialUSB1.print(timestamp_ms); SerialUSB1.print(',');
  SerialUSB1.print(pitch_deg, 4);     SerialUSB1.print(',');
  SerialUSB1.print(pitchRate_dps, 4); SerialUSB1.print(',');
  SerialUSB1.print(vL_dps, 4);        SerialUSB1.print(',');
  SerialUSB1.print(vR_dps, 4);        SerialUSB1.print(',');
  SerialUSB1.print(yaw_deg, 4);       SerialUSB1.print(',');
  if (motor == nullptr) {
    SerialUSB1.println(yawRate_dps, 4);
    return;
  }
  SerialUSB1.print(yawRate_dps, 4);      SerialUSB1.print(',');
  SerialUSB1.print(motor->iL_A, 3);      SerialUSB1.print(',');
  SerialUSB1.print(motor->iR_A, 3);      SerialUSB1.print(',');
  SerialUSB1.print(motor->cmdL_A, 3);    SerialUSB1.print(',');
  SerialUSB1.print(motor->cmdR_A, 3);    SerialUSB1.print(',');
  SerialUSB1.print(motor->wL_dps, 3);    SerialUSB1.print(',');
  SerialUSB1.print(motor->wR_dps, 3);    SerialUSB1.print(',');
  SerialUSB1.println((int)motor->fresh);
}

bool MPCLink::poll() {
  bool gotNew = false;

  while (SerialUSB1.available() > 0) {
    char c = SerialUSB1.read();

    if (c == '\n') {
      _rxBuf[_rxLen] = '\0';

      // 期望格式: U,<seq>,<u_l_Nm>,<u_r_Nm>
      if (_rxLen > 2 && _rxBuf[0] == 'U' && _rxBuf[1] == ',') {
        char* token = strtok(_rxBuf + 2, ",");   // 跳過 "U,"
        if (token != nullptr) {
          strtoul(token, nullptr, 10);            // seq，目前只用來除錯，先不特別處理
          char* lTok = strtok(nullptr, ",");
          char* rTok = strtok(nullptr, ",");
          if (lTok != nullptr && rTok != nullptr) {
            _lastTorqueL    = atof(lTok);
            _lastTorqueR    = atof(rTok);
            _lastRxMillis   = millis();
            gotNew = true;
          }
        }
      }
      // 期望格式: C,<seq>,<cmd>   (操作指令：e/d/l/u/z/x/c/6/9)
      else if (_rxLen > 3 && _rxBuf[0] == 'C' && _rxBuf[1] == ',') {
        char* seqTok = strtok(_rxBuf + 2, ",");
        char* cmdTok = strtok(nullptr, ",");
        if (seqTok != nullptr && cmdTok != nullptr && cmdTok[0] != '\0') {
          pushCommand(cmdTok[0]);
        }
      }
      _rxLen = 0;

    } else if (c != '\r') {
      if (_rxLen < sizeof(_rxBuf) - 1) {
        _rxBuf[_rxLen++] = c;
      } else {
        _rxLen = 0;   // 超長，異常資料，丟棄重來
      }
    }
  }

  return gotNew;
}

void MPCLink::sendAck(char cmd, int result) {
  SerialUSB1.print('A'); SerialUSB1.print(',');
  SerialUSB1.print(cmd); SerialUSB1.print(',');
  SerialUSB1.println(result);
}