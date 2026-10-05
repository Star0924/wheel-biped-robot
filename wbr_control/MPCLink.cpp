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
                        double yaw_deg, double yawRate_dps, uint32_t timestamp_ms) {
  SerialUSB1.print('S'); SerialUSB1.print(',');
  SerialUSB1.print(timestamp_ms); SerialUSB1.print(',');
  SerialUSB1.print(pitch_deg, 4);     SerialUSB1.print(',');
  SerialUSB1.print(pitchRate_dps, 4); SerialUSB1.print(',');
  SerialUSB1.print(vL_dps, 4);        SerialUSB1.print(',');
  SerialUSB1.print(vR_dps, 4);        SerialUSB1.print(',');
  SerialUSB1.print(yaw_deg, 4);       SerialUSB1.print(',');
  SerialUSB1.println(yawRate_dps, 4);
}

bool MPCLink::poll() {
  bool gotNew = false;

  while (SerialUSB1.available() > 0) {
    char c = SerialUSB1.read();

    if (c == '\n') {
      _rxBuf[_rxLen] = '\0';

      // 期望格式: U,<seq>,<torque_Nm>
      if (_rxLen > 2 && _rxBuf[0] == 'U' && _rxBuf[1] == ',') {
        char* token = strtok(_rxBuf + 2, ",");   // 跳過 "U,"
        if (token != nullptr) {
          strtoul(token, nullptr, 10);            // seq，目前只用來除錯，先不特別處理
          char* torqueTok = strtok(nullptr, ",");
          char* diffTok   = strtok(nullptr, ",");
          if (torqueTok != nullptr && diffTok != nullptr) {
            _lastTorque     = atof(torqueTok);
            _lastTorqueDiff = atof(diffTok);
            _lastRxMillis   = millis();
            gotNew = true;
          }
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