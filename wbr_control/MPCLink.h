#pragma once
#include <Arduino.h>

// ============================================================
// PC <-> Teensy 的 MPC 通訊橋接
//
// 使用 Teensy 第二組 USB 虛擬序列埠 (SerialUSB1)，跟原本用來
// 下 e/d/l/u 等人工指令的 Serial 完全分開，避免協定字元互相衝突。
//
// !! 使用前必須先在 Arduino IDE 設定 !!
//    Tools -> USB Type -> 選擇 "Dual Serial" 或 "Triple Serial"
//    這樣才會多出 SerialUSB1 這個埠可用。
//
// 通訊格式（皆為 ASCII，換行結尾）：
//   Teensy -> PC :  S,<millis>,<pitch_deg>,<pitchRate_dps>,<wheelSpeed_dps>\n
//   PC -> Teensy :  U,<seq>,<torque_Nm>\n
//
// 安全設計：poll() 只在收到「格式正確」的完整訊息時才更新 _lastRxMillis，
// 所以 isFresh() 反映的是「最後一次成功解析」的時間，格式錯誤/斷線都會
// 讓 isFresh() 判定逾時，讓主控迴圈自動關閉輪子。
// ============================================================

class MPCLink {
  public:
    void begin(unsigned long baud = 2000000);

    // 每個控制週期呼叫一次，把目前狀態送給 PC
    void sendState(double pitch_deg, double pitchRate_dps,
                   double wheelSpeed_dps, uint32_t timestamp_ms);

    // 建議每次 loop() 都呼叫，非阻塞地解析PC送回的扭矩指令
    // 有收到新的完整指令時回傳 true
    bool poll();

    double lastTorqueCmd() const { return _lastTorque; }
    uint32_t lastRxMillis() const { return _lastRxMillis; }

    // 距離上次收到PC指令是否還在 timeoutMs 之內（用來判斷斷線）
    bool isFresh(uint32_t timeoutMs) const {
      return (millis() - _lastRxMillis) < timeoutMs;
    }

    // 啟動輪子/切換到MPC模式時呼叫，避免PC還沒開始送資料就被誤判逾時
    void resetWatchdog() { _lastRxMillis = millis(); }

  private:
    double   _lastTorque   = 0.0;
    uint32_t _lastRxMillis = 0;
    char     _rxBuf[64];
    uint8_t  _rxLen = 0;
};

extern MPCLink mpcLink;