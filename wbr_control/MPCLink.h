#pragma once
#include <Arduino.h>

// ============================================================
// PC <-> Teensy 的 MPC 通訊橋接
//
// 使用 Teensy 第二組 USB 虛擬序列埠 (SerialUSB1)，跟原本用來
// 下 e/d/l/u 等人工指令的 Serial 完全分開，避免協定字元互相衝突。
//
// 通訊格式（皆為 ASCII，換行結尾）：
//   Teensy -> PC :  S,<millis>,<pitch_deg>,<pitchRate_dps>,<vL>,<vR>,<yaw_deg>,<yawRate_dps>
//                   [,<iL_A>,<iR_A>,<cmdL_A>,<cmdR_A>,<wL_dps>,<wR_dps>,<fresh>]\n  (後 7 欄為馬達遙測，可省略)
//   PC -> Teensy :  U,<seq>,<u_l_Nm>,<u_r_Nm>\n   (左輪、右輪扭矩，前進為正)
//   PC -> Teensy :  C,<seq>,<cmd>\n                (操作指令，全部由 PC 發起；<cmd> 單一字元)
//                     e 啟動輪子  d 關閉輪子  l 關節自鎖  u 關節解鎖
//                     z 偏航歸零  x XY歸零  c 加速度校正  6 六軸  9 九軸
//   Teensy -> PC :  A,<cmd>,<result>\n             (指令回覆：1=已執行 0=被拒絕；
//                                                    cmd='F' 傾倒保護觸發、'T' PC逾時，皆代表輪子已被 Teensy 關閉)
// ============================================================

// 馬達遙測：附在 S 封包後面，PC 端拿來記錄/畫圖。全部是「前進為正」的座標。
struct MotorTelemetry {
  double iL_A = 0, iR_A = 0;        // 回授轉矩電流 (A)
  double cmdL_A = 0, cmdR_A = 0;    // 最近一次實際送出的電流指令 (A，已夾限)
  double wL_dps = 0, wR_dps = 0;    // 未濾波輪速 (deg/s)
  uint8_t fresh = 0;                // bit0=左輪、bit1=右輪：自上一個 S 封包起有收到新的狀態回覆
};

class MPCLink {
  public:
    void begin(unsigned long baud = 2000000);

    // 每個控制週期呼叫一次，把目前狀態送給 PC
    void sendState(double pitch_deg, double pitchRate_dps,
                  double vL_dps, double vR_dps,
                  double yaw_deg, double yawRate_dps, uint32_t timestamp_ms,
                  const MotorTelemetry* motor = nullptr);   // motor 給 nullptr = 舊的 8 欄格式
    double lastTorqueR() const { return _lastTorqueR; }

    // 有收到新的完整指令時回傳 true
    bool poll();

    double lastTorqueL() const { return _lastTorqueL; }
    uint32_t lastRxMillis() const { return _lastRxMillis; }

    // 距離上次收到PC指令是否還在 timeoutMs 之內（用來判斷斷線）
    bool isFresh(uint32_t timeoutMs) const {
      return (millis() - _lastRxMillis) < timeoutMs;
    }

    // 啟動輪子/切換到MPC模式時呼叫，避免PC還沒開始送資料就被誤判逾時
    void resetWatchdog() { _lastRxMillis = millis(); }

    // 清掉上一次的扭矩指令，避免重新啟動時殘留舊值
    void clearTorqueCmd() { _lastTorqueL = 0.0; _lastTorqueR = 0.0; }

    // ---- 操作指令 (C 封包) ----
    // 有指令時回傳 true 並交出一個字元 (內部是小型佇列，同一次 poll 收到多個也不會掉)
    bool takeCommand(char& cmd) {
      if (_cmdHead == _cmdTail) return false;
      cmd = _cmdQueue[_cmdTail];
      _cmdTail = (_cmdTail + 1) % CMD_QUEUE_SIZE;
      return true;
    }

    // 回覆 PC：A,<cmd>,<result>
    void sendAck(char cmd, int result);

  private:
    double   _lastTorqueL  = 0.0;   // 左輪扭矩指令 (Nm，前進為正)
    double   _lastTorqueR  = 0.0;   // 右輪扭矩指令 (Nm，前進為正)
    uint32_t _lastRxMillis = 0;
    char     _rxBuf[64];
    uint8_t  _rxLen = 0;

    static const uint8_t CMD_QUEUE_SIZE = 8;
    char     _cmdQueue[CMD_QUEUE_SIZE];
    uint8_t  _cmdHead = 0, _cmdTail = 0;
    void pushCommand(char c) {
      uint8_t next = (_cmdHead + 1) % CMD_QUEUE_SIZE;
      if (next == _cmdTail) return;      // 佇列滿了就丟棄
      _cmdQueue[_cmdHead] = c;
      _cmdHead = next;
    }
};

extern MPCLink mpcLink;