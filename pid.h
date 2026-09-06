#pragma once
#include <Arduino.h>

// 標準 PID + 微分項低通濾波 (filtered derivative)，
// 微分項是對「量測值」而非「誤差」做微分 (derivative-on-measurement)，
// 可避免 setpoint(target) 突然改變時產生微分項暴衝(derivative kick)。
class PID {
  public:
    PID(double kp, double ki, double kd);

    void setpid(double kp, double ki, double kd);

    // currentpos: 呼叫 compute() 前，先用這個初始化狀態(避免第一次呼叫時微分項暴衝)
    void init(double currentpos);

    // 輸出限制；若不呼叫，預設為 ±1000 (見下方 Outmax/Outmin)，
    // 請務必依實際安全範圍設定，不要依賴預設值。
    void setOutputLimits(double outMin, double outMax);

    // target: 目標值   currentpos: 目前量測值   回傳: 控制輸出
    double compute(double target, double currentpos);

  private:
    double _kp, _ki, _kd;
    double _error;
    unsigned long _lasttime;
    double _integral;
    double _pv, _lastpv;
    double _lastD;
    double _lastOutput;

    double Outmax = 1000;   // 預設輸出上限；務必用 setOutputLimits() 覆寫成安全值
    double Outmin = -1000;  // 預設輸出下限；務必用 setOutputLimits() 覆寫成安全值

    const double Imax = 200;   // 積分項限幅上限，之後要依實測再調整
    const double Imin = -200;  // 積分項限幅下限，之後要依實測再調整
    const double _tau_f = 0.01; // 微分項低通濾波時間常數 (s)
};