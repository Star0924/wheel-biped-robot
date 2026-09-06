#include "pid.h"

PID::PID(double kp, double ki, double kd) {
  setpid(kp, ki, kd);
  _error = 0;
  _lasttime = micros();
  _integral = 0;
  _pv = 0;
  _lastpv = 0;
  _lastD = 0;
  _lastOutput = 0;
}

void PID::setOutputLimits(double outMin, double outMax) {
  Outmin = outMin;
  Outmax = outMax;
}

void PID::setpid(double kp, double ki, double kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
}

void PID::init(double currentpos) {
  _error = 0;
  _lasttime = micros();
  _integral = 0;
  _pv = 0;
  _lastpv = currentpos;   // 讓第一次 compute() 的微分項不會暴衝
  _lastD = 0;
  _lastOutput = 0;
}

double PID::compute(double target, double currentpos) {
  unsigned long now = micros();
  double Ts = (double)(now - _lasttime) * 1e-6;   // micros() 溢位時，unsigned 減法仍會得到正確的經過時間
  if (Ts <= 0) return _lastOutput;

  double error = target - currentpos;

  // ---- P ----
  double P = _kp * error;

  // ---- I (含限幅，避免積分飽和/windup) ----
  _integral += _ki * error * Ts;
  if (_integral > Imax) _integral = Imax;
  else if (_integral < Imin) _integral = Imin;
  double I = _integral;

  // ---- D (對量測值微分，再做低通濾波去除高頻雜訊放大) ----
  _pv = currentpos;
  double rawDerivative = _kd * -(_pv - _lastpv) / Ts;
  double alpha = Ts / (_tau_f + Ts);
  double D = alpha * rawDerivative + (1.0 - alpha) * _lastD;

  // ---- 合成輸出並限幅 ----
  double output = P + I + D;
  if (output > Outmax) output = Outmax;
  else if (output < Outmin) output = Outmin;

  _lasttime = now;
  _lastpv = _pv;
  _lastD = D;
  _lastOutput = output;

  return output;
}