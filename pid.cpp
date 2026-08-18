#include "pid.h"

PID::PID(double kp, double ki, double kd){
  PID::setpid(kp, ki, kd);
  _error = 0;
  _lasttime = micros();
  _integral = 0;
  _pv = 0;
  _lastpv = 0;
  _lastD = 0;
  _lastOutput = 0;
}

void PID::setpid(double kp, double ki, double kd){
  _kp = kp;
  _ki = ki;
  _kd = kd;
}

void PID::init(double currentpos){
  _error = 0;
  _lasttime = micros();
  _integral = 0;
  _pv = 0;
  _lastpv = currentpos;
  _lastD = 0;
  _lastOutput = 0;
}

double PID::compute(double target, double currentpos){
  unsigned long now = micros();
  double Ts = (double)(now - _lasttime) * 1e-6;
  if(Ts <= 0)return _lastOutput;

  double error = target - currentpos;

  double P = _kp * error;

  _integral += _ki * error * Ts;
  if(_integral > Imax)_integral = Imax;
  else if(_integral < Imin)_integral = Imin;
  double I = _integral;

  _pv = currentpos;
  double d = _kd * -(_pv - _lastpv) / Ts;

  double alpha = Ts / (_tau_f + Ts);
  double D = alpha * d + (1.0 - alpha) * _lastD;

  double output = P+I+D;

  if(output > Outmax) output = Outmax;
  else if(output < Outmin) output = Outmin;

  _lasttime = now;
  _lastpv = _pv;
  _lastD = D;
  _lastOutput = output;

  return output;
}