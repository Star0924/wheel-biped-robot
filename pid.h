#pragma once
#include <Arduino.h>

class PID{
  private:
    double _kp, _ki, _kd;
    double _error;
    unsigned long _lasttime;
    double _integral;
    double _pv, _lastpv;
    double _lastD;
    double _lastOutput;
    double Outmax = 1000;
    double Outmin = -1000;
    const double Imax = 200; //之後要調整
    const double Imin = -200; //之後要調整
    const double _tau_f = 0.01;

  public:
    PID(double kp, double ki, double kd);
    void setpid(double kp, double ki, double kd);
    void init(double currentpos);
    void setOutputLimits(double outMin, double outMax);
    double compute(double target, double currentpos);
};