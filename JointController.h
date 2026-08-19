#pragma once
#include "LKMotor.h"
#include "config.h"

class JointController{
  public:
    void begin();
    void lockAll();
    void unlockAll();
    bool isLocked() const {return _locked;}
  
  private:
    static const int COUNT = 4;
    LKMotor _motors[COUNT] = {
        LKMotor(HIP_LEFT_CFG.id,  HIP_LEFT_CFG.reduction,  HIP_LEFT_CFG.bus),
        LKMotor(KNEE_LEFT_CFG.id, KNEE_LEFT_CFG.reduction, KNEE_LEFT_CFG.bus),
        LKMotor(HIP_RIGHT_CFG.id, HIP_RIGHT_CFG.reduction, HIP_RIGHT_CFG.bus),
        LKMotor(KNEE_RIGHT_CFG.id,KNEE_RIGHT_CFG.reduction,KNEE_RIGHT_CFG.bus),
    };
    const char* _names[COUNT] = {"左髖", "左膝", "右髖", "右膝"};
    bool _locked = false;
};