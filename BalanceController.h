#pragma once
#include "HWT906.h"
#include "pid.h"
#include "LKMotor.h"
#include "RobotState.h"
#include "config.h"

class BalanceController {
public:
    void begin();
    void update(RobotState &state);   // 每次 loop 呼叫，內部自行做頻率控制
    void enable();
    void disable();
    bool isEnabled() const { return _enabled; }
    const IMUData& imuData() const { return _imu.getData(); }

    void zeroYaw()      { _imu.zeroYaw(); }
    void zeroXY()        { _imu.zeroXY(); }
    void calibrateAcc()  { _imu.calibrateAcc(); }

private:
    HWT906 _imu;
    PID _pid{BALANCE_KP, BALANCE_KI, BALANCE_KD};
    LKMotor _wheelLeft {WHEEL_LEFT_CFG.id,  WHEEL_LEFT_CFG.reduction,  WHEEL_LEFT_CFG.bus};
    LKMotor _wheelRight{WHEEL_RIGHT_CFG.id, WHEEL_RIGHT_CFG.reduction, WHEEL_RIGHT_CFG.bus};
    bool _enabled = false;
    uint32_t _lastControlMs = 0;
    double _filteredPitch = 0.0; 
    double _alpha = 0.3; // 濾波係數：根據實際震盪情況調整 (建議範圍 0.1 ~ 0.5)
};