#include "BalanceController.h"

void BalanceController::begin() {
    _wheelLeft.Serial_Init();
    _wheelRight.Serial_Init();
    _imu.begin(IMU_SERIAL, IMU_BAUD);
    _pid.init(0.0);
}

void BalanceController::enable() {
    _wheelLeft.Write_Motor_Enable();
    _wheelRight.Write_Motor_Enable();
    _pid.init(0.0);
    _enabled = true;
    _filteredPitch = _imu.getData().angle[1];
    Serial.println(">>> 輪子已啟動，開始平衡");
}

void BalanceController::disable() {
    _wheelLeft.Write_Motor_Disable();
    _wheelRight.Write_Motor_Disable();
    _enabled = false;
    Serial.println(">>> 輪子已關閉");
}

void BalanceController::update(RobotState &state) {
    _imu.update();

    uint32_t now = millis();
    if (now - _lastControlMs < CONTROL_PERIOD_MS) return;
    _lastControlMs = now;

    double pitch = _imu.getData().angle[1];

    _filteredPitch = (_alpha * pitch) + ((1.0 - _alpha) * _filteredPitch);

    if (fabs(_filteredPitch) > FALL_LIMIT_DEG) {
        if (_enabled) {
            disable();
            state = RobotState::FALLEN;
            Serial.println("!!! 傾角過大，自動關閉輪子 !!!");
        }
        return;
    }

    if (_enabled) {
        // PID 計算改用濾波後的平滑角度
        double out = _pid.compute(TARGET_ANGLE_DEG, _filteredPitch);
        _wheelLeft.Write_angularvel_MultiRound(WHEEL_LEFT_SIGN * out);
        _wheelRight.Write_angularvel_MultiRound(WHEEL_RIGHT_SIGN * out);
    }
}