#include "JointController.h"

void JointController::begin(){
  for (int i = 0; i < COUNT; i++) _motors[i].Serial_Init();
}

void JointController::lockAll() {
    Serial.println(">>> 執行關節自鎖...");
    for (int i = 0; i < COUNT; i++) {
        _motors[i].Write_Motor_Enable();
        delay(5);
        _motors[i].Read_Angle_MultiRound();
        double lockAngle = _motors[i].motor_angle;
        _motors[i].Write_Angle_MultiRound(lockAngle, JOINT_LOCK_SPEED);
        Serial.printf("  %s 自鎖於角度: %.2f\n", _names[i], lockAngle);
        delay(5);
    }
    _locked = true;
    Serial.println(">>> 關節自鎖完成");
}

void JointController::unlockAll() {
    for (int i = 0; i < COUNT; i++) _motors[i].Write_Motor_Disable();
    _locked = false;
    Serial.println(">>> 關節已解鎖");
}