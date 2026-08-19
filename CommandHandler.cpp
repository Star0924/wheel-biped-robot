// CommandHandler.cpp
#include "CommandHandler.h"

void CommandHandler::poll(RobotState &state, JointController &joints, BalanceController &balance) {
    if (Serial.available() <= 0) return;
    char cmd = Serial.read();

    switch (cmd) {
        case 'e': case 'E':
            if (state != RobotState::JOINT_LOCKED) {
                Serial.println(">>> 關節尚未自鎖，先執行 l 鎖定關節");
            } else {
                balance.enable();
                state = RobotState::BALANCING;
            }
            break;

        case 'd': case 'D':
            balance.disable();
            state = joints.isLocked() ? RobotState::JOINT_LOCKED : RobotState::JOINT_UNLOCKED;
            break;

        case 'l': case 'L':
            joints.lockAll();
            state = RobotState::JOINT_LOCKED;
            break;

        case 'u': case 'U':
            if (balance.isEnabled()) balance.disable();
            joints.unlockAll();
            state = RobotState::JOINT_UNLOCKED;
            break;

        case 'z': case 'Z':
            balance.zeroYaw();
            Serial.println(">>> 偏航角歸零");
            break;

        case 'x': case 'X':
            balance.zeroXY();
            Serial.println(">>> XY 軸角度歸零");
            break;

        case 'c': case 'C':
            Serial.println(">>> 加速度校驗中，請保持靜止 4 秒");
            balance.calibrateAcc();
            Serial.println(">>> 校驗完成");
            break;


        default:
            break;
    }
}