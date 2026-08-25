#pragma once
#include "config.h" // 需要引入設定才能控制馬達與讀取狀態

void lockJoints();
void unlockJoints();
void enableWheels();
void disableWheels();
void handleSerialCommand();
void PrintMotorStatus(LKMotor &motor, const char *name);

