#pragma once

#include <Arduino.h>
#include "HWT906.h"
#include "WBRKinematics.h"
#include "pid.h"
#include "LKMotor.h"
#include "filter.h"

// ================= 硬體與常數設定 =================
#define IMU_SERIAL   Serial2
#define IMU_BAUD     921600

#define WHEEL_LEFT_SIGN   (-1.0)
#define WHEEL_RIGHT_SIGN  (+1.0)

const int JOINT_COUNT         = 4;
const long JOINT_LOCK_SPEED   = 20;   // 自鎖時的移動速度上限(deg/s)
const double TARGET_ANGLE     = 0.0;  
const double FALL_LIMIT_DEG   = 40.0; // 傾倒保護角


// ================= 全域物件與變數宣告 (extern) =================
extern LKMotor wheelLeft;
extern LKMotor wheelRight;
extern LKMotor hipLeft;
extern LKMotor kneeLeft;
extern LKMotor hipRight;
extern LKMotor kneeRight;

extern LKMotor* jointMotors[4];
extern const char* jointNames[4];

extern HWT906 imu;
extern PID CurrentPID;
extern PID balancePID;
extern PID velPID;
extern KalmanFilter kalmanPitch;
extern LowPassFilter lowPassPitch;
extern KalmanFilter speedFilterLeft;
extern KalmanFilter speedFilterRight;

extern double finalFilteredPitch;
extern bool wheelsEnabled;
extern bool jointsLocked;
extern double Avgspeed;
extern double motorOutput;
extern double torqueOutput;
extern double targetangle;
