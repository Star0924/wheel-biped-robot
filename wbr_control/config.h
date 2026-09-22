#pragma once

#include <Arduino.h>
#include "HWT906.h"
#include "WBRKinematics.h"
#include "pid.h"
#include "LKMotor.h"
#include "filter.h"
#include "MPCLink.h"

// ================= 硬體與常數設定 =================
#define IMU_SERIAL   Serial2
#define IMU_BAUD     921600

#define WHEEL_LEFT_SIGN   (-1.0)
#define WHEEL_RIGHT_SIGN  (+1.0)


const double GYRO_PITCH_SIGN = +1.0;    // 陀螺儀方向係數
const double PITCH_FUSE_ALPHA = 0.02;   // 仰角估測權重(0~1)，越小越信任陀螺儀
const int JOINT_COUNT         = 4;      // 左髖、左膝、右髖、右膝
const long JOINT_LOCK_SPEED   = 20;     // 自鎖時的移動速度上限(deg/s)
const double TARGET_ANGLE     = 0.0;    // 目標角度(deg)，MPC模式下由PC端算出，PID模式下由板載PID算出
const double FALL_LIMIT_DEG   = 30.0;   // 傾倒保護角
const double WHEEL_MAX_CURRENT_A = 7.5; // 輪胎馬達電流上限(A)

// ================= 左輪摩擦前饋設定 ================= 
const double FRICTION_LEFT_NM   = 0.401 * 0.7;
// ================= 右輪摩擦前饋設定 ================= 
const double FRICTION_RIGHT_NM  = 0.391 * 0.7;


// 輪速多少(deg/s)以上就完全用輪速方向判斷，以下則平滑過渡到指令方向
// [2nd] 6.0 -> 20.0：第二次 log 顯示輪速有 32% 的時間落在 ±6dps 內、每秒變號 3.6 次，
// 過窄的過渡帶讓摩擦前饋又帶了一點 relay 味道(在5.8Hz有0.30Nm的成分)。
// 輪速整體標準差 41dps、尖峰 229dps，過渡帶放寬到 20dps 仍然只佔很小一段。
const double FRICTION_SPEED_EPS_DPS = 20.0;
// 指令多小(Nm)以內視為「沒有明確方向」，避免零附近亂補償
const double FRICTION_CMD_EPS_NM    = 0.05;

// ================= 控制模式 =================
enum ControlMode { MODE_PID, MODE_MPC };
extern ControlMode controlMode;

// ================= MPC (PC端運算) 相關設定 =================
const unsigned long MPC_TIMEOUT_MS = 100;

const unsigned long CONTROL_PERIOD_MS = 15;

const double MOTOR_TORQUE_CONSTANT = 1.6;

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

// extern PID CurrentPID;
extern PID balancePID;
extern PID velPID;

extern PitchEstimator pitchEstimator;

extern KalmanFilter kalmanPitch;       // 保留，PID 模式仍可用
extern LowPassFilter lowPassPitch;     // 保留
extern LowPassFilter speedFilterLeft;
extern LowPassFilter speedFilterRight;

extern double filteredPitch;
extern double filteredPitchRate;
extern bool wheelsEnabled;
extern bool jointsLocked;
extern double Avgspeed;
extern double motorOutput;
extern double torqueOutput;
extern double targetangle;