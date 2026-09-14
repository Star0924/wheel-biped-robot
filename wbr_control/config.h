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

const int JOINT_COUNT         = 4;
const long JOINT_LOCK_SPEED   = 20;   // 自鎖時的移動速度上限(deg/s)
const double TARGET_ANGLE     = 0.0;
const double FALL_LIMIT_DEG   = 40.0; // 傾倒保護角

// ================= 輪馬達安全電流上限 =================
// 依馬達規格書 MG6012E-i8 V2：Max Current = 8A (momentary)，Nominal Current = 4A (continuous)。
// LKMotor::Write_Torque_MultiRound() 的協定欄位滿刻度是 33A(見 LKMotor.h)，
// 這個 33A 只是「數值編碼」用的滿刻度，跟這顆馬達實際能承受的電流無關；
// 真正的安全上限一定要用馬達規格書的數字，也就是這裡的 WHEEL_MAX_CURRENT_A。
// 建構 wheelLeft / wheelRight 時會把這個值傳進去，之後不管上層 PID/MPC
// 算出多大的扭矩指令，LKMotor 內部都會自動夾限，不會真的送出超過此值的電流。
//
// 這裡先取比 Max Current(8A) 略保守一點的值，保留安全餘裕；
// 若之後要壓榨到滿載扭矩，務必先確認散熱/供電沒問題再往上調，且不建議超過8A。
const double WHEEL_MAX_CURRENT_A = 7.0;

// ================= 扭矩死區補償 =================
// 保守估計：實測發現扭矩指令太小時，會因為馬達/齒輪箱的靜摩擦而完全不轉，
// 導致MPC以為有出力、實際上輪子沒動。這裡設一個下限：
// 只要MPC「想要出力」（指令非零），就把量值墊到至少這個值，
// 但不會疊加在原本已經夠大的指令上（避免正常指令被過度放大）。
// 0.3Nm 是先取一個保守值；之後可以實測輪子「剛好開始轉」的臨界扭矩，
// 再回頭微調這個數字。
const double DEADZONE_TORQUE_NM = 0.1;

// ================= 控制模式 =================
enum ControlMode { MODE_PID, MODE_MPC };
extern ControlMode controlMode;

// ================= MPC (PC端運算) 相關設定 =================
// 超過這麼久沒收到PC送來的新扭矩指令，視為斷線，自動關閉輪子
const unsigned long MPC_TIMEOUT_MS = 100;

// TODO !! 請務必依馬達實際規格 "實測" 校正扭矩常數，不要只信任規格書標稱值 !!
// Write_Torque_MultiRound() 收的是電流(A)，MPC 算出來的是扭矩(N*m)，
// 換算式為: current_A = torque_Nm / MOTOR_TORQUE_CONSTANT
// 目前這個值等於規格書上的 Torque Constant (1.6 Nm/A)，可以先用，
// 但正式上線前建議用測力計實測校正一次(規格書標稱值通常有±5~10%誤差)。
const double MOTOR_TORQUE_CONSTANT = 1.6;

// ================= 全域物件與變數宣告 (extern) =================
// 告訴編譯器：這些變數存在，但實體定義在 config.cpp 中
extern LKMotor wheelLeft;
extern LKMotor wheelRight;
extern LKMotor hipLeft;
extern LKMotor kneeLeft;
extern LKMotor hipRight;
extern LKMotor kneeRight;

extern LKMotor* jointMotors[4];
extern const char* jointNames[4];

extern HWT906 imu;

// balancePID  : 內環，用「濾波後俯仰角」算出動力輪目標轉速(或MPC關閉時的板載輸出)
// velPID      : 外環，用「輪速」回授算出俯仰角補償量(讓機器人能前後移動/抗擾動)
// CurrentPID  : 目前程式碼「尚未接進控制迴圈」，屬於保留/未使用狀態，
//               若之後要做電流內環，記得在 loop() 裡實際呼叫 CurrentPID.compute()。
extern PID CurrentPID;
extern PID balancePID;
extern PID velPID;

extern KalmanFilter kalmanPitch;
extern LowPassFilter lowPassPitch;
extern MovingAverageFilter speedFilterLeft;
extern MovingAverageFilter speedFilterRight;

extern double filteredPitch;
extern double filteredPitchRate;
extern bool wheelsEnabled;
extern bool jointsLocked;
extern double Avgspeed;
extern double motorOutput;
extern double torqueOutput;   // 目前未被讀寫使用，保留給之後除錯/擴充用
extern double targetangle;