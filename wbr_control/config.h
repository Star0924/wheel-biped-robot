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

// 【新增】陀螺儀 pitch 軸相對於角度 pitch 軸的符號。
// 校正方法：把 printDebugInfo() 打開，用手「緩慢地」把車體往前傾，
// 觀察 raw(角度) 是往正還往負走，再看 gyroY 的符號是否同向。
// 同向填 +1.0，反向填 -1.0。填錯會讓 PitchEstimator 變成正回授，一定要先確認。
const double GYRO_PITCH_SIGN = +1.0;

// 【新增】IMU 角度修正權重，越小越信任陀螺儀(延遲更低、漂移修正更慢)
const double PITCH_FUSE_ALPHA = 0.02;

const int JOINT_COUNT         = 4;
const long JOINT_LOCK_SPEED   = 20;   // 自鎖時的移動速度上限(deg/s)
const double TARGET_ANGLE     = 0.0;
const double FALL_LIMIT_DEG   = 40.0; // 傾倒保護角

// ================= 輪馬達安全電流上限 =================
// 依馬達規格書 MG6012E-i8 V2：Max Current = 8A (momentary)，Nominal = 4A (continuous)。
const double WHEEL_MAX_CURRENT_A = 7.0;

// ================= 靜摩擦前饋補償 (取代原本的「死區墊高」) =================
// 【為什麼要改】
// 原本的 applyTorqueDeadzone() 是：只要指令非零、但量值小於 deadzone，
// 就直接把它「墊高」到 ±deadzone。實測 log 顯示，單輪指令(u0/2)有 58.7% 的
// 時間落在左輪死區(0.302Nm)以內，也就是說超過一半的時間，
// 左輪收到的其實是固定大小 ±0.302Nm 的方波，跟 MPC 算出來的量值完全無關。
// 在平衡點附近，控制器等於失去了「比例控制權」，只剩下 bang-bang，
// 必然產生極限環。log 裡扭矩指令每秒變號 4.5 次、頻譜在 4.76Hz 有明顯尖峰，
// 就是這個機制造成的抖動。
// 另外左右死區不同(0.302 / 0.235)，兩輪同時被墊高時會出現約 0.067Nm 的
// 差動扭矩，會讓車體慢慢偏航。
//
// 【改成什麼】
// 摩擦補償應該跟「輪子的運動方向」走，而不是跟「指令的符號」走：
//   tau_out = u + F_c * dir
// 其中 dir 在輪子有在轉時取輪速方向，快停下來時才平滑地改用指令方向，
// 用 tanh() 連續過渡，不會在零點產生跳變。
// 這樣小指令還是小輸出(保有比例控制權)，但克服靜摩擦所需的那一份由前饋補上。
//
// FRICTION_*_NM：實測「輪子剛好開始轉」的臨界扭矩。
// 先從原本量到的 0.503 / 0.391 打 7 折開始；如果還是會自己緩慢滑動，就再往下調。
const double FRICTION_LEFT_NM   = 0.503 * 0.7;
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
// 控制週期(ms)。目前 PC 端 solve 的 p95 約 10ms，15ms 是安全的；
// 若之後把 N 再壓小、solve p95 < 6ms，可以考慮降到 10ms 提高頻寬。
const unsigned long CONTROL_PERIOD_MS = 15;

// TODO !! 請務必依馬達實際規格 "實測" 校正扭矩常數 !!
// 用 log 做的參數辨識顯示：實際 domega/dt 對 u 的靈敏度只有模型預測的
// 約 54~74%，代表「MPC 以為送出去 1Nm，車體實際只吃到約 0.6~0.7Nm」。
// 可能來源：扭矩常數標稱值偏樂觀、減速箱效率、電流環追不上。
// 在沒有實測校正前，建議在 PC 端 RobotParams.torque_gain 填 0.7 讓模型貼近現實；
// 等你用測力計/擺錘實測出真值後，再把這裡改對、把 torque_gain 調回 1.0。
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

extern PID CurrentPID;
extern PID balancePID;
extern PID velPID;

// 【新增】取代 kalmanPitch + lowPassPitch 的低延遲俯仰角估測器
extern PitchEstimator pitchEstimator;

extern KalmanFilter kalmanPitch;       // 保留，PID 模式仍可用
extern LowPassFilter lowPassPitch;     // 保留
extern MovingAverageFilter speedFilterLeft;
extern MovingAverageFilter speedFilterRight;

extern double filteredPitch;
extern double filteredPitchRate;
extern bool wheelsEnabled;
extern bool jointsLocked;
extern double Avgspeed;
extern double motorOutput;
extern double torqueOutput;
extern double targetangle;