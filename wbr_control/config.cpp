#include "config.h"

// ================= 馬達物件實例化 =================
// LKMotor(馬達ID, 減速比, Serial埠號, 安全電流上限A)
// 輪馬達會實際用到 Write_Torque_MultiRound()，務必帶入 WHEEL_MAX_CURRENT_A，
// 讓 LKMotor 內部自動夾限電流，避免任何情況下超過馬達規格書的 Max Current。
LKMotor wheelLeft (4, 8,  5, WHEEL_MAX_CURRENT_A);   // 左輪馬達
LKMotor wheelRight(1, 8,  5, WHEEL_MAX_CURRENT_A);   // 右輪馬達

// 關節馬達目前只用位置/使能指令，不會呼叫 Write_Torque_MultiRound()，
// 保留預設安全上限(等同不設限)即可。
LKMotor hipLeft   (6, 10, 5);   // 左髖馬達
LKMotor kneeLeft  (5, 10, 5);   // 左膝馬達
LKMotor hipRight  (3, 10, 5);   // 右髖馬達
LKMotor kneeRight (2, 10, 5);   // 右膝馬達

LKMotor* jointMotors[4]  = { &hipLeft, &kneeLeft, &hipRight, &kneeRight };
const char* jointNames[4] = { "左髖", "左膝", "右髖", "右膝" };

// ================= 系統物件實例化 =================
HWT906 imu;

PID CurrentPID(0.1, 0.0, 0.0);      // 目前未接進控制迴圈，見 config.h 註解
PID balancePID(41.1, 0.01, 0.15);
PID velPID(0.0084, 0.0, 0.0);

KalmanFilter kalmanPitch{1.0, 1.0, 0.05}; // R=1.0, P=1.0, Q=0.05
MovingAverageFilter speedFilterLeft(10);
MovingAverageFilter speedFilterRight(10);
LowPassFilter lowPassPitch(0.3); // 低通濾波器，alpha=0.3

// ================= 狀態變數初始化 =================
double filteredPitch = 0.0;
double filteredPitchRate = 0.0;
bool wheelsEnabled = false;
bool jointsLocked  = false;
double Avgspeed = 0.0;
double motorOutput = 0.0;
double torqueOutput = 0.0;
double targetangle = 0.0;

// 預設開機用板載 PID，穩定後再用 m 指令切換到 MPC
ControlMode controlMode = MODE_PID;