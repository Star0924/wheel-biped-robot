#include "config.h"

// ================= 馬達物件實例化 =================
LKMotor wheelLeft (4, 8,  5, WHEEL_MAX_CURRENT_A);   // 左輪馬達
LKMotor wheelRight(1, 8,  5, WHEEL_MAX_CURRENT_A);   // 右輪馬達

LKMotor hipLeft   (6, 10, 5);   // 左髖馬達
LKMotor kneeLeft  (5, 10, 5);   // 左膝馬達
LKMotor hipRight  (3, 10, 5);   // 右髖馬達
LKMotor kneeRight (2, 10, 5);   // 右膝馬達

LKMotor* jointMotors[4]  = { &hipLeft, &kneeLeft, &hipRight, &kneeRight };
const char* jointNames[4] = { "左髖", "左膝", "右髖", "右膝" };

// 建立imu class物件
HWT906 imu;

// PID CurrentPID(0.1, 0.0, 0.0);
PID balancePID(41.1, 0.01, 0.15);
PID velPID(0.0084, 0.0, 0.0);

// 俯仰角估測器 (MPC 模式使用)
PitchEstimator pitchEstimator(PITCH_FUSE_ALPHA);

// Kalman + 低通濾波器 (PID 模式使用)
KalmanFilter kalmanPitch{1.0, 1.0, 0.05};
LowPassFilter lowPassPitch(0.3);

// 輪速濾波器
LowPassFilter speedFilterLeft(0.5);
LowPassFilter speedFilterRight(0.5);

// ================= 狀態變數初始化 =================
double filteredPitch = 0.0;
double filteredPitchRate = 0.0;
bool wheelsEnabled = false;
bool jointsLocked  = false;
double Avgspeed = 0.0;
double motorOutput = 0.0;
double torqueOutput = 0.0;
double targetangle = 0.0;

// 建立控制模式變數
ControlMode controlMode = MODE_MPC;  // 預設控制模式：MPC