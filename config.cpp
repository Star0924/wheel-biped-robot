#include "config.h"

// ================= 馬達物件實例化 =================
// LKMotor(馬達ID, 減速比, Serial埠號)
LKMotor wheelLeft (4, 8,  5);   // 左輪馬達
LKMotor wheelRight(1, 8,  5);   // 右輪馬達
LKMotor hipLeft   (6, 10, 5);   // 左髖馬達
LKMotor kneeLeft  (5, 10, 5);   // 左膝馬達
LKMotor hipRight  (3, 10, 5);   // 右髖馬達
LKMotor kneeRight (2, 10, 5);   // 右膝馬達

LKMotor* jointMotors[4]  = { &hipLeft, &kneeLeft, &hipRight, &kneeRight };
const char* jointNames[4] = { "左髖", "左膝", "右髖", "右膝" };

// ================= 系統物件實例化 =================
HWT906 imu;
PID CurrentPID(0.1, 0.0, 0.0); // 用於電流控制的 PID
PID balancePID(42.0, 0.01, 0.15);   //42.0 0.01 0.15
PID velPID(0.018, 0.0015, 0.0); // 0.018 0.0015 0.0
KalmanFilter kalmanPitch{1.0, 1.0, 0.05}; // R=1.0, P=1.0, Q=0.05
KalmanFilter speedFilterLeft(10); 
KalmanFilter speedFilterRight(10);
LowPassFilter lowPassPitch(0.3); // 低通濾波器，alpha=0.3

// ================= 狀態變數初始化 =================
double finalFilteredPitch = 0.0;
bool wheelsEnabled = false;
bool jointsLocked  = false;
double Avgspeed = 0.0;
double motorOutput = 0.0;
double torqueOutput = 0.0;
double targetangle = 0.0;
