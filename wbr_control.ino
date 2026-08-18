#include <Arduino.h>
#include "HWT906.h"
#include "WBRKinematics.h"
#include "pid.h"
#include "LKMotor.h"

// ================================================================
// 6馬達串聯式雙輪足機器人 主控制程式
// 組成：2顆輪馬達（平衡驅動） + 4顆關節馬達（左右髖/膝，開機自鎖）
// 流程：開機 -> 4顆關節馬達自鎖(鎖定當下姿態) -> 等待指令 -> 啟動輪子平衡控制
// ================================================================

// ---------------- 硬體設定 ----------------
#define IMU_SERIAL   Serial2
#define IMU_BAUD     921600

// LKMotor(馬達ID, 減速比, Serial埠號)
// ↓↓↓ 請依實際出廠設定的馬達ID / 減速比修改，6顆馬達目前假設共用同一個 RS485 (Serial5) ↓↓↓
LKMotor wheelLeft (4, 8,  5);   // 左輪馬達 (MG6012E-i8)
LKMotor wheelRight(1, 8,  5);   // 右輪馬達 (MG6012E-i8)
LKMotor hipLeft   (6, 10, 5);   // 左髖馬達 (MG10015E-i10)
LKMotor kneeLeft  (5, 10, 5);   // 左膝馬達 (MG10015E-i10)
LKMotor hipRight  (3, 10, 5);   // 右髖馬達 (MG10015E-i10)
LKMotor kneeRight (2, 10, 5);   // 右膝馬達 (MG10015E-i10)

// 方便用陣列迴圈處理 4 顆關節馬達
LKMotor* jointMotors[4]      = { &hipLeft, &kneeLeft, &hipRight, &kneeRight };
const char* jointNames[4]    = { "左髖", "左膝", "右髖", "右膝" };
const int JOINT_COUNT        = 4;
const long JOINT_LOCK_SPEED  = 20; // 自鎖時的移動速度上限(deg/s)，鎖定當下角度時理論上不會真的移動

// 左右輪若為鏡像安裝，其中一輪的「正轉」方向會跟另一輪相反，實測後可調整
#define WHEEL_LEFT_SIGN   (+1.0)
#define WHEEL_RIGHT_SIGN  (-1.0)

// ---------------- IMU / PID ----------------
HWT906 imu;
PID balancePID(40.0, 0.0, 0.0);   // 初始值僅供起步測試，請依實際車體重新調參
const double TARGET_ANGLE = 0.0;  // 平衡目標角度(deg)，若車體重心偏移可微調此值

// 傾倒保護角
const double FALL_LIMIT_DEG = 40.0;

// ---------------- 狀態旗標 ----------------
bool wheelsEnabled  = false;
bool jointsLocked   = false;

// ---------------- 函式原型 ----------------
void lockJoints();
void unlockJoints();
void enableWheels();
void disableWheels();
void handleSerialCommand();
void PrintMotorStatus(LKMotor &motor, const char *name);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  // ---- 馬達初始化（6顆馬達共用 Serial5，各自仍需呼叫一次以設定內部指標）----
  wheelLeft.Serial_Init();
  wheelRight.Serial_Init();
  hipLeft.Serial_Init();
  kneeLeft.Serial_Init();
  hipRight.Serial_Init();
  kneeRight.Serial_Init();
  delay(100);

  // ---- 開機第一件事：4顆關節馬達自鎖，避免機構因無支撐而倒塌 ----
  Serial.println("===== 6馬達雙輪足機器人 初始化 =====");
  lockJoints();

  // ---- IMU 初始化 ----
  imu.begin(IMU_SERIAL, IMU_BAUD);

  // ---- PID 初始化 ----
  balancePID.init(0.0);

  Serial.println("===== 初始化完成 =====");
  Serial.println("指令：e=啟動輪子平衡  d=關閉輪子  l=關節重新自鎖  u=解鎖關節(可手動搬動腿部)");
  Serial.println("      z=偏航歸零  x=XY軸歸零  c=加速度校正");
  Serial.println(">>> 關節已自鎖，請將車體扶正後輸入 e 啟動輪子開始平衡");
}

void loop() {
  imu.update();

  // ---------- 100Hz 平衡控制迴圈（僅控制輪子）----------
  static uint32_t lastControlTime = 0;
  if (millis() - lastControlTime >= 10) {
    lastControlTime = millis();

    const IMUData& imuData = imu.getData();

    // 車體俯仰角，若感測器安裝方向不同，可能需改成 angle[0]
    double currentPitch = imuData.angle[1];

    // ---- 跌倒保護：只關閉輪子，關節維持自鎖以保護機構 ----
    if (fabs(currentPitch) > FALL_LIMIT_DEG) {
      if (wheelsEnabled) {
        disableWheels();
        Serial.println("!!! 傾角過大，已自動關閉輪子馬達 !!!");
      }
    } else if (wheelsEnabled) {
      // PID 輸出範圍已在 pid.cpp 內限制在 [-255, 255]
      double motorOutput = balancePID.compute(TARGET_ANGLE, currentPitch);

      wheelLeft.Write_angularvel_MultiRound(WHEEL_LEFT_SIGN * motorOutput);
      wheelRight.Write_angularvel_MultiRound(WHEEL_RIGHT_SIGN * motorOutput);
    }
  }

  // ---------- 10Hz 除錯輸出 ----------
  static uint32_t lastPrintTime = 0;
  if (millis() - lastPrintTime >= 100) {
    lastPrintTime = millis();
    const IMUData& imuData = imu.getData();
    Serial.printf("Roll: %6.2f | Pitch: %6.2f | Yaw: %6.2f | 溫度: %.1f°C | 輪子:%s | 關節:%s\n",
                  imuData.angle[0], imuData.angle[1], imuData.angle[2],
                  imuData.temperature,
                  wheelsEnabled ? "ON" : "OFF",
                  jointsLocked ? "LOCK" : "FREE");
  }

  handleSerialCommand();
}

void handleSerialCommand() {
  if (Serial.available() <= 0) return;
  char cmd = Serial.read();

  switch (cmd) {
    case 'e': case 'E':
      if (!jointsLocked) {
        Serial.println(">>> 關節尚未自鎖，先執行 l 鎖定關節再啟動輪子");
      } else {
        enableWheels();
      }
      break;
    case 'd': case 'D':
      disableWheels();
      break;
    case 'l': case 'L':
      lockJoints();
      break;
    case 'u': case 'U':
      unlockJoints();
      break;
    case 'z': case 'Z':
      Serial.println(">>> 執行 Z 軸偏航角歸零");
      imu.zeroYaw();
      break;
    case 'c': case 'C':
      Serial.println(">>> 執行零偏校驗，請保持模組靜止 4 秒");
      imu.calibrateAcc();
      Serial.println(">>> 校驗完成！");
      break;
    case 'x': case 'X':
      Serial.println(">>> 執行 XY 軸角度歸零");
      imu.zeroXY();
      break;
    default:
      break; // 忽略換行等其他字元
  }
}

// 讓 4 顆關節馬達（左右髖/膝）鎖定在「目前所在角度」，
// 也就是先讀取現在的實際角度，再以位置閉環命令自己回到同一角度，
// 藉此讓馬達進入伺服自鎖狀態、抵抗外力，但不會因為指令到某個固定角度而突然大幅度移動。
void lockJoints() {
  Serial.println(">>> 執行關節自鎖...");
  for (int i = 0; i < JOINT_COUNT; i++) {
    jointMotors[i]->Write_Motor_Enable();
    delay(5);
    jointMotors[i]->Read_Angle_MultiRound();          // 讀取目前實際角度
    double lockAngle = jointMotors[i]->motor_angle;
    jointMotors[i]->Write_Angle_MultiRound(lockAngle, JOINT_LOCK_SPEED); // 鎖定在該角度

    Serial.print("  "); Serial.print(jointNames[i]);
    Serial.print(" 自鎖於角度: "); Serial.println(lockAngle);
    delay(5);
  }
  jointsLocked = true;
  Serial.println(">>> 關節自鎖完成");
}

// 解除關節自鎖（關閉關節馬達），方便手動調整腿部姿態；
// 解鎖期間輪子平衡功能會被強制禁止啟動，避免機構失去支撐時輪子還在動作
void unlockJoints() {
  for (int i = 0; i < JOINT_COUNT; i++) {
    jointMotors[i]->Write_Motor_Disable();
  }
  jointsLocked = false;
  if (wheelsEnabled) {
    disableWheels();
  }
  Serial.println(">>> 關節已解鎖，可手動調整腿部姿態，完成後請輸入 l 重新自鎖");
}

void enableWheels() {
  wheelLeft.Write_Motor_Enable();
  wheelRight.Write_Motor_Enable();
  balancePID.init(0.0);   // 重置積分項，避免啟動瞬間輸出暴衝
  wheelsEnabled = true;
  Serial.println(">>> 輪子已啟動，開始平衡");
}

void disableWheels() {
  wheelLeft.Write_Motor_Disable();
  wheelRight.Write_Motor_Disable();
  wheelsEnabled = false;
  Serial.println(">>> 輪子已關閉");
}

void PrintMotorStatus(LKMotor &motor, const char *name) {
  motor.Read_Motor_Status2();
  Serial.print(name);
  Serial.print(" 速度:"); Serial.print(motor.motor_dspeed);
  Serial.print(" 溫度:"); Serial.print(motor.motor_temperature);
  Serial.print(" 電流:"); Serial.println(motor.motor_current);
}
