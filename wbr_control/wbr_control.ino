#include <Arduino.h>
#include "HWT906.h"
#include "LKMotor.h"
#include "filter.h"
#include "MPCLink.h"

  // ============== 指令表 ================
  // 小車   -> e: 啟動輪子平衡  d: 關閉輪子  l: 關節重新自鎖  u: 解鎖關節(可手動搬動腿部)
  // IMU    -> z: 偏航歸零  x: XY軸歸零  c: 加速度校正  6: 切換至6軸模式  // 9: 切換至9軸模式
  // =====================================

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
const double GYRO_YAW_SIGN = +1.0;   // 左轉(逆時針)為正；手轉車體確認方向
const double YAW_ANGLE_SIGN = +1.0;

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

// ================= 腿部高度控制 (PC 端算 IK，Teensy 只執行) =================
// PC 傳 J,<seq>,<dq1_deg>,<dq2_deg> = 相對「自鎖姿態」的 q1/q2 增量，
// 左右腿對稱動作；馬達角度 = 符號 * dq。【符號必須實機校正】：
// 先用很小的高度變化 (按 w 一下) 看腿是往上伸還是往下縮、左右是否對稱，不對就改這四個符號。
const double JOINT_DQ_SIGN[4]     = { -1.0, -1.0, +1.0, +1.0 };  // 左髖、左膝、右髖、右膝 (依自鎖註解的鏡像符號猜的)
const double JOINT_DQ_LIMIT_DEG   = 70.0;   // 安全夾限：相對自鎖姿態最多移動幾度
const long   JOINT_MOVE_SPEED_DPS = 10;     // 位置模式速度上限 (輸出軸 deg/s)；實際速度由 PC 端的高度斜率決定

// ================= MPC (PC端運算) 相關設定 =================
const unsigned long MPC_TIMEOUT_MS = 100;

const unsigned long CONTROL_PERIOD_MS = 15;

const double MOTOR_TORQUE_CONSTANT = 1.6;

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

PitchEstimator pitchEstimator(PITCH_FUSE_ALPHA); // 俯仰角估測器 
LowPassFilter speedFilterLeft(0.5); // 輪速濾波器
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
double filteredYaw = 0.0;
double yawRate = 0.0;
double yawZeroDeg = 0.0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  // ---- MPC 通訊埠初始化 (SerialUSB1，需在 Tools->USB Type 選 Dual/Triple Serial) ----
  mpcLink.begin();

  // ---- 馬達初始化 ----
  wheelLeft.Serial_Init();
  wheelRight.Serial_Init();
  hipLeft.Serial_Init();
  kneeLeft.Serial_Init();
  hipRight.Serial_Init();
  kneeRight.Serial_Init();
  delay(100);

  // ---- 關節馬達自鎖 ----
  lockJoints();
  imu.begin(IMU_SERIAL, IMU_BAUD);
}

void loop() {
  imu.update();
  mpcLink.poll();
  applyJointCommand();

  updateBalanceControl();
  printDebugInfo();

  handleSerialCommand();
}

// 套用 PC 傳來的腿部高度指令 (僅在關節已自鎖時有效)
void applyJointCommand() {
  double dq1, dq2;
  if (!mpcLink.takeJointCmd(dq1, dq2)) return;
  if (!jointsLocked) return;

  dq1 = constrain(dq1, -JOINT_DQ_LIMIT_DEG, JOINT_DQ_LIMIT_DEG);
  dq2 = constrain(dq2, -JOINT_DQ_LIMIT_DEG, JOINT_DQ_LIMIT_DEG);

  // 順序：左髖、左膝、右髖、右膝；0=髖(q1)，1=膝(q2)
  jointMotors[0]->Write_Angle_MultiRound(JOINT_DQ_SIGN[0] * dq1, JOINT_MOVE_SPEED_DPS);
  jointMotors[1]->Write_Angle_MultiRound(JOINT_DQ_SIGN[1] * (dq1 + dq2), JOINT_MOVE_SPEED_DPS);
  jointMotors[2]->Write_Angle_MultiRound(JOINT_DQ_SIGN[2] * dq1, JOINT_MOVE_SPEED_DPS);
  jointMotors[3]->Write_Angle_MultiRound(JOINT_DQ_SIGN[3] * (dq1 + dq2), JOINT_MOVE_SPEED_DPS);
}

// ================================================================
// 平衡控制迴圈 (CONTROL_PERIOD_MS 節流)
// ================================================================
void updateBalanceControl() {
  static uint32_t lastControlTime = 0;
  uint32_t now = millis();
  if (now - lastControlTime < CONTROL_PERIOD_MS) return;

  double dt_s;
  if (lastControlTime == 0){
    dt_s = CONTROL_PERIOD_MS * 1e-3;
  } else{
    dt_s = (now - lastControlTime) * 1e-3;
  }
  lastControlTime = now;

  // 讀取輪速
  double leftFiltered  = speedFilterLeft.update(wheelLeft.motor_dspeed);
  double rightFiltered = speedFilterRight.update(wheelRight.motor_dspeed);
  Avgspeed = (-leftFiltered + rightFiltered) / 2.0;

  // 俯仰角與角速度
  const IMUData& imuData = imu.getData();
  double rawYaw = YAW_ANGLE_SIGN * imuData.angle[2] - yawZeroDeg;
  filteredYaw = fmod(rawYaw + 540.0, 360.0) - 180.0;      // wrap 到 -180~180
  yawRate     = GYRO_YAW_SIGN * imuData.gyro[2];
  double rawPitch     = imuData.angle[1];
  double rawPitchRate = GYRO_PITCH_SIGN * imuData.gyro[1];
  filteredPitchRate   = rawPitchRate;


  filteredPitch = pitchEstimator.update(rawPitch, rawPitchRate, dt_s);

  if (!wheelsEnabled) return;

  // 跌倒保護
  if (fabs(filteredPitch) > FALL_LIMIT_DEG) {
    disableWheels();
    return;
  }

  runMpcControlStep(leftFiltered, rightFiltered);
}

// MPC 模式：把狀態送給PC，套用PC回傳的扭矩指令
void runMpcControlStep(double leftSpeed_dps, double rightSpeed_dps) {
  double vL_fwd = WHEEL_LEFT_SIGN  * leftSpeed_dps;   // 前進為正
  double vR_fwd = WHEEL_RIGHT_SIGN * rightSpeed_dps;

  mpcLink.sendState(filteredPitch, filteredPitchRate, vL_fwd, vR_fwd,
                    filteredYaw, yawRate, millis());

  if (!mpcLink.isFresh(MPC_TIMEOUT_MS)) {
    disableWheels();
    Serial.println("!!! MPC 連線逾時(PC無回應)，已自動關閉輪子 !!!");
    return;
  }

  double uSum  = mpcLink.lastTorqueCmd();
  double uDiff = mpcLink.lastTorqueDiff();
  double baseL = (uSum - uDiff) / 2.0;   // 左輪扭矩(前進為正)
  double baseR = (uSum + uDiff) / 2.0;

  double torqueLeft_Nm  = applyFrictionFF(baseL, vL_fwd, FRICTION_LEFT_NM);
  double torqueRight_Nm = applyFrictionFF(baseR, vR_fwd, FRICTION_RIGHT_NM);

  wheelLeft.Write_Torque_MultiRound(WHEEL_LEFT_SIGN  * torqueLeft_Nm  / MOTOR_TORQUE_CONSTANT);
  wheelRight.Write_Torque_MultiRound(WHEEL_RIGHT_SIGN * torqueRight_Nm / MOTOR_TORQUE_CONSTANT);

  motorOutput  = uSum;
  torqueOutput = torqueLeft_Nm + torqueRight_Nm;
}

// ================================================================
// 10Hz 除錯輸出
// ================================================================
void printDebugInfo() {
  static uint32_t lastPrintTime = 0;
  if (millis() - lastPrintTime < 100) return;
  lastPrintTime = millis();
  const IMUData& imuData = imu.getData();

  Serial.print("raw:");       Serial.print(imuData.angle[1]);
  Serial.print(" est:");      Serial.print(filteredPitch);
  Serial.print(" yaw:");      Serial.print(filteredYaw);
  Serial.print(" gyroY:");    Serial.print(GYRO_PITCH_SIGN * imuData.gyro[1]);
  Serial.print(" uReq:");     Serial.print(motorOutput);
  Serial.print(" uApp:");     Serial.print(torqueOutput);
  Serial.print(" current:");  Serial.print(wheelRight.motor_current);
  Serial.print(" avgspeed:"); Serial.println(Avgspeed);
}



// ================================================================
// 摩擦前饋：tau_out = u + Fc * dir
//   speed_dps : 該輪在「與 u 相同座標」下的角速度
//   dir       : 輪子在轉 -> 取輪速方向；快停下來 -> 平滑過渡到指令方向
// 兩段都用 tanh() 做連續過渡，整條曲線在 u=0 附近是連續的，
// 不會像原本的死區墊高那樣產生 ±Fc 的跳變(那正是極限環的來源)。
// ================================================================
double applyFrictionFF(double u, double speed_dps, double Fc) {
  double dirSpeed = tanh(speed_dps / FRICTION_SPEED_EPS_DPS);   // -1 ~ +1
  double dirCmd   = tanh(u / FRICTION_CMD_EPS_NM);              // -1 ~ +1
  double w        = fabs(dirSpeed);                             // 輪子越快越信任輪速方向
  double dir      = w * dirSpeed + (1.0 - w) * dirCmd;
  return u + Fc * dir;
}

// 序列埠指令函式
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

    case '6':
        Serial.println(">>> 切換至 6 軸模式");
        imu.switchTo6Axis();
        break;

    case '9':
        Serial.println(">>> 切換至 9 軸模式");
        imu.switchTo9Axis();
        break;

    default:
        break;
  }
}

// 4關節特定角度自鎖
void lockJoints() {
  Serial.println(">>> 執行關節自鎖 (將移動至固定預設姿態)...");

  for (int i = 0; i < JOINT_COUNT; i++) {
    jointMotors[i]->Write_Motor_Enable();
    delay(5);

    // 純粹讀出目前角度供記錄/除錯用，不會拿來當作鎖定目標
    jointMotors[i]->Read_Angle_MultiRound();
    Serial.print("  "); Serial.print(jointNames[i]);
    Serial.print(" 目前角度: "); Serial.println(jointMotors[i]->motor_angle);
    delay(5);
  }

  // 設定關節角度(設馬達出軸方向為負)
  jointMotors[0]->Write_Angle_MultiRound(0,   JOINT_LOCK_SPEED);  //-45     左髖
  jointMotors[1]->Write_Angle_MultiRound(0,  JOINT_LOCK_SPEED);   //70.2    左膝
  jointMotors[2]->Write_Angle_MultiRound(0,    JOINT_LOCK_SPEED); //45      右髖
  jointMotors[3]->Write_Angle_MultiRound(0, JOINT_LOCK_SPEED);    //-70.2   右膝
  jointsLocked = true;
  Serial.println(">>> 關節自鎖完成 (已移動至預設姿態)");
}

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

// 輪子啟動 / 關閉
void enableWheels() {
  wheelLeft.Write_Motor_Enable();
  wheelRight.Write_Motor_Enable();

  // 初始化所有濾波器的狀態，強制設定為當前角度與零速度，避免啟動瞬間輸出暴衝
  yawZeroDeg = YAW_ANGLE_SIGN * imu.getData().angle[2];
  double currentPitch = imu.getData().angle[1];
  filteredPitch = currentPitch;

  speedFilterLeft.reset(0.0);          // 重置左輪速度濾波器
  speedFilterRight.reset(0.0);         // 重置右輪速度濾波器

  mpcLink.resetWatchdog();           // 避免PC還沒開始送指令就被判定逾時斷線

  wheelsEnabled = true;
  Serial.println(">>> 輪子已啟動，開始平衡");
}

void disableWheels() {
  wheelLeft.Write_Motor_Disable();
  wheelRight.Write_Motor_Disable();
  wheelsEnabled = false;
  motorOutput = 0.0;
  targetangle = 0.0;
  Avgspeed = 0.0;
  filteredPitch = 0.0;
  speedFilterLeft.reset(0.0); 
  speedFilterRight.reset(0.0);
  wheelLeft.motor_current = 0.0; 
  wheelRight.motor_current = 0.0;
  Serial.println(">>> 輪子已關閉");
}

// 除錯輸出
void PrintMotorStatus(LKMotor &motor, const char *name) {
  motor.Read_Motor_Status2();
  Serial.print(name);
  Serial.print(" 速度:"); Serial.print(motor.motor_dspeed);
  Serial.print(" 溫度:"); Serial.print(motor.motor_temperature);
  Serial.print(" 電流:"); Serial.println(motor.motor_current);
}
