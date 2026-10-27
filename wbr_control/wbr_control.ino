#include <Arduino.h>
#include "HWT906.h"
#include "LKMotor.h"
#include "filter.h"
#include "imu_fusion.h"
#include "MPCLink.h"

  // ============== 指令表 (全部由 PC 端 bridge 透過 SerialUSB1 以 "C,<seq>,<cmd>" 發送) ================
  // 小車   -> e: 啟動輪子平衡  d: 關閉輪子  l: 關節自鎖  u: 解鎖關節(可手動搬動腿部)
  // IMU    -> z: 偏航歸零  x: 靜態校正(陀螺儀零偏 + pitch/roll 零點，約1秒，車體需靜止)
  //           c: 模組加速度校正  6: 切換至6軸模式  9: 切換至9軸模式
  // Teensy 上電後「不會自己做任何動作」(不自鎖、不啟動輪子)，一律等 PC 指令。
  // =====================================

#define IMU_SERIAL   Serial2
#define IMU_BAUD     921600
#define WHEEL_LEFT_SIGN   (-1.0)
#define WHEEL_RIGHT_SIGN  (+1.0)

const double GYRO_YAW_SIGN = +1.0;   // 左轉(逆時針)為正；手轉車體確認方向
const double GYRO_PITCH_SIGN = +1.0;    // 陀螺儀方向係數 (pitch 增加 <-> gyro Y 為正)
const int JOINT_COUNT         = 4;      // 左髖、左膝、右髖、右膝
const long JOINT_LOCK_SPEED   = 20;     // 自鎖時的移動速度上限(deg/s)
const double TARGET_ANGLE     = 0.0;    // 目標角度(deg)
const double FALL_LIMIT_DEG   = 30.0;   // 傾倒保護角
const double WHEEL_MAX_CURRENT_A = 7.5; // 輪胎馬達電流上限(A)


// ================= IMU raw data -> 姿態融合 (imu_fusion.h) =================
// 不再使用模組內建融合的 angle[]，改用模組輸出的原始 acc / gyro，在 Teensy 上自己做互補濾波。
const double FUSION_TAU_S        = 0.5;   // 互補濾波時間常數(s)：越大越信任陀螺儀，加速度脈衝影響越小
const double IMU_ACC_CUTOFF_HZ   = 10.0;  // 加速度低通 (只走慢速修正路徑，延遲不影響回授)
const double IMU_GYRO_CUTOFF_HZ  = 0.0;   // 陀螺儀低通，0 = 不濾 (這條直接進 MPC 回授，濾越多延遲越大)
const double PITCH_MOUNT_OFFSET_DEG = 0.0;// 安裝角度偏移：跑一次 x 後把 Serial 印出的 pitchOffset 填在這，重開機才不用再校正
const double WHEEL_RADIUS_M      = 0.108; // 與 balance_mpc_yaw.py 的 r 一致，用來把輪速換成車體線加速度
const double LIN_ACC_GAIN        = 0.0;
const double WHEEL_ACC_LPF_ALPHA = 0.3;   // 輪速微分後的一階低通係數
const uint32_t IMU_TIMEOUT_MS    = 50;    // 輪子運轉中超過這麼久沒收到 IMU 幀 -> 關輪子

// 是否仍接受 USB Serial (Serial Monitor) 的本地指令。
// false = 只聽 PC bridge 的指令 (預設)；除錯時才改成 true。
const bool ALLOW_LOCAL_SERIAL_CMD = false;

// ================= 輪摩擦前饋設定 ================= 
const double FRICTION_LEFT_NM   = 0.3;
const double FRICTION_RIGHT_NM  = 0.3;
const double FRICTION_SPEED_EPS_DPS = 20.0;
const double FRICTION_CMD_EPS_NM    = 0.05;

// ================= MPC (PC端運算) 相關設定 =================
const unsigned long MPC_TIMEOUT_MS = 100;
const unsigned long CONTROL_PERIOD_MS = 10;
const double MOTOR_TORQUE_CONSTANT = 1.6;


// ---- 前置宣告 (避免 Arduino 自動產生原型時對 static / 引用參數處理出錯) ----
static void onImuFrame(uint8_t type, const IMUData& d);
void reportImuCalibration();
void autoGyroBiasCalOnce();
void updateWheelAccel(double avgSpeed_dps);

// ================= 馬達物件實例化 =================
LKMotor wheelLeft (4, 8,  5, WHEEL_MAX_CURRENT_A);   // 左輪馬達
LKMotor wheelRight(1, 8,  5, WHEEL_MAX_CURRENT_A);   // 右輪馬達
LKMotor hipLeft   (6, 10, 5);   // 左髖馬達
LKMotor kneeLeft  (5, 10, 5);   // 左膝馬達
LKMotor hipRight  (3, 10, 5);   // 右髖馬達
LKMotor kneeRight (2, 10, 5);   // 右膝馬達
LKMotor* jointMotors[4]  = { &hipLeft, &kneeLeft, &hipRight, &kneeRight };
const char* jointNames[4] = { "左髖", "左膝", "右髖", "右膝" };
// ================= 建立imu class物件 ===================
HWT906 imu;

ImuFusion       imuFusion;    // raw acc + gyro -> pitch / roll (互補濾波)
ImuConditioner  imuCond;      // 原始資料去突波 / 低通
static uint8_t  imuRxBuf[512];// 加大 Serial2 接收緩衝，避免控制迴圈忙碌時掉 IMU 幀
LowPassFilter speedFilterLeft(0.5); // 輪速濾波器
LowPassFilter speedFilterRight(0.5);

// ================= 狀態變數初始化 =================
double filteredPitch = 0.0;
double filteredPitchRate = 0.0;
bool wheelsEnabled = false;
bool jointsLocked  = false;
double Avgspeed = 0.0;
double motorOutputL = 0.0, motorOutputR = 0.0;    // PC 要求的 u_l / u_r (Nm)
double torqueOutputL = 0.0, torqueOutputR = 0.0;  // 含摩擦前饋後實際送出的左/右扭矩 (Nm)
double targetangle = 0.0;
double filteredYaw = 0.0;
double yawRate = 0.0;
double yawZeroDeg = 0.0;

// ---- IMU 融合輸出 (由 onImuFrame() 在每個 IMU 取樣更新) ----
double   imuYawDeg      = 0.0;   // 陀螺儀 Z 軸積分 (扣零偏)，未 wrap
double   imuYawRateDps  = 0.0;
uint32_t imuFrameCount  = 0;
uint32_t lastImuFrameMs = 0;
bool     imuAccSeen     = false;

// ---- 輪速 -> 車體前向線加速度 (g) ----
double wheelAccel_g = 0.0;

// 馬達遙測用：最近一次實際送給輪馬達的電流指令 (A，馬達座標、已夾限)
double lastCmdL_A = 0.0;
double lastCmdR_A = 0.0;

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

  // ---- IMU：raw data 進融合 ----
  IMU_SERIAL.addMemoryForRead(imuRxBuf, sizeof(imuRxBuf));   // 必須在 begin() 之前
  imuFusion.p.tau_s         = FUSION_TAU_S;
  imuFusion.p.gyroPitchSign = GYRO_PITCH_SIGN;
  imuFusion.p.linAccGain    = LIN_ACC_GAIN;
  imuFusion.pitchOffset     = PITCH_MOUNT_OFFSET_DEG;
  imuCond.p.accCutoffHz     = IMU_ACC_CUTOFF_HZ;
  imuCond.p.gyroCutoffHz    = IMU_GYRO_CUTOFF_HZ;
  imuCond.p.median          = true;     // 加速度去突波
  imuCond.p.gyroMedian      = false;    // 陀螺儀不做中值 (會多 1 個取樣延遲)
  imuCond.reconfigure();
  imu.setFrameCallback(onImuFrame);
  imu.begin(IMU_SERIAL, IMU_BAUD);

  // 不再於開機時自動自鎖關節：關節維持上電時的狀態，等 PC 送 'l' 才動作
  Serial.println(">>> Teensy 就緒，等待 PC 指令 (l 自鎖關節 -> e 啟動輪子)");
}

void loop() {
  imu.update();
  reportImuCalibration();
  handlePcCommand();
  updateBalanceControl();
  if (mpcLink.poll()) applyPcTorque();   // 收到 PC 力矩立刻寫入馬達
  if (!wheelsEnabled) printDebugInfo();  // 輪子平衡期間暫停 10Hz 序列埠大量印字，避免阻塞迴圈！
  if (ALLOW_LOCAL_SERIAL_CMD) handleSerialCommand();
}

// ================================================================
// IMU 幀處理：每收到一個完整幀就會被 HWT906 parser 呼叫
//   協定每個取樣依序送 0x51(acc) -> 0x52(gyro) -> 0x53(angle)；
//   這裡以 gyro 幀當「一個完整取樣」的觸發，acc 用剛剛收到的那一筆，
//   時間戳用 micros()，融合頻率 = IMU 輸出頻率，與 10ms 控制迴圈互相獨立。
//   模組自己算的 angle (0x53) 完全不參與計算，只在除錯輸出拿來對照。
// ================================================================
static double wrapDeg180(double x) { return x - 360.0 * floor((x + 180.0) / 360.0); }

static void onImuFrame(uint8_t type, const IMUData& d) {
  if (type == (uint8_t)FrameType::ACC_F) { imuAccSeen = true; return; }
  if (type != (uint8_t)FrameType::GYRO_F || !imuAccSeen) return;

  double a[3] = { d.acc[0],  d.acc[1],  d.acc[2]  };   // g
  double g[3] = { d.gyro[0], d.gyro[1], d.gyro[2] };   // deg/s
  double aF[3], gF[3];
  imuCond.process(a, g, 1.0 / imuFusion.dt, aF, gF);
  double linF = imuCond.processLin(wheelAccel_g);      // 延遲與 aF 對齊
  imuFusion.stepMicros(aF, gF, micros(), linF);

  // yaw：acc 無法提供航向，只能積分 gyro Z (零偏取自靜態校正)
  double gz = GYRO_YAW_SIGN * (gF[2] - imuFusion.gyroBias[2]);
  imuYawRateDps = gz;
  imuYawDeg    += gz * imuFusion.dt;

  imuFrameCount++;
  lastImuFrameMs = millis();
}

// 靜態校正結果回報 (校正是非阻塞的，在 loop 裡輪詢狀態)
void reportImuCalibration() {
  static int lastReported = 0;
  int st = imuFusion.calStatus();
  if (st == 1 || st == lastReported) { if (st == 1) lastReported = 1; return; }
  lastReported = st;
  if (st == 2) {
    Serial.print(">>> 校正完成  gyroBias(dps)= ");
    Serial.print(imuFusion.gyroBias[0], 4); Serial.print(", ");
    Serial.print(imuFusion.gyroBias[1], 4); Serial.print(", ");
    Serial.print(imuFusion.gyroBias[2], 4);
    Serial.print("  pitchOffset(deg)= "); Serial.print(imuFusion.pitchOffset, 4);
    Serial.print("  rollOffset(deg)= ");  Serial.println(imuFusion.rollOffset, 4);
    Serial.println(">>> 要重開機後仍保留 pitch 零點：把 pitchOffset 填進 PITCH_MOUNT_OFFSET_DEG");
  } else if (st == -1) {
    Serial.println(">>> 校正失敗：校正期間車體有移動 (>3 dps)，請保持靜止後重試");
  }
}

// 開機後自動做一次「只估陀螺儀零偏」的校正 (不動 pitch 零點)；若當下車體在動就放棄
void autoGyroBiasCalOnce() {
  static bool done = false;
  if (done || wheelsEnabled || imuFrameCount < 200 || millis() < 2000) return;
  done = true;
  imuFusion.startCalibration(constrain((int)(1.0 / imuFusion.dt), 100, 400), false);
}

// 輪速 -> 車體前向線加速度 (g)。只在輪子運轉、且有新的馬達回覆時才微分，
// 否則 motor_dspeed 是舊值 (輪子關閉時只有除錯輸出每 100ms 會更新一次)，微分出來全是假脈衝。
void updateWheelAccel(double avgSpeed_dps) {
  static uint32_t lastCntL = 0, lastCntR = 0, lastMs = 0;
  static double vPrev = 0.0;
  if (!wheelsEnabled) { wheelAccel_g = 0.0; lastMs = 0; return; }

  bool fresh = (wheelLeft.status_rx_count != lastCntL) || (wheelRight.status_rx_count != lastCntR);
  lastCntL = wheelLeft.status_rx_count;
  lastCntR = wheelRight.status_rx_count;
  uint32_t now = millis();
  double v = avgSpeed_dps * (M_PI / 180.0) * WHEEL_RADIUS_M;      // m/s，前進為正
  if (lastMs == 0) { vPrev = v; lastMs = now; wheelAccel_g = 0.0; return; }
  if (!fresh) return;
  double dts = (now - lastMs) * 1e-3;
  if (dts <= 0.0) return;
  double acc_g = ((v - vPrev) / dts) / 9.81;
  wheelAccel_g += WHEEL_ACC_LPF_ALPHA * (acc_g - wheelAccel_g);
  vPrev = v; lastMs = now;
}

// ================================================================
// 平衡控制迴圈 (CONTROL_PERIOD_MS 節流)
// ================================================================
void updateBalanceControl() {
  static uint32_t lastControlTime = 0;
  uint32_t now = millis();
  if (now - lastControlTime < CONTROL_PERIOD_MS) return;

  lastControlTime = now;

  // 讀取輪速
  double leftFiltered  = speedFilterLeft.update(wheelLeft.motor_dspeed);
  double rightFiltered = speedFilterRight.update(wheelRight.motor_dspeed);
  Avgspeed = (-leftFiltered + rightFiltered) / 2.0;

  updateWheelAccel(Avgspeed);
  autoGyroBiasCalOnce();

  // 俯仰角與角速度：來自 raw acc/gyro 的互補濾波結果 (在 onImuFrame 中更新)
  filteredYaw       = wrapDeg180(imuYawDeg - yawZeroDeg);
  yawRate           = imuYawRateDps;
  filteredPitch     = imuFusion.pitch;
  filteredPitchRate = imuFusion.pitchRate;     // 已扣零偏的歐拉角速度

  if (!wheelsEnabled) return;

  // IMU 資料中斷保護：融合結果會停在最後一筆，必須關輪子，不然會拿舊姿態繼續控制
  if (millis() - lastImuFrameMs > IMU_TIMEOUT_MS) {
    disableWheels();
    mpcLink.sendAck('F', 1);      // 沿用 F 讓 PC 端同步關閉 (bridge 只認 F/T)，原因看 Teensy Serial
    Serial.println("!!! IMU 資料中斷，已自動關閉輪子 !!!");
    return;
  }

  // 跌倒保護
  if (fabs(filteredPitch) > FALL_LIMIT_DEG) {
    disableWheels();
    mpcLink.sendAck('F', 1);      // 通知 PC：傾倒保護觸發
    return;
  }

  runMpcControlStep(leftFiltered, rightFiltered);
}

// MPC 模式：把狀態送給PC，套用PC回傳的扭矩指令
void runMpcControlStep(double leftSpeed_dps, double rightSpeed_dps) {
  double vL_fwd = WHEEL_LEFT_SIGN  * leftSpeed_dps;   // 前進為正
  double vR_fwd = WHEEL_RIGHT_SIGN * rightSpeed_dps;

  // ---- 馬達遙測 (前進為正) ----
  // 注意時序：current/speed 是「上一拍 Write_Torque 的回覆」，所以這裡送出的
  // (cmd, i, w) 是同一拍的配對：上一拍送出的指令 + 它換來的回授。
  static uint32_t lastCntL = 0, lastCntR = 0;
  MotorTelemetry tm;
  tm.iL_A   = WHEEL_LEFT_SIGN  * wheelLeft.motor_current;
  tm.iR_A   = WHEEL_RIGHT_SIGN * wheelRight.motor_current;
  tm.cmdL_A = WHEEL_LEFT_SIGN  * lastCmdL_A;
  tm.cmdR_A = WHEEL_RIGHT_SIGN * lastCmdR_A;
  tm.wL_dps = WHEEL_LEFT_SIGN  * wheelLeft.motor_dspeed;
  tm.wR_dps = WHEEL_RIGHT_SIGN * wheelRight.motor_dspeed;
  tm.fresh  = (wheelLeft.status_rx_count  != lastCntL ? 1 : 0)
            | (wheelRight.status_rx_count != lastCntR ? 2 : 0);
  lastCntL = wheelLeft.status_rx_count;
  lastCntR = wheelRight.status_rx_count;

  mpcLink.sendState(filteredPitch, filteredPitchRate, vL_fwd, vR_fwd,
                    filteredYaw, yawRate, millis(), &tm);

  if (!mpcLink.isFresh(MPC_TIMEOUT_MS)) {
    disableWheels();
    mpcLink.sendAck('T', 1);      // 通知 PC：逾時關閉
    Serial.println("!!! MPC 連線逾時(PC無回應)，已自動關閉輪子 !!!");
    return;
  }
}

void applyPcTorque() {
  if (!wheelsEnabled) return;
  double vL_fwd = WHEEL_LEFT_SIGN  * speedFilterLeft.output;
  double vR_fwd = WHEEL_RIGHT_SIGN * speedFilterRight.output;

  double baseL = mpcLink.lastTorqueL();
  double baseR = mpcLink.lastTorqueR();
  double torqueLeft_Nm  = applyFrictionFF(baseL, vL_fwd, FRICTION_LEFT_NM);
  double torqueRight_Nm = applyFrictionFF(baseR, vR_fwd, FRICTION_RIGHT_NM);

  double cmdL_A = WHEEL_LEFT_SIGN  * torqueLeft_Nm  / MOTOR_TORQUE_CONSTANT;
  double cmdR_A = WHEEL_RIGHT_SIGN * torqueRight_Nm / MOTOR_TORQUE_CONSTANT;

  lastCmdL_A = constrain(cmdL_A, -WHEEL_MAX_CURRENT_A, WHEEL_MAX_CURRENT_A);
  lastCmdR_A = constrain(cmdR_A, -WHEEL_MAX_CURRENT_A, WHEEL_MAX_CURRENT_A);
  wheelLeft.Write_Torque_MultiRound(cmdL_A);
  wheelRight.Write_Torque_MultiRound(cmdR_A);

  motorOutputL = baseL;  motorOutputR = baseR;
  torqueOutputL = torqueLeft_Nm;  torqueOutputR = torqueRight_Nm;
}

// ================================================================
// 10Hz 除錯輸出
// ================================================================
void printDebugInfo() {
  static uint32_t lastPrintTime = 0;
  if (millis() - lastPrintTime < 100) return;
  lastPrintTime = millis();
  const IMUData& imuData = imu.getData();

  // fus = 融合 pitch；mod = 模組內建 angle[1] (僅對照)；accR/accC = 純加速度算出的 pitch (未補償/補償後)
  Serial.print(" fus:");      Serial.print(filteredPitch);
  Serial.print(" mod:");      Serial.print(imuData.angle[1]);
  Serial.print(" accR:");     Serial.print(imuFusion.accPitchRaw);
  Serial.print(" accC:");     Serial.print(imuFusion.accPitchComp);
  Serial.print(" trust:");    Serial.print(imuFusion.trust, 2);
  Serial.print(" la_g:");     Serial.print(wheelAccel_g, 3);
  Serial.print(" biasY:");    Serial.print(imuFusion.gyroBias[1], 3);
  Serial.print(" imuHz:");    Serial.print(1.0 / imuFusion.dt, 0);
  Serial.print(" yaw:");      Serial.print(filteredYaw);
  Serial.print(" gyroY:");    Serial.print(filteredPitchRate);
  Serial.print(" uL:");       Serial.print(motorOutputL);
  Serial.print(" uR:");       Serial.print(motorOutputR);
  Serial.print(" uLapp:");    Serial.print(torqueOutputL);
  Serial.print(" uRapp:");    Serial.print(torqueOutputR);
  Serial.print(" current:");  Serial.print(wheelRight.motor_current);
  Serial.print(" avgspeed:"); Serial.println(Avgspeed);
  // PrintMotorStatus(wheelLeft,  "左輪");
  // PrintMotorStatus(wheelRight, "右輪");
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

// ================================================================
// 操作指令：統一入口。PC (C 封包) 與本地 Serial 都走這裡。
// 回傳 true = 已執行；false = 被拒絕 (條件不符)。
// ================================================================
bool executeCommand(char cmd) {
  if (cmd >= 'A' && cmd <= 'Z') cmd += 'a' - 'A';

  // 會阻塞數秒的 IMU 指令，輪子運轉中一律拒絕，否則控制迴圈會停擺
  bool isImuCmd = (cmd == 'x' || cmd == 'c' || cmd == '6' || cmd == '9');   // z 現在是軟體歸零，不阻塞
  if (isImuCmd && wheelsEnabled) {
    Serial.println(">>> 輪子運轉中，拒絕 IMU 指令，請先關閉輪子 (d)");
    return false;
  }

  switch (cmd) {
    case 'e':
      if (!jointsLocked) {
        Serial.println(">>> 關節尚未自鎖，先執行 l 鎖定關節再啟動輪子");
        return false;
      }
      enableWheels();
      return true;

    case 'd':
      disableWheels();
      return true;

    case 'l':
      if (wheelsEnabled) {
        Serial.println(">>> 輪子運轉中，拒絕關節自鎖，請先關閉輪子 (d)");
        return false;
      }
      lockJoints();
      return true;

    case 'u':
      unlockJoints();
      return true;

    case 'z':
      Serial.println(">>> 偏航角歸零 (軟體)");
      yawZeroDeg = imuYawDeg;
      return true;

    case 'c':
      Serial.println(">>> 執行模組加速度校正，請保持模組靜止 4 秒 (阻塞)");
      imu.calibrateAcc();
      Serial.println(">>> 校驗完成！");
      return true;

    case 'x':
      Serial.println(">>> 開始靜態校正 (約 1 秒)：請保持車體靜止在要當作 pitch=0 的姿態");
      imuFusion.startCalibration(constrain((int)(1.0 / imuFusion.dt), 100, 400), true);
      return true;

    case '6':
      Serial.println(">>> 切換至 6 軸模式");
      imu.switchTo6Axis();
      return true;

    case '9':
      Serial.println(">>> 切換至 9 軸模式");
      imu.switchTo9Axis();
      return true;

    default:
      return false;
  }
}

// 處理 PC 發來的 C 封包，執行後回 A 封包
void handlePcCommand() {
  char cmd;
  while (mpcLink.takeCommand(cmd)) {
    bool ok = executeCommand(cmd);
    mpcLink.sendAck(cmd, ok ? 1 : 0);
  }
}

// 本地 Serial 指令 (僅 ALLOW_LOCAL_SERIAL_CMD = true 時使用)
void handleSerialCommand() {
  if (Serial.available() <= 0) return;
  executeCommand((char)Serial.read());
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
  jointMotors[0]->Write_Angle_MultiRound(-50.24,   JOINT_LOCK_SPEED);  //-50.24     左髖
  jointMotors[1]->Write_Angle_MultiRound(76.22,  JOINT_LOCK_SPEED);   //76.22    左膝
  jointMotors[2]->Write_Angle_MultiRound(50.24,    JOINT_LOCK_SPEED); //50.24      右髖
  jointMotors[3]->Write_Angle_MultiRound(-76.22, JOINT_LOCK_SPEED);    //-76.22   右膝
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
  Serial.println(">>> 關節已解鎖，可手動調整腿部姿態，完成後請由 PC 送 l 重新自鎖");
}

// 輪子啟動 / 關閉
void enableWheels() {
  wheelLeft.Write_Motor_Enable();
  wheelRight.Write_Motor_Enable();

  // 融合濾波器從開機就持續在跑 (已收斂)，這裡不重置；只把 yaw 零點設成目前航向
  yawZeroDeg = imuYawDeg;

  speedFilterLeft.reset(0.0);          // 重置左輪速度濾波器
  speedFilterRight.reset(0.0);         // 重置右輪速度濾波器

  mpcLink.resetWatchdog();           // 避免PC還沒開始送指令就被判定逾時斷線
  mpcLink.clearTorqueCmd();          // 清掉上一輪殘留的扭矩指令，第一拍從 0 開始

  wheelsEnabled = true;
  Serial.println(">>> 輪子已啟動，開始平衡");
}

void disableWheels() {
  wheelLeft.Write_Motor_Disable();
  wheelRight.Write_Motor_Disable();
  wheelsEnabled = false;
  motorOutputL = motorOutputR = 0.0;
  torqueOutputL = torqueOutputR = 0.0;
  targetangle = 0.0;
  Avgspeed = 0.0;
  wheelAccel_g = 0.0;
  speedFilterLeft.reset(0.0); 
  speedFilterRight.reset(0.0);
  wheelLeft.motor_current = 0.0; 
  wheelRight.motor_current = 0.0;
  Serial.println(">>> 輪子已關閉");
}


