#include <Arduino.h>
#include "config.h"
#include "command.h"

  // ============== 指令表 ================
  // 小車   -> e: 啟動輪子平衡  d: 關閉輪子  l: 關節重新自鎖  u: 解鎖關節(可手動搬動腿部)
  // IMU    -> z: 偏航歸零  x: XY軸歸零  c: 加速度校正  6: 切換至6軸模式  // 9: 切換至9軸模式
  // 控制器  -> m: 切換 PID(板載) / MPC(PC端) 控制模式 (須先按 d 關閉輪子才能切換)
  // =====================================

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

  balancePID.init(0.0);
  velPID.init(0.0);
  // CurrentPID.init(0.0);
  // CurrentPID.setOutputLimits(-4, 4);
}

void loop() {
  imu.update();
  mpcLink.poll();

  updateBalanceControl();
  printDebugInfo();

  handleSerialCommand();
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
  double rawPitch     = imuData.angle[1];
  double rawPitchRate = GYRO_PITCH_SIGN * imuData.gyro[1];
  filteredPitchRate   = rawPitchRate;

  if (controlMode == MODE_MPC) {
    filteredPitch = pitchEstimator.update(rawPitch, rawPitchRate, dt_s);
  } else {
    filteredPitch = lowPassPitch.update(kalmanPitch.update(rawPitch));
  }

  if (!wheelsEnabled) return;

  // 跌倒保護
  if (fabs(filteredPitch) > FALL_LIMIT_DEG) {
    disableWheels();
    return;
  }

  // 依控制模式送出馬達指令
  if (controlMode == MODE_MPC) {
    runMpcControlStep(leftFiltered, rightFiltered);
  } else {
    runOnboardPidControlStep();
  }
}

// MPC 模式：把狀態送給PC，套用PC回傳的扭矩指令
void runMpcControlStep(double leftSpeed_dps, double rightSpeed_dps) {
  // 先送狀態，讓 PC 有最長的時間可以算
  mpcLink.sendState(filteredPitch, filteredPitchRate, Avgspeed, millis());

  if (!mpcLink.isFresh(MPC_TIMEOUT_MS)) {
    disableWheels();
    Serial.println("!!! MPC 連線逾時(PC無回應)，已自動關閉輪子 !!!");
    return;
  }

  double baseTorque_Nm = mpcLink.lastTorqueCmd() / 2.0;   // 單輪扭矩

  // 【修改】摩擦前饋取代死區墊高。方向以「該輪實際轉向」為主，
  // 停止時才平滑過渡到指令方向，零點附近不會跳變。
  // 注意左輪機械方向與訊號相反(WHEEL_LEFT_SIGN=-1)，所以判斷方向時要先轉成同一個座標。
  double torqueLeft_Nm  = applyFrictionFF(baseTorque_Nm,
                                          WHEEL_LEFT_SIGN * leftSpeed_dps,
                                          FRICTION_LEFT_NM);
  double torqueRight_Nm = applyFrictionFF(baseTorque_Nm,
                                          WHEEL_RIGHT_SIGN * rightSpeed_dps,
                                          FRICTION_RIGHT_NM);

  wheelLeft.Write_Torque_MultiRound(WHEEL_LEFT_SIGN  * torqueLeft_Nm  / MOTOR_TORQUE_CONSTANT);
  wheelRight.Write_Torque_MultiRound(WHEEL_RIGHT_SIGN * torqueRight_Nm / MOTOR_TORQUE_CONSTANT);

  motorOutput  = baseTorque_Nm * 2.0;                 // MPC 要求的總扭矩
  torqueOutput = torqueLeft_Nm + torqueRight_Nm;      // 實際送出的總扭矩(含摩擦前饋)
}

// 板載 PID 模式（原本邏輯，維持不變）
void runOnboardPidControlStep() {
  targetangle = -velPID.compute(0.0, Avgspeed);
  motorOutput = balancePID.compute(targetangle, filteredPitch);
  wheelLeft.Write_angularvel_MultiRound(WHEEL_LEFT_SIGN * motorOutput);
  wheelRight.Write_angularvel_MultiRound(WHEEL_RIGHT_SIGN * motorOutput);
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

// 保留舊版死區補償供對照/回退使用（目前未被呼叫）
// double applyTorqueDeadzone(double u, double deadzone) {
//   if (fabs(u) < 1e-6) return 0.0;
//   if (fabs(u) < deadzone) {
//     return (u > 0) ? deadzone : -deadzone;
//   }
//   return u;
// }
