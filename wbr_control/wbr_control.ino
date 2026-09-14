#include <Arduino.h>
#include "config.h"
#include "command.h"

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

  // ---- 開機第一件事：4顆關節馬達自鎖 ----
  Serial.println("===== 6馬達雙輪足機器人 初始化 =====");
  lockJoints();

  // ---- IMU 初始化 ----
  imu.begin(IMU_SERIAL, IMU_BAUD);

  // ---- PID 初始化 ----
  balancePID.init(0.0);
  velPID.init(0.0);
  CurrentPID.init(0.0);

  // !! 注意 !!
  // balancePID 的輸出是「motorOutput」，會直接送進 Write_angularvel_MultiRound()；
  // 目前這裡只設了 CurrentPID 的輸出限制(-4~4A)，但 CurrentPID 並沒有被接進
  // 67Hz 控制迴圈 (見 loop() 裡的 PID 模式分支)，所以這個限制實際上完全沒有作用。
  // 真正在跑的 balancePID / velPID 目前是用 pid.h 裡的預設值 (Outmax/Outmin = ±1000)。
  // 這個值有沒有對應到你的機構安全轉速上限，請務必確認後再上機測試，
  // 建議依實際安全轉速呼叫 balancePID.setOutputLimits(...) / velPID.setOutputLimits(...)。
  CurrentPID.setOutputLimits(-4, 4);
  // TODO: balancePID.setOutputLimits(±你的安全轉速上限);
  // TODO: velPID.setOutputLimits(±你允許的俯仰角補償上限);

  Serial.println("===== 初始化完成 =====");
  Serial.println("指令：e=啟動輪子平衡  d=關閉輪子  l=關節重新自鎖  u=解鎖關節(可手動搬動腿部)");
  Serial.println("      z=偏航歸零  x=XY軸歸零  c=加速度校正  6=切換至6軸模式  9=切換至9軸模式");
  Serial.println("      m=切換 PID(板載) / MPC(PC端) 控制模式 (須先按 d 關閉輪子才能切換)");
  Serial.print(">>> 目前控制模式: ");
  Serial.println(controlMode == MODE_MPC ? "MPC" : "PID");
  Serial.println(">>> 關節已自鎖，請將車體扶正後輸入 e 啟動輪子開始平衡");
}

void loop() {
  imu.update();

  // MPC 指令是非同步收的，每個 loop() 都要 poll，不要卡在 67Hz 控制區塊裡
  mpcLink.poll();

  updateBalanceControl();  // 67Hz 平衡控制迴圈 (內部自己做15ms節流)
  printDebugInfo();        // 10Hz 除錯輸出   (內部自己做100ms節流)

  handleSerialCommand();   // 處理序列埠指令
}

// ================================================================
// 67Hz 平衡控制迴圈
// ================================================================
void updateBalanceControl() {
  static uint32_t lastControlTime = 0;
  if (millis() - lastControlTime < 15) return;
  lastControlTime = millis();

  if (!wheelsEnabled) return;
  // ---- 1. 讀取/濾波輪速 ----
  double leftFiltered  = speedFilterLeft.update(wheelLeft.motor_dspeed);
  double rightFiltered = speedFilterRight.update(wheelRight.motor_dspeed);
  Avgspeed = (-leftFiltered + rightFiltered) / 2.0;  // 左輪訊號方向與右輪相反，故取負號後平均

  // ---- 2. 讀取/濾波俯仰角 ----
  const IMUData& imuData = imu.getData();
  double rawPitch = imuData.angle[1];
  double kalmanPitchOut = kalmanPitch.update(rawPitch);      // 第一級：卡爾曼濾波，消除隨機雜訊
  filteredPitch = lowPassPitch.update(kalmanPitchOut);  // 第二級：低通濾波，消除高頻結構震動
  

  // ---- 3. 跌倒保護 (兩種控制模式都適用) ----
  if (fabs(filteredPitch) > FALL_LIMIT_DEG) {
    if (wheelsEnabled) {
      disableWheels();
      Serial.println("!!! 傾角過大，已自動關閉輪子馬達 !!!");
    }
    return;
  }

  // ---- 4. 依控制模式送出馬達指令 ----
  if (controlMode == MODE_MPC) {
    runMpcControlStep();
  } else {
    runOnboardPidControlStep();
  }
}

// MPC 模式：把狀態送給PC，套用PC回傳的扭矩指令
void runMpcControlStep() {
  double pitchRate_dps = imu.getData().gyro[1];   // 直接用IMU角速度，不用對濾波後角度微分
  mpcLink.sendState(filteredPitch, pitchRate_dps, Avgspeed, millis());

  if (!mpcLink.isFresh(MPC_TIMEOUT_MS)) {
    // PC斷線或跟不上，安全起見直接關輪子，不要用舊指令硬撐
    disableWheels();
    Serial.println("!!! MPC 連線逾時(PC無回應)，已自動關閉輪子 !!!");
    return;
  }

  double torqueCmd_Nm = mpcLink.lastTorqueCmd() / 2;   //input為兩顆輪胎合力，因此要/2來分給兩顆馬達
  torqueCmd_Nm = applyTorqueDeadzone(torqueCmd_Nm, DEADZONE_TORQUE_NM);
  double currentCmd_A = torqueCmd_Nm / MOTOR_TORQUE_CONSTANT;

  // 注意：wheelLeft / wheelRight 建構時已帶入 WHEEL_MAX_CURRENT_A，
  // Write_Torque_MultiRound() 內部會自動把電流夾限在安全範圍，
  // 這裡不需要再手動 clamp 一次。
  wheelLeft.Write_Torque_MultiRound(WHEEL_LEFT_SIGN * currentCmd_A);
  wheelRight.Write_Torque_MultiRound(WHEEL_RIGHT_SIGN * currentCmd_A);

  motorOutput = torqueCmd_Nm * 2; // 借用同一個除錯變數印出來看
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

  // 同時印出角度，方便在 Serial Plotter 觀察波形
  Serial.print("raw:");       Serial.print(imuData.angle[1]);
  Serial.print(" filtered:"); Serial.print(filteredPitch);
  Serial.print(" output:");   Serial.print(motorOutput);
  Serial.print(" current:");  Serial.print(wheelRight.motor_current);
  Serial.print(" avgspeed:"); Serial.println(Avgspeed);
  // Serial.print(" input:");    Serial.println(targetangle);
}

// 死區補償：只要指令非零，扭矩量值至少墊到 deadzone；
double applyTorqueDeadzone(double u, double deadzone) {
  if (fabs(u) < 1e-6) return 0.0;
  if (fabs(u) < deadzone) {
    return (u > 0) ? deadzone : -deadzone;
  }
  return u;
}
