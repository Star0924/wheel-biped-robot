#include <Arduino.h>
#include "config.h"
#include "command.h"


void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

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
  CurrentPID.setOutputLimits(-4, 4);

  Serial.println("===== 初始化完成 =====");
  Serial.println("指令：e=啟動輪子平衡  d=關閉輪子  l=關節重新自鎖  u=解鎖關節(可手動搬動腿部)");
  Serial.println("      z=偏航歸零  x=XY軸歸零  c=加速度校正  6=切換至6軸模式  9=切換至9軸模式");
  Serial.println(">>> 關節已自鎖，請將車體扶正後輸入 e 啟動輪子開始平衡");
}

void loop() {
  imu.update();

  // ---------- 100Hz 平衡控制迴圈 ----------
  static uint32_t lastControlTime = 0;
  if (millis() - lastControlTime >= 10) {
    lastControlTime = millis();

    const IMUData& imuData = imu.getData();
    // 在 loop() 中分開過濾兩顆輪子的速度
    double leftFiltered = speedFilterLeft.update(wheelLeft.motor_dspeed);
    double rightFiltered = speedFilterRight.update(wheelRight.motor_dspeed);

    // 然後再計算平均 (注意你原本左輪有加負號)
    Avgspeed = (-leftFiltered + rightFiltered) / 2.0;

    // 1. 取得原始俯仰角
    double rawPitch = imuData.angle[1];

    // 2. 第一級：卡爾曼濾波 (消除隨機雜訊)
    double kalmanPitchOut = kalmanPitch.update(rawPitch);

    // 3. 第二級：低通濾波耦合 (消除高頻結構震動)
    finalFilteredPitch = lowPassPitch.update(kalmanPitchOut);

    // ---- 跌倒保護 ----
    if (fabs(finalFilteredPitch) > FALL_LIMIT_DEG) {
      if (wheelsEnabled) {
        disableWheels();
        Serial.println("!!! 傾角過大，已自動關閉輪子馬達 !!!");
      }
    } else if (wheelsEnabled) {
      // PID 計算改用最終雙重濾波後的角度
      targetangle = -velPID.compute(0.0, Avgspeed);
      motorOutput = balancePID.compute(targetangle, finalFilteredPitch);
      // torqueOutput = CurrentPID.compute(targetangle, finalFilteredPitch);

      wheelLeft.Write_angularvel_MultiRound(WHEEL_LEFT_SIGN * motorOutput);
      wheelRight.Write_angularvel_MultiRound(WHEEL_RIGHT_SIGN * motorOutput);
      // wheelLeft.Write_Torque_MultiRound(WHEEL_LEFT_SIGN * 0.2);
      // wheelRight.Write_Torque_MultiRound(WHEEL_RIGHT_SIGN * 0.2);
    }
  }

  // ---------- 10Hz 除錯輸出 ----------
  static uint32_t lastPrintTime = 0;
  if (millis() - lastPrintTime >= 100) {
    lastPrintTime = millis();
    const IMUData& imuData = imu.getData();
    
    // 同時印出角度，方便在 Serial Plotter 觀察波形
    // Serial.print("raw:");   Serial.print(imuData.angle[1]);
    // Serial.print(" filtered:"); Serial.print(finalFilteredPitch);
    // Serial.print(" output:");   Serial.println(motorOutput);
    Serial.print("current:");   Serial.println(wheelRight.motor_current);
    Serial.print("avgspeed:");   Serial.println(Avgspeed);
    // Serial.print("targetangle:");   Serial.println(targetangle);
  }

  // ---------- 處理序列埠指令 ----------
  handleSerialCommand();
}