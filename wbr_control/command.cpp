#include "command.h"

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

    case 'm': case 'M':
        if (wheelsEnabled) {
            Serial.println(">>> 請先按 d 關閉輪子，才能切換控制模式");
        } else {
            controlMode = (controlMode == MODE_PID) ? MODE_MPC : MODE_PID;
            Serial.print(">>> 控制模式切換為: ");
            Serial.println(controlMode == MODE_MPC ? "MPC (由PC端SerialUSB1運算)" : "PID (板載)");
        }
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

  balancePID.init(0.0);
  velPID.init(0.0);
  // CurrentPID.init(0.0);

  // 初始化所有濾波器的狀態，強制設定為當前角度與零速度，避免啟動瞬間輸出暴衝
  double currentPitch = imu.getData().angle[1];
  kalmanPitch.reset(currentPitch);     // 重置卡爾曼濾波器
  lowPassPitch.reset(currentPitch);    // 重置低通濾波器
  filteredPitch = currentPitch;

  speedFilterLeft.reset(0.0);          // 重置左輪速度濾波器
  speedFilterRight.reset(0.0);         // 重置右輪速度濾波器

  if (controlMode == MODE_MPC) {
    mpcLink.resetWatchdog();           // 避免PC還沒開始送指令就被判定逾時斷線
  }

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