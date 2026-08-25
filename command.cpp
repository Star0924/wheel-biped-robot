#include "command.h"

void handleSerialCommand() {
  if (Serial.available() <= 0) return;
  char cmd = Serial.read();

  switch (cmd) {
    case 'e': case 'E':
        if (!jointsLocked){ Serial.println(">>> 關節尚未自鎖，先執行 l 鎖定關節再啟動輪子");} 
        else{ enableWheels();}
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

void lockJoints() {
  Serial.println(">>> 執行關節自鎖...");
  for (int i = 0; i < JOINT_COUNT; i++) {
    jointMotors[i]->Write_Motor_Enable();
    delay(5);
    jointMotors[i]->Read_Angle_MultiRound();          
    double lockAngle = jointMotors[i]->motor_angle;
    // jointMotors[i]->Write_Angle_MultiRound(lockAngle, JOINT_LOCK_SPEED); 

    Serial.print("  "); Serial.print(jointNames[i]);
    Serial.print(" 自鎖於角度: "); Serial.println(lockAngle);
    delay(5);
  }
  jointMotors[0]->Write_Angle_MultiRound(-45, JOINT_LOCK_SPEED); 
  jointMotors[1]->Write_Angle_MultiRound(70.2, JOINT_LOCK_SPEED);
  jointMotors[2]->Write_Angle_MultiRound(45, JOINT_LOCK_SPEED);  
  jointMotors[3]->Write_Angle_MultiRound(-70.2, JOINT_LOCK_SPEED); 
  jointsLocked = true;
  Serial.println(">>> 關節自鎖完成");
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


void enableWheels() {
  wheelLeft.Write_Motor_Enable();
  wheelRight.Write_Motor_Enable();
  balancePID.init(0.0);
  velPID.init(0.0);
  CurrentPID.init(0.0);   

  // 初始化所有濾波器的狀態，強制設定為當前角度與零速度，避免啟動瞬間輸出暴衝
  double currentPitch = imu.getData().angle[1]; 
  kalmanPitch.reset(currentPitch);     // 重置卡爾曼濾波器
  lowPassPitch.reset(currentPitch);    // 重置低通濾波器
  finalFilteredPitch = currentPitch;   

  speedFilterLeft.reset(0.0);          // 重置左輪速度濾波器
  speedFilterRight.reset(0.0);         // 重置右輪速度濾波器

  wheelsEnabled = true;
  Serial.println(">>> 輪子已啟動，開始平衡");
}

void disableWheels() {
  wheelLeft.Write_Motor_Disable();
  wheelRight.Write_Motor_Disable();
  wheelsEnabled = false;
  motorOutput = 0.0;
  targetangle = 0.0;
  Serial.println(">>> 輪子已關閉");
}

void PrintMotorStatus(LKMotor &motor, const char *name) {
  motor.Read_Motor_Status2();
  Serial.print(name);
  Serial.print(" 速度:"); Serial.print(motor.motor_dspeed);
  Serial.print(" 溫度:"); Serial.print(motor.motor_temperature);
  Serial.print(" 電流:"); Serial.println(motor.motor_current);
}