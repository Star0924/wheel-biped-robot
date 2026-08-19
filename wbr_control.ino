#include <Arduino.h>
#include "Config.h"
#include "RobotState.h"
#include "JointController.h"
#include "BalanceController.h"
#include "CommandHandler.h"

JointController   joints;
BalanceController  balance;
CommandHandler     cmdHandler;
RobotState         state = RobotState::JOINT_UNLOCKED;

void setup() {
    Serial.begin(TERMINAL_BAUD);
    while (!Serial && millis() < 3000);

    joints.begin();
    balance.begin();
    delay(100);

    Serial.println("===== 6馬達雙輪足機器人 初始化 =====");
    joints.lockAll();
    state = RobotState::JOINT_LOCKED;

    Serial.println("===== 初始化完成 =====");
    Serial.println("指令：e=啟動  d=關閉  l=鎖定  u=解鎖  z=偏航歸零  x=XY歸零  c=校準");
}

void loop() {
    balance.update(state);
    cmdHandler.poll(state, joints, balance);

    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= PRINT_PERIOD_MS) {
        lastPrint = millis();
        const IMUData& d = balance.imuData();
        Serial.printf("Roll:%6.2f Pitch:%6.2f Yaw:%6.2f 溫度:%.1f 狀態:%s\n",
                      d.angle[0], d.angle[1], d.angle[2],
                      d.temperature, toString(state));
    }
}