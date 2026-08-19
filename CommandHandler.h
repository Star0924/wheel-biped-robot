// CommandHandler.h
#pragma once
#include "JointController.h"
#include "BalanceController.h"
#include "RobotState.h"

class CommandHandler {
public:
    void poll(RobotState &state, JointController &joints, BalanceController &balance);
};