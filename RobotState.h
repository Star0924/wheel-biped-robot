#pragma once

enum class RobotState{
  JOINT_UNLOCKED,
  JOINT_LOCKED,
  BALANCING,
  FALLEN
};

inline const char* toString(RobotState s) {
    switch (s) {
        case RobotState::JOINT_UNLOCKED: return "解鎖";
        case RobotState::JOINT_LOCKED:   return "鎖定";
        case RobotState::BALANCING:      return "平衡中";
        case RobotState::FALLEN:         return "跌倒";
    }
    return "未知";
}