#pragma once

#define IMU_SERIAL   Serial2
#define IMU_BAUD     921600
#define TERMINAL_BAUD 115200

struct MotorConfig{
  uint8_t id;
  uint16_t reduction;
  uint8_t bus;
};

static const double BALANCE_KP = 40.0;
static const double BALANCE_KI = 0.0;
static const double BALANCE_KD = 0.0;
static const double TARGET_ANGLE_DEG = 0.0;
static const double FALL_LIMIT_DEG   = 40.0;
static const long   JOINT_LOCK_SPEED = 20;

static const MotorConfig WHEEL_LEFT_CFG  = {4, 8, 5};
static const MotorConfig WHEEL_RIGHT_CFG = {1, 8, 5};

static const MotorConfig HIP_LEFT_CFG   = {6, 10, 5};
static const MotorConfig KNEE_LEFT_CFG  = {5, 10, 5};
static const MotorConfig HIP_RIGHT_CFG  = {3, 10, 5};
static const MotorConfig KNEE_RIGHT_CFG = {2, 10, 5};

#define WHEEL_LEFT_SIGN  (+1.0)
#define WHEEL_RIGHT_SIGN (-1.0)

static const uint32_t CONTROL_PERIOD_MS = 10;   // 100Hz
static const uint32_t PRINT_PERIOD_MS   = 100;  // 10Hz


