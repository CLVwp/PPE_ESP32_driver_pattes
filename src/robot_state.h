#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

enum class LegCtrlMode : uint8_t {
  Stand,
  StandLowGround,
  StandLowGorilla,
  Walk,
  WalkGorilla,
  Pose
};

enum class RobotCmdKind : uint8_t {
  None,
  ModeStand,
  ModeStandLowGround,
  ModeStandLowGorilla,
  ModeWalk,
  ModeWalkGorilla,
  SetWalkSpeed,
  SetWalkMotion,
  ServoAngle,
};

struct RobotCommandMsg {
  RobotCmdKind kind;
  float value;
  float moveX;
  float turnYaw;
  uint8_t servoIdx;
  uint8_t angleDeg;
};

struct RobotRuntimeState {
  LegCtrlMode mode;
  float gaitPhase01;
  float walkSpeed;
  float walkMoveX;
  float walkTurnYaw;
  uint8_t anglesDeg[8];
};

void robotStateInit();

extern QueueHandle_t gQueueRobotCmd;
extern RobotRuntimeState gRobotRuntime;
extern SemaphoreHandle_t mutexRobotRuntime;
