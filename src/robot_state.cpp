#include "robot_state.h"

QueueHandle_t gQueueRobotCmd = nullptr;
RobotRuntimeState gRobotRuntime;
SemaphoreHandle_t mutexRobotRuntime = nullptr;

void robotStateInit() {
  gQueueRobotCmd = xQueueCreate(16, sizeof(RobotCommandMsg));

  mutexRobotRuntime = xSemaphoreCreateMutex();
  gRobotRuntime.mode = LegCtrlMode::Stand;
  gRobotRuntime.gaitPhase01 = 0.f;
  gRobotRuntime.walkSpeed = 0.55f;
  for (int i = 0; i < 8; i++) {
    gRobotRuntime.anglesDeg[i] = 90;
  }
}
