#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "robot_config.h"
#include "robot_state.h"
#include "task_servos.h"

namespace {

using namespace RobotConfig;

uint16_t degToUs(float deg) {
  deg = constrain(deg, 0.f, 180.f);
  return static_cast<uint16_t>(
      SERVO_US_MIN +
      (SERVO_US_MAX - SERVO_US_MIN) * (deg / 180.f));
}

void pwmWriteUs(uint8_t channel, uint16_t us) {
  us = constrain(us, SERVO_US_MIN, SERVO_US_MAX);
  const uint32_t maxDuty = (1u << LEDC_RES_BITS) - 1u;
  const uint32_t duty =
      static_cast<uint32_t>((uint64_t)us * maxDuty / 20000u);
  ledcWrite(channel, duty);
}

void initPwm() {
  for (int ch = 0; ch < 8; ch++) {
    ledcSetup(ch, LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcAttachPin(SERVO_PINS[ch], ch);
    pwmWriteUs(ch, degToUs(90.f));
  }
}

void computeStandAngles(uint8_t out[8]) {
  for (int i = 0; i < 8; i++) {
    const float deg =
        (i % 2 == 0) ? NEUTRAL_HIP_DEG : NEUTRAL_KNEE_DEG;
    out[i] = static_cast<uint8_t>(constrain(deg, 0.f, 180.f));
  }
}

void computeWalkAngles(float phase01, uint8_t out[8]) {
  const float p = phase01 * 2.f * PI;
  const float off[4] = {0.f, PI, PI, 0.f};

  for (int leg = 0; leg < 4; leg++) {
    const float s = sinf(p + off[leg]);
    const float hip = NEUTRAL_HIP_DEG + WALK_HIP_SWING_DEG * s;
    const float knee =
        NEUTRAL_KNEE_DEG + WALK_KNEE_LIFT_DEG * fmaxf(0.f, s);
    out[leg * 2] = static_cast<uint8_t>(constrain(hip, 0.f, 180.f));
    out[leg * 2 + 1] = static_cast<uint8_t>(constrain(knee, 0.f, 180.f));
  }
}

}  // namespace

void taskServos(void *pvParameters) {
  initPwm();

  LegCtrlMode mode = LegCtrlMode::Stand;
  float phase01 = 0.f;
  float walkSpeed = gRobotRuntime.walkSpeed;
  uint8_t angles[8];
  uint8_t poseAngles[8];
  computeStandAngles(poseAngles);
  computeStandAngles(angles);

  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(20);

  for (;;) {
    RobotCommandMsg cmd;
    while (gQueueRobotCmd != nullptr &&
           xQueueReceive(gQueueRobotCmd, &cmd, 0) == pdTRUE) {
      switch (cmd.kind) {
        case RobotCmdKind::ModeStand:
          mode = LegCtrlMode::Stand;
          computeStandAngles(poseAngles);
          memcpy(angles, poseAngles, sizeof(angles));
          break;
        case RobotCmdKind::ModeWalk:
          mode = LegCtrlMode::Walk;
          if (cmd.value > 0.05f && cmd.value <= 1.f) {
            walkSpeed = cmd.value;
          }
          break;
        case RobotCmdKind::SetWalkSpeed:
          walkSpeed = constrain(cmd.value, 0.1f, 1.f);
          break;
        case RobotCmdKind::ServoAngle:
          mode = LegCtrlMode::Pose;
          if (cmd.servoIdx < 8) {
            poseAngles[cmd.servoIdx] =
                static_cast<uint8_t>(constrain(cmd.angleDeg, 0, 180));
            memcpy(angles, poseAngles, sizeof(angles));
          }
          break;
        default:
          break;
      }
    }

    const TickType_t now = xTaskGetTickCount();
    const float dtMs =
        static_cast<float>(pdTICKS_TO_MS(now - lastWake));
    lastWake = now;

    switch (mode) {
      case LegCtrlMode::Stand:
        computeStandAngles(angles);
        memcpy(poseAngles, angles, sizeof(poseAngles));
        break;
      case LegCtrlMode::Walk: {
        const float step =
            (dtMs / static_cast<float>(WALK_CYCLE_MS)) * walkSpeed;
        phase01 += step;
        if (phase01 >= 1.f) {
          phase01 -= 1.f;
        }
        computeWalkAngles(phase01, angles);
        memcpy(poseAngles, angles, sizeof(poseAngles));
        break;
      }
      case LegCtrlMode::Pose:
        memcpy(angles, poseAngles, sizeof(angles));
        break;
    }

    for (int ch = 0; ch < 8; ch++) {
      pwmWriteUs(ch, degToUs(static_cast<float>(angles[ch])));
    }

    if (mutexRobotRuntime != nullptr &&
        xSemaphoreTake(mutexRobotRuntime, pdMS_TO_TICKS(2)) == pdTRUE) {
      gRobotRuntime.mode = mode;
      gRobotRuntime.gaitPhase01 = phase01;
      gRobotRuntime.walkSpeed = walkSpeed;
      memcpy(gRobotRuntime.anglesDeg, angles, sizeof(angles));
      xSemaphoreGive(mutexRobotRuntime);
    }

    vTaskDelayUntil(&lastWake, period);
  }
}
