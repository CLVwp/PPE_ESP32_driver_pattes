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

uint8_t logicalDegToPwmDeg(uint8_t channel, uint8_t logicalDeg) {
  if (SERVO_ANGLE_MIRROR[channel]) {
    return static_cast<uint8_t>(constrain(180 - logicalDeg, 0, 180));
  }
  return logicalDeg;
}

/** Angle logique utilisateur + trim, borné 0–180, puis miroir → PWM. */
uint8_t logicalToPwmOutput(uint8_t channel, uint8_t logicalDeg) {
  const float withTrim =
      static_cast<float>(logicalDeg) + SERVO_TRIM_DEG[channel];
  const uint8_t clamped =
      static_cast<uint8_t>(constrain(withTrim, 0.f, 180.f));
  return logicalDegToPwmDeg(channel, clamped);
}

/**
 * IK 2R L1=L2, pied sur la verticale de l’épaule (vue latérale patte).
 * δ = acos(H / (2L)) : écart du fémur par rapport à la verticale (vers l’extérieur rangée avant).
 * Rangée avant : épaule = neutre + δ ; arrière : neutre − δ (repère mécanique opposé).
 */
void computeVerticalFootLeg(int legIdx, float heightMm, uint8_t *shoulderDeg,
                            uint8_t *kneeDeg) {
  const float hMax = 2.f * LEG_LINK_LENGTH_MM;
  const float h = constrain(heightMm, IK_HEIGHT_MIN_MM, hMax);
  const float cosDelta = h / hMax;
  const float deltaRad = acosf(cosDelta);
  const float deltaDeg = deltaRad * (180.f / static_cast<float>(PI));
  const float kappaDeg = 180.f - 2.f * deltaDeg;

  const bool frontRow = legIdx < 2;
  const float sh =
      NEUTRAL_SHOULDER_DEG + (frontRow ? deltaDeg : -deltaDeg);
  const float kneeGain =
      frontRow ? IK_KNEE_GAIN_FRONT : IK_KNEE_GAIN_REAR;
  const float kn =
      NEUTRAL_KNEE_DEG + kneeGain * (180.f - kappaDeg);

  *shoulderDeg =
      static_cast<uint8_t>(constrain(sh + 0.5f, 0.f, 180.f));
  *kneeDeg = static_cast<uint8_t>(constrain(kn + 0.5f, 0.f, 180.f));
}

void computeStandAngles(uint8_t out[8]) {
  for (int leg = 0; leg < 4; leg++) {
    computeVerticalFootLeg(leg, STAND_HEIGHT_MM, &out[leg * 2],
                          &out[leg * 2 + 1]);
  }
}

void computeStandLowGroundAngles(uint8_t out[8]) {
  for (int leg = 0; leg < 4; leg++) {
    const float h = leg < 2 ? STAND_LOW_HEIGHT_FRONT_ROW_MM
                            : STAND_LOW_HEIGHT_REAR_ROW_MM;
    computeVerticalFootLeg(leg, h, &out[leg * 2], &out[leg * 2 + 1]);
  }
}

/** Inverse (approx.) de l’épaule IK : hauteur sous verticale depuis l’angle épaule logique. */
float estimateHeightMmFromShoulder(int legIdx, uint8_t shoulderDeg) {
  const float S = static_cast<float>(shoulderDeg);
  const bool frontRow = legIdx < 2;
  float deltaDeg =
      frontRow ? (S - NEUTRAL_SHOULDER_DEG) : (NEUTRAL_SHOULDER_DEG - S);
  deltaDeg = constrain(deltaDeg, 0.f, 89.f);
  const float deltaRad = deltaDeg * (static_cast<float>(PI) / 180.f);
  return 2.f * LEG_LINK_LENGTH_MM * cosf(deltaRad);
}

void initPwm() {
  uint8_t stand[8];
  computeStandAngles(stand);
  for (int ch = 0; ch < 8; ch++) {
    ledcSetup(ch, LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcAttachPin(SERVO_PINS[ch], ch);
    const uint8_t pwmDeg = logicalToPwmOutput(ch, stand[ch]);
    pwmWriteUs(ch, degToUs(static_cast<float>(pwmDeg)));
  }
}

void computeWalkAngles(float phase01, uint8_t out[8]) {
  const float p = phase01 * 2.f * PI;
  const float off[4] = {0.f, PI, PI, 0.f};

  for (int leg = 0; leg < 4; leg++) {
    const float s = sinf(p + off[leg]);
    const float shoulder =
        NEUTRAL_SHOULDER_DEG + WALK_SHOULDER_SWING_DEG * s;
    const float knee =
        NEUTRAL_KNEE_DEG + WALK_KNEE_LIFT_DEG * fmaxf(0.f, s);
    out[leg * 2] =
        static_cast<uint8_t>(constrain(shoulder, 0.f, 180.f));
    out[leg * 2 + 1] =
        static_cast<uint8_t>(constrain(knee, 0.f, 180.f));
  }
}

struct PoseBlendState {
  bool active = false;
  bool heightIkBlend = false;
  uint32_t t0_ms = 0;
  uint8_t from[8]{};
  uint8_t to[8]{};
  float hFrom[4]{};
  float hTo[4]{};
};

PoseBlendState gPoseBlend;

void cancelPoseBlend() {
  gPoseBlend.active = false;
  gPoseBlend.heightIkBlend = false;
}

/** Interpolation classique dans l’espace des 8 angles (ex. marche → stand). */
void startPoseBlend(uint8_t currentAngles[8], const uint8_t target[8]) {
  int maxDelta = 0;
  for (int i = 0; i < 8; i++) {
    const int d =
        static_cast<int>(currentAngles[i]) - static_cast<int>(target[i]);
    const int ad = d < 0 ? -d : d;
    if (ad > maxDelta) {
      maxDelta = ad;
    }
  }
  if (maxDelta < 2) {
    gPoseBlend.active = false;
    gPoseBlend.heightIkBlend = false;
    memcpy(currentAngles, target, sizeof(gPoseBlend.to));
    return;
  }
  gPoseBlend.heightIkBlend = false;
  memcpy(gPoseBlend.from, currentAngles, sizeof(gPoseBlend.from));
  memcpy(gPoseBlend.to, target, sizeof(gPoseBlend.to));
  gPoseBlend.t0_ms = millis();
  gPoseBlend.active = true;
}

/**
 * stand ↔ stand_low : interpole la hauteur H par patte puis recalcule l’IK à chaque pas
 * (le pied reste sur la verticale du modèle) ; pas une ligne droite dans l’espace articulaire.
 */
void startStandPoseHeightBlend(uint8_t currentAngles[8], bool destIsStandHigh) {
  uint8_t target[8];
  if (destIsStandHigh) {
    computeStandAngles(target);
  } else {
    computeStandLowGroundAngles(target);
  }

  int maxDelta = 0;
  for (int i = 0; i < 8; i++) {
    const int d =
        static_cast<int>(currentAngles[i]) - static_cast<int>(target[i]);
    const int ad = d < 0 ? -d : d;
    if (ad > maxDelta) {
      maxDelta = ad;
    }
  }
  if (maxDelta < 2) {
    gPoseBlend.active = false;
    gPoseBlend.heightIkBlend = false;
    memcpy(currentAngles, target, sizeof(target));
    return;
  }

  for (int leg = 0; leg < 4; leg++) {
    gPoseBlend.hFrom[leg] =
        estimateHeightMmFromShoulder(leg, currentAngles[leg * 2]);
    gPoseBlend.hTo[leg] =
        destIsStandHigh
            ? STAND_HEIGHT_MM
            : (leg < 2 ? STAND_LOW_HEIGHT_FRONT_ROW_MM
                       : STAND_LOW_HEIGHT_REAR_ROW_MM);
  }
  gPoseBlend.heightIkBlend = true;
  gPoseBlend.t0_ms = millis();
  gPoseBlend.active = true;
}

/** Si blend actif : écrit dans `out` l’interpolation ; retourne true si encore en cours. */
bool applyPoseBlend(uint8_t out[8]) {
  if (!gPoseBlend.active) {
    return false;
  }
  const uint32_t elapsed = millis() - gPoseBlend.t0_ms;
  float u = static_cast<float>(elapsed) /
            static_cast<float>(POSE_BLEND_DURATION_MS);
  if (u >= 1.f) {
    u = 1.f;
    gPoseBlend.active = false;
  }
  const float s = u * u * (3.f - 2.f * u);
  if (gPoseBlend.heightIkBlend) {
    for (int leg = 0; leg < 4; leg++) {
      const float h = gPoseBlend.hFrom[leg] +
                      s * (gPoseBlend.hTo[leg] - gPoseBlend.hFrom[leg]);
      computeVerticalFootLeg(leg, h, &out[leg * 2], &out[leg * 2 + 1]);
    }
    if (!gPoseBlend.active) {
      gPoseBlend.heightIkBlend = false;
    }
  } else {
    for (int i = 0; i < 8; i++) {
      const float v = static_cast<float>(gPoseBlend.from[i]) +
                      s * (static_cast<float>(gPoseBlend.to[i]) -
                           static_cast<float>(gPoseBlend.from[i]));
      out[i] = static_cast<uint8_t>(constrain(v + 0.5f, 0.f, 180.f));
    }
  }
  return gPoseBlend.active;
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
  TickType_t lastTick = lastWake;
  const TickType_t period = pdMS_TO_TICKS(20);

  for (;;) {
    RobotCommandMsg cmd;
    while (gQueueRobotCmd != nullptr &&
           xQueueReceive(gQueueRobotCmd, &cmd, 0) == pdTRUE) {
      switch (cmd.kind) {
        case RobotCmdKind::ModeStand: {
          const bool fromStandish =
              (mode == LegCtrlMode::Stand ||
               mode == LegCtrlMode::StandLowGround);
          mode = LegCtrlMode::Stand;
          uint8_t tgt[8];
          computeStandAngles(tgt);
          if (fromStandish) {
            startStandPoseHeightBlend(angles, true);
          } else {
            startPoseBlend(angles, tgt);
          }
          memcpy(poseAngles, angles, sizeof(angles));
          break;
        }
        case RobotCmdKind::ModeStandLowGround: {
          const bool fromStandish =
              (mode == LegCtrlMode::Stand ||
               mode == LegCtrlMode::StandLowGround);
          mode = LegCtrlMode::StandLowGround;
          uint8_t tgt[8];
          computeStandLowGroundAngles(tgt);
          if (fromStandish) {
            startStandPoseHeightBlend(angles, false);
          } else {
            startPoseBlend(angles, tgt);
          }
          memcpy(poseAngles, angles, sizeof(angles));
          break;
        }
        case RobotCmdKind::ModeWalk:
          cancelPoseBlend();
          mode = LegCtrlMode::Walk;
          if (cmd.value > 0.05f && cmd.value <= 1.f) {
            walkSpeed = cmd.value;
          }
          break;
        case RobotCmdKind::SetWalkSpeed:
          walkSpeed = constrain(cmd.value, 0.1f, 1.f);
          break;
        case RobotCmdKind::ServoAngle:
          cancelPoseBlend();
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
        static_cast<float>(pdTICKS_TO_MS(now - lastTick));
    lastTick = now;

    switch (mode) {
      case LegCtrlMode::Stand: {
        uint8_t nominal[8];
        computeStandAngles(nominal);
        if (!applyPoseBlend(angles)) {
          memcpy(angles, nominal, sizeof(angles));
        }
        memcpy(poseAngles, angles, sizeof(poseAngles));
        break;
      }
      case LegCtrlMode::StandLowGround: {
        uint8_t nominal[8];
        computeStandLowGroundAngles(nominal);
        if (!applyPoseBlend(angles)) {
          memcpy(angles, nominal, sizeof(angles));
        }
        memcpy(poseAngles, angles, sizeof(poseAngles));
        break;
      }
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
      const uint8_t pwmDeg = logicalToPwmOutput(ch, angles[ch]);
      pwmWriteUs(ch, degToUs(static_cast<float>(pwmDeg)));
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
