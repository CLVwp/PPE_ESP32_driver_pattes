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

void computeStandLowGorillaAngles(uint8_t out[8]) {
  for (int i = 0; i < 8; i++) {
    out[i] = static_cast<uint8_t>(constrain(STAND_LOW_GORILLA_DEG[i], 0.f, 180.f));
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
  computeStandLowGroundAngles(stand);
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
    float shoulderSwing = WALK_SHOULDER_SWING_DEG * s;
    const bool frontRow = leg < 2;
    const bool inwardPhase =
        (frontRow && shoulderSwing < 0.f) || (!frontRow && shoulderSwing > 0.f);
    if (inwardPhase) {
      shoulderSwing *= WALK_SHOULDER_INNER_SCALE;
      shoulderSwing =
          constrain(shoulderSwing, -WALK_SHOULDER_INNER_MAX_DEG,
                    WALK_SHOULDER_INNER_MAX_DEG);
    }
    const float shBase =
        frontRow ? WALK_BASE_SHOULDER_FRONT_DEG : WALK_BASE_SHOULDER_REAR_DEG;
    const float knBase =
        frontRow ? WALK_BASE_KNEE_FRONT_DEG : WALK_BASE_KNEE_REAR_DEG;
    const float shoulder = shBase + shoulderSwing;
    float kneeLift = WALK_KNEE_LIFT_DEG * fmaxf(0.f, s);
    if (inwardPhase && !frontRow) {
      kneeLift *= WALK_KNEE_LIFT_INNER_SCALE;
    }
    const float knee = knBase + kneeLift;
    out[leg * 2] =
        static_cast<uint8_t>(constrain(shoulder, 0.f, 180.f));
    out[leg * 2 + 1] =
        static_cast<uint8_t>(constrain(knee, 0.f, 180.f));
  }
}

/**
 * FK 2R : θ1 = angle fémur depuis +x (rad), θ2 = angle tibia relatif au fémur (rad).
 * Convention alignée sur les angles logiques : on pose θ1_deg = S, θ2_deg = K
 * (calage nominal via STAND_LOW_GORILLA_DEG ; si ton méca diffère, ajuste les trims).
 */
void fkGorillaFootMm(float shoulderDeg, float kneeDeg, float *xMm, float *zMm) {
  const float L1 = LEG_LINK_LENGTH_MM;
  const float L2 = LEG_LINK_LENGTH_MM;
  const float t1 = shoulderDeg * (static_cast<float>(PI) / 180.f);
  const float t2 = kneeDeg * (static_cast<float>(PI) / 180.f);
  *xMm = L1 * cosf(t1) + L2 * cosf(t1 + t2);
  *zMm = L1 * sinf(t1) + L2 * sinf(t1 + t2);
}

bool ikGorillaFootMm(float xMm, float zMm, float *shoulderDeg, float *kneeDeg) {
  const float L1 = LEG_LINK_LENGTH_MM;
  const float L2 = LEG_LINK_LENGTH_MM;
  const float r2 = xMm * xMm + zMm * zMm;
  const float r = sqrtf(r2);
  const float rMax = L1 + L2 - 0.5f;
  float x = xMm;
  float z = zMm;
  if (r > rMax) {
    const float s = rMax / (r + 1e-6f);
    x *= s;
    z *= s;
  }
  const float c2 = constrain(
      (x * x + z * z - L1 * L1 - L2 * L2) / (2.f * L1 * L2), -1.f, 1.f);
  const float s2 = sqrtf(fmaxf(0.f, 1.f - c2 * c2));
  const float th2 = atan2f(s2, c2);
  const float phi = atan2f(z, x);
  const float psi =
      atan2f(L2 * sinf(th2), L1 + L2 * cosf(th2));
  const float th1 = phi - psi;
  *shoulderDeg = th1 * (180.f / static_cast<float>(PI));
  *kneeDeg = th2 * (180.f / static_cast<float>(PI));
  return true;
}

void computeWalkGorillaAngles(float phase01, float moveX, float turnYaw,
                              uint8_t out[8]) {
  /**
   * Trot diagonal : FL+RR vs FR+RL. Pied (x,z) en mm depuis l’épaule.
   * Stance : glisse vers −x (poussée). Swing : remonte en z et avance en x.
   */
  const float p = phase01 * 2.f * PI;
  const float off[4] = {0.f, PI, PI, 0.f};
  const float mx = constrain(moveX, -1.f, 1.f);
  const float yaw = constrain(turnYaw, -1.f, 1.f);

  for (int leg = 0; leg < 4; leg++) {
    const float s = sinf(p + off[leg]);
    const float lift = fmaxf(0.f, s);
    const float stance = fmaxf(0.f, -s);
    const float lift2 = lift * lift;
    const float stance2 = stance * stance;
    const bool isLeft = (leg == 0 || leg == 2);

    const float S0 = STAND_LOW_GORILLA_DEG[leg * 2];
    const float K0 = STAND_LOW_GORILLA_DEG[leg * 2 + 1];
    float x0 = 0.f;
    float z0 = 0.f;
    fkGorillaFootMm(S0, K0, &x0, &z0);
    float Sn = 0.f;
    float Kn = 0.f;
    ikGorillaFootMm(x0, z0, &Sn, &Kn);
    const float offS = S0 - Sn;
    const float offK = K0 - Kn;

    const float dxStance = -mx * WALK_GORILLA_STANCE_SLIDE_MM * stance2;
    const float dxSwing = mx * WALK_GORILLA_STRIDE_MM * lift2;
    const float dzSwing = -WALK_GORILLA_CLEARANCE_MM * lift2;
    const float dxTurn =
        yaw * WALK_GORILLA_TURN_STRIDE_MM * (isLeft ? 1.f : -1.f) * (lift2 + stance2);

    const float xt = x0 + dxStance + dxSwing + dxTurn;
    const float zt = z0 + dzSwing;

    float S = S0;
    float K = K0;
    ikGorillaFootMm(xt, zt, &S, &K);
    S = S + offS;
    K = K + offK;

    out[leg * 2] = static_cast<uint8_t>(constrain(S + 0.5f, 0.f, 180.f));
    out[leg * 2 + 1] = static_cast<uint8_t>(constrain(K + 0.5f, 0.f, 180.f));
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

  LegCtrlMode mode = LegCtrlMode::StandLowGround;
  float phase01 = 0.f;
  float walkSpeed = gRobotRuntime.walkSpeed;
  float walkMoveX = gRobotRuntime.walkMoveX;
  float walkTurnYaw = gRobotRuntime.walkTurnYaw;
  uint8_t angles[8];
  uint8_t poseAngles[8];
  computeStandLowGroundAngles(poseAngles);
  computeStandLowGroundAngles(angles);

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
        case RobotCmdKind::ModeStandLowGorilla: {
          mode = LegCtrlMode::StandLowGorilla;
          uint8_t tgt[8];
          computeStandLowGorillaAngles(tgt);
          startPoseBlend(angles, tgt);
          memcpy(poseAngles, angles, sizeof(angles));
          break;
        }
        case RobotCmdKind::ModeWalk:
          cancelPoseBlend();
          mode = LegCtrlMode::Walk;
          phase01 = 0.f;
          if (cmd.value > 0.05f && cmd.value <= 1.f) {
            walkSpeed = cmd.value;
          }
          walkMoveX = cmd.moveX;
          walkTurnYaw = cmd.turnYaw;
          break;
        case RobotCmdKind::ModeWalkGorilla:
          cancelPoseBlend();
          mode = LegCtrlMode::WalkGorilla;
          phase01 = 0.f;
          if (cmd.value > 0.05f && cmd.value <= 1.f) {
            walkSpeed = cmd.value;
          }
          walkMoveX = cmd.moveX;
          walkTurnYaw = cmd.turnYaw;
          break;
        case RobotCmdKind::SetWalkSpeed:
          walkSpeed = constrain(cmd.value, 0.1f, 1.f);
          break;
        case RobotCmdKind::SetWalkMotion:
          walkMoveX = cmd.moveX;
          walkTurnYaw = cmd.turnYaw;
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
      case LegCtrlMode::StandLowGorilla: {
        uint8_t nominal[8];
        computeStandLowGorillaAngles(nominal);
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
      case LegCtrlMode::WalkGorilla: {
        const float step =
            (dtMs / static_cast<float>(WALK_GORILLA_CYCLE_MS)) * walkSpeed;
        phase01 += step;
        if (phase01 >= 1.f) {
          phase01 -= 1.f;
        }
        computeWalkGorillaAngles(phase01, walkMoveX, walkTurnYaw, angles);
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
      gRobotRuntime.walkMoveX = walkMoveX;
      gRobotRuntime.walkTurnYaw = walkTurnYaw;
      memcpy(gRobotRuntime.anglesDeg, angles, sizeof(angles));
      xSemaphoreGive(mutexRobotRuntime);
    }

    vTaskDelayUntil(&lastWake, period);
  }
}
