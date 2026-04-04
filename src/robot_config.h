#pragma once

#include <Arduino.h>

/**
 * Brochage et repère logique des 8 servos.
 * GPIO 4/5 réservés à l’I2C (SDA/SCL).
 *
 * Indices 0–7 : FL, FR, RL, RR — pour chaque patte : épaule puis genou.
 * (Breaking change par rapport à l’ancien ordre mélangé d’indices.)
 */
namespace RobotConfig {

/** Ordre : FL épaule, FL genou, FR épaule, FR genou, RL…, RR… */
constexpr uint8_t SERVO_PINS[8] = {33, 25, 26, 32, 13, 12, 14, 27};

/**
 * Miroir mécanique côté droit (FR, RR) : angle_PWM = 180° − angle_logique.
 * Neutre 90° inchangé.
 */
constexpr bool SERVO_ANGLE_MIRROR[8] = {
    false, false,  // FL
    true,  true,   // FR
    false, false,  // RL
    true,  true    // RR
};

/**
 * Impulsion min / max pour 0° et 180°.
 * Feetech (ex. FS190M / FT90M) : souvent 500–2500 µs ; certains micros ~900–2100 µs.
 */
constexpr uint16_t SERVO_US_MIN = 500;
constexpr uint16_t SERVO_US_MAX = 2500;

constexpr uint8_t LEDC_RES_BITS = 14;
constexpr uint32_t LEDC_FREQ_HZ = 50;

/**
 * Pose « stand » : point de départ (repère dessin épaule / genou).
 * À affiner au banc pour stabilité — voir README.
 */
constexpr float NEUTRAL_SHOULDER_DEG = 90.f;
constexpr float NEUTRAL_KNEE_DEG = 45.f;

/** Oscillation de marche autour des neutres ci-dessus (trot diagonal). */
constexpr float WALK_SHOULDER_SWING_DEG = 22.f;
constexpr float WALK_KNEE_LIFT_DEG = 28.f;
constexpr uint32_t WALK_CYCLE_MS = 720;

}  // namespace RobotConfig
