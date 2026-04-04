#pragma once

#include <Arduino.h>

/**
 * Brochage PWM — GPIO 4/5 réservés à l’I2C (SDA/SCL).
 *
 * Ordre : FL hanche/genou, FR, RL, RR (indices 0..7).
 */
namespace RobotConfig {

constexpr uint8_t SERVO_PINS[8] = {12, 13, 14, 27, 26, 25, 33, 32};

/**
 * Impulsion min / max pour 0° et 180°.
 * Feetech (ex. FS190M / FT90M) : souvent 500–2500 µs ; certains micros ~900–2100 µs.
 * Si course trop courte ou bord bloqué, essaie 900 / 2100 à la place.
 */
constexpr uint16_t SERVO_US_MIN = 500;
constexpr uint16_t SERVO_US_MAX = 2500;

constexpr uint8_t LEDC_RES_BITS = 14;
constexpr uint32_t LEDC_FREQ_HZ = 50;

constexpr float NEUTRAL_HIP_DEG = 90.f;
constexpr float NEUTRAL_KNEE_DEG = 90.f;

constexpr float WALK_HIP_SWING_DEG = 22.f;
constexpr float WALK_KNEE_LIFT_DEG = 28.f;
constexpr uint32_t WALK_CYCLE_MS = 720;

}  // namespace RobotConfig
