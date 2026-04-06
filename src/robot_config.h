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
 * Offset mécanique par canal (degrés), ajouté à l’angle logique **avant** miroir et PWM.
 * Positif = même sens qu’une augmentation d’angle logique sur ce servo.
 * À régler au banc pour aligner stand / pose « tous à 90° » ; la télémétrie reste **sans** trim.
 */
constexpr float SERVO_TRIM_DEG[8] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

/**
 * Impulsion min / max pour 0° et 180°.
 * Feetech (ex. FS190M / FT90M) : souvent 500–2500 µs ; certains micros ~900–2100 µs.
 */
constexpr uint16_t SERVO_US_MIN = 500;
constexpr uint16_t SERVO_US_MAX = 2500;

constexpr uint8_t LEDC_RES_BITS = 14;
constexpr uint32_t LEDC_FREQ_HZ = 50;

/**
 * Pose « stand » : angles logiques quand la patte est **tout droite verticalement**
 * (fémur + tibia alignés sous l’épaule, L1+L2 = 2×LEG_LINK_LENGTH_MM).
 * L’IK « pied sous l’épaule » part de ce calage ; affine au banc avec SERVO_TRIM_DEG.
 */
constexpr float NEUTRAL_SHOULDER_DEG = 90.f;
constexpr float NEUTRAL_KNEE_DEG = 45.f;

/** Longueur d’un maillon (fémur ou tibia), mm — schéma 13 cm / 13 cm. */
constexpr float LEG_LINK_LENGTH_MM = 130.f;

/** Hauteur épaule → sol le long de la verticale pied sous épaule (mm). Max = 2×L. */
constexpr float STAND_HEIGHT_MM = 2.f * LEG_LINK_LENGTH_MM;

/**
 * Hauteur cible stand bas (mm), par rangée : avant (FL/FR) et arrière (RL/RR).
 * Valeurs par défaut calées sur l’ancienne pose ~140°/150° avant et ~30°/160° arrière.
 */
constexpr float STAND_LOW_HEIGHT_FRONT_ROW_MM = 167.f;
constexpr float STAND_LOW_HEIGHT_REAR_ROW_MM = 130.f;
/**
 * Pose stand_low "gorille" en angles directs (FL..RR : épaule puis genou).
 * Issue de ton calage manuel `srv`.
 */
constexpr float STAND_LOW_GORILLA_DEG[8] = {
    150.f, 80.f,   // FL, FR (avant)
    150.f, 80.f,
    150.f, 0.f,    // RL, RR (arrière)
    150.f, 0.f
};

/** Hauteur minimale (mm) pour éviter acos instable ; reste sous la cible IK. */
constexpr float IK_HEIGHT_MIN_MM = 50.f;

/**
 * Gain genou : K = NEUTRAL_KNEE + gain × (180° − κ), avec κ angle intérieur au genou.
 * Pour une patte isocèle pied sous épaule : 180° − κ = 2δ (δ = écart fémur / verticale).
 * Deux gains si le montage avant / arrière n’est pas identique.
 */
constexpr float IK_KNEE_GAIN_FRONT = 1.05f;
constexpr float IK_KNEE_GAIN_REAR = 115.f / 120.f;

/**
 * Pose de base en marche (différente du stand IK) : posture basse / large, proche du schéma « gorille ».
 * Plus les genoux sont fléchis (valeurs plus élevées ici), plus le corps est près du sol.
 */
constexpr float WALK_BASE_SHOULDER_FRONT_DEG = 112.f;
constexpr float WALK_BASE_SHOULDER_REAR_DEG = 66.f;
constexpr float WALK_BASE_KNEE_FRONT_DEG = 105.f;
constexpr float WALK_BASE_KNEE_REAR_DEG = 108.f;

/** Oscillation autour de ces bases (trot diagonal). Réduire si les pattes se croisent encore. */
constexpr float WALK_SHOULDER_SWING_DEG = 14.f;
/** Réduction de l'amplitude sur la phase qui rapproche AV/ARR d'un même côté. */
constexpr float WALK_SHOULDER_INNER_SCALE = 0.35f;
/**
 * Butée dure sur l'offset d'épaule pendant cette phase intérieure (deg).
 * Plus petit => moins de risque que les jambes se touchent.
 */
constexpr float WALK_SHOULDER_INNER_MAX_DEG = 2.f;
/**
 * Relevé de genou en phase d’oscillation (trot : pattes en diagonale décollent un peu).
 * Plus petit = pieds restent plus près du sol pendant la marche.
 */
constexpr float WALK_KNEE_LIFT_DEG = 9.f;
/** En phase intérieure, le genou AR (s>0) relevait fort : facteur < 1 pour éviter le croisement. */
constexpr float WALK_KNEE_LIFT_INNER_SCALE = 0.2f;
constexpr uint32_t WALK_CYCLE_MS = 1200;//720;

/**
 * Marche `walk_gorille` : IK 2D (pied x,z en mm, épaule à l’origine, +x avant, +z bas).
 * Les angles logiques de `STAND_LOW_GORILLA_DEG` sont interprétés comme θ1/θ2 géométriques (deg)
 * pour le FK/IK (voir commentaire dans task_servos.cpp).
 * moveX / turnYaw : facteurs -1..1 sur les amplitudes ci‑dessous.
 */
constexpr float WALK_GORILLA_STRIDE_MM = 22.f;
constexpr float WALK_GORILLA_CLEARANCE_MM = 26.f;
constexpr float WALK_GORILLA_STANCE_SLIDE_MM = 18.f;
constexpr float WALK_GORILLA_TURN_STRIDE_MM = 10.f;
/** Gains par rangée pour corriger un avant trop passif. */
constexpr float WALK_GORILLA_FRONT_STRIDE_SCALE = 1.45f;
constexpr float WALK_GORILLA_FRONT_CLEARANCE_SCALE = 1.8f;
constexpr float WALK_GORILLA_FRONT_STANCE_SCALE = 1.35f;
/** Rend l'avant plus nerveux en swing/traction. */
constexpr float WALK_GORILLA_FRONT_LIFT_BOOST = 1.55f;
constexpr float WALK_GORILLA_FRONT_TRACTION_BOOST = 1.45f;
constexpr uint32_t WALK_GORILLA_CYCLE_MS = 1400;

/**
 * Durée de fusion entre deux poses prédéfinies (stand ↔ stand_low, etc.).
 * Les 8 servos interpolent ensemble (smoothstep) pour limiter les trajets bancals
 * où un axe arrive avant l’autre ; ajuster au ressenti (300–800 ms typique).
 */
constexpr uint32_t POSE_BLEND_DURATION_MS = 1200; // 480

}  // namespace RobotConfig
