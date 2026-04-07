#include <Arduino.h>
#include <ArduinoJson.h>
#include <cctype>
#include <cstring>

#include "robot_state.h"
#include "sensors_shared.h"
#include "task_json.h"

/*
 * Protocole série (USB ↔ Raspberry Pi), une ligne JSON = un message (NDJSON).
 *
 * ESP32 → Pi  (télémétrie) :
 *   {"t":"tel","seq":N,"imu":{...},"robot":{...}}
 *
 * Pi → ESP32  (commandes) :
 *   {"t":"cmd","m":"stand"}
 *   {"t":"cmd","m":"stand_low"}   
 * stand bas IK (hauteurs dans robot_config.h)
 *   {"t":"cmd","m":"stand_low_gorille"} 
 *  stand bas "gorille" (angles directs)
 *   {"t":"cmd","m":"walk","v":0.7}     
 * v optionnel, 0.1–1.0 = vitesse relative
 *   {"t":"cmd","m":"walk_gorille","v":0.45,"x":1,"yaw":0}
 *   {"t":"cmd","m":"motion","x":-1,"yaw":0.3} 
 * ajuste direction à chaud
 *   {"t":"cmd","m":"speed","v":0.5}    
 * en mode marche
 *   {"t":"srv","i":0,"a":90}           
 * servo i (0–7), angle °, passe en mode pose
 */

/** À remettre à true pour réafficher la télémétrie JSON sur le Serial. */
static constexpr bool kSerialTelemetryOut = true;

static void serialAckOk(const char *m) {
  if (kSerialTelemetryOut) {
    return;
  }
  Serial.print(F("{\"t\":\"ack\",\"m\":\""));
  Serial.print(m);
  Serial.println(F("\"}"));
  Serial.flush();
}

static void serialAckJsonErr() {
  if (kSerialTelemetryOut) {
    return;
  }
  Serial.println(F("{\"t\":\"ack\",\"err\":\"json\"}"));
  Serial.flush();
}

static void serialAckIgnored() {
  if (kSerialTelemetryOut) {
    return;
  }
  Serial.println(F("{\"t\":\"ack\",\"err\":\"ignored\"}"));
  Serial.flush();
}

namespace {

constexpr size_t kSerialLineMax = 384;
char sLineBuf[kSerialLineMax];
size_t sLineLen = 0;
uint32_t sTelSeq = 0;

static const char *modeToStr(LegCtrlMode m) {
  switch (m) {
    case LegCtrlMode::Stand:
      return "stand";
    case LegCtrlMode::StandLowGround:
      return "stand_low";
    case LegCtrlMode::StandLowGorilla:
      return "stand_low_gorille";
    case LegCtrlMode::Walk:
      return "walk";
    case LegCtrlMode::WalkGorilla:
      return "walk_gorille";
    case LegCtrlMode::Pose:
      return "pose";
    default:
      return "stand";
  }
}

void trimLine(char *line) {
  char *start = line;
  while (*start != '\0' && isspace(static_cast<unsigned char>(*start))) {
    ++start;
  }
  if (start != line) {
    memmove(line, start, strlen(start) + 1);
  }
  size_t n = strlen(line);
  while (n > 0 && isspace(static_cast<unsigned char>(line[n - 1]))) {
    line[--n] = '\0';
  }
}

void handleOneLine(char *line) {
  trimLine(line);
  if (line[0] != '{') {
    return;
  }

  JsonDocument doc;
  if (deserializeJson(doc, line)) {
    serialAckJsonErr();
    return;
  }

  const char *t = doc["t"];
  if (!t) {
    serialAckIgnored();
    return;
  }

  if (strcmp(t, "srv") == 0) {
    RobotCommandMsg msg = {};
    msg.kind = RobotCmdKind::ServoAngle;
    msg.servoIdx = static_cast<uint8_t>(doc["i"] | 255);
    {
      const int a = doc["a"] | 90;
      msg.angleDeg = static_cast<uint8_t>(constrain(a, 0, 180));
    }
    if (msg.servoIdx < 8 && gQueueRobotCmd != nullptr &&
        xQueueSend(gQueueRobotCmd, &msg, 0) == pdTRUE) {
      serialAckOk("srv");
    } else {
      serialAckIgnored();
    }
    return;
  }

  if (strcmp(t, "cmd") != 0) {
    serialAckIgnored();
    return;
  }

  const char *m = doc["m"];
  if (!m || gQueueRobotCmd == nullptr) {
    serialAckIgnored();
    return;
  }

  RobotCommandMsg msg = {};
  if (strcmp(m, "stand") == 0) {
    msg.kind = RobotCmdKind::ModeStand;
  } else if (strcmp(m, "stand_low") == 0) {
    msg.kind = RobotCmdKind::ModeStandLowGround;
  } else if (strcmp(m, "stand_low_gorille") == 0 ||
             strcmp(m, "stand_low_gorilla") == 0) {
    msg.kind = RobotCmdKind::ModeStandLowGorilla;
  } else if (strcmp(m, "walk") == 0) {
    msg.kind = RobotCmdKind::ModeWalk;
    msg.value = doc["v"] | 0.55f;
    msg.moveX = constrain(doc["x"] | 1.f, -1.f, 1.f);
    msg.turnYaw = constrain(doc["yaw"] | 0.f, -1.f, 1.f);
  } else if (strcmp(m, "walk_gorille") == 0 ||
             strcmp(m, "walk_gorilla") == 0) {
    msg.kind = RobotCmdKind::ModeWalkGorilla;
    msg.value = doc["v"] | 0.45f;
    msg.moveX = constrain(doc["x"] | 1.f, -1.f, 1.f);
    msg.turnYaw = constrain(doc["yaw"] | 0.f, -1.f, 1.f);
  } else if (strcmp(m, "speed") == 0) {
    msg.kind = RobotCmdKind::SetWalkSpeed;
    msg.value = doc["v"] | 0.5f;
  } else if (strcmp(m, "motion") == 0) {
    msg.kind = RobotCmdKind::SetWalkMotion;
    msg.moveX = constrain(doc["x"] | 1.f, -1.f, 1.f);
    msg.turnYaw = constrain(doc["yaw"] | 0.f, -1.f, 1.f);
  } else {
    serialAckIgnored();
    return;
  }

  if (xQueueSend(gQueueRobotCmd, &msg, 0) == pdTRUE) {
    serialAckOk(m);
  } else {
    serialAckIgnored();
  }
}

void pumpSerialCommands() {
  while (Serial.available() > 0) {
    const int raw = Serial.read();
    if (raw < 0) {
      break;
    }
    const char c = static_cast<char>(raw);
    if (c == '\n' || c == '\r') {
      if (sLineLen > 0) {
        sLineBuf[sLineLen] = '\0';
        handleOneLine(sLineBuf);
        sLineLen = 0;
      }
    } else if (sLineLen < kSerialLineMax - 1) {
      sLineBuf[sLineLen++] = c;
    } else {
      sLineLen = 0;
    }
  }
}

}  // namespace

void taskJson(void *pvParameters) {
  (void)pvParameters;

  const TickType_t period = pdMS_TO_TICKS(50);
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    pumpSerialCommands();

    if (kSerialTelemetryOut) {
      SensorsData snap{};
      if (mutexSensors != nullptr &&
          xSemaphoreTake(mutexSensors, pdMS_TO_TICKS(5)) == pdTRUE) {
        snap = gSensors;
        xSemaphoreGive(mutexSensors);
      }

      RobotRuntimeState bot{};
      if (mutexRobotRuntime != nullptr &&
          xSemaphoreTake(mutexRobotRuntime, pdMS_TO_TICKS(5)) == pdTRUE) {
        bot = gRobotRuntime;
        xSemaphoreGive(mutexRobotRuntime);
      }

      JsonDocument doc;

      doc["t"] = "tel";
      doc["seq"] = sTelSeq++;

      JsonObject imu = doc["imu"].to<JsonObject>();
      JsonArray acc = imu["acc"].to<JsonArray>();
      acc.add(snap.acc[0]);
      acc.add(snap.acc[1]);
      acc.add(snap.acc[2]);
      JsonArray gyr = imu["gyro"].to<JsonArray>();
      gyr.add(snap.gyro[0]);
      gyr.add(snap.gyro[1]);
      gyr.add(snap.gyro[2]);
      JsonArray mag = imu["mag"].to<JsonArray>();
      mag.add(snap.mag[0]);
      mag.add(snap.mag[1]);
      mag.add(snap.mag[2]);
      JsonObject baro = imu["baro"].to<JsonObject>();
      baro["lps_p"] = snap.lpsPressure;
      baro["lps_alt"] = snap.lpsAlt;
      baro["bmp_temp"] = snap.bmpTemp;
      baro["bmp_p"] = snap.bmpPressure;
      baro["bmp_alt"] = snap.bmpAlt;

      JsonObject robot = doc["robot"].to<JsonObject>();
      robot["mode"] = modeToStr(bot.mode);
      robot["phase"] = bot.gaitPhase01;
      robot["speed"] = bot.walkSpeed;
      JsonArray srv = robot["srv"].to<JsonArray>();
      for (int i = 0; i < 8; i++) {
        srv.add(bot.anglesDeg[i]);
      }

      serializeJson(doc, Serial);
      Serial.println();
      Serial.flush();
    }

    vTaskDelayUntil(&lastWake, period);
  }
}
