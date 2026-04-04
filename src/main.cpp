#include <Arduino.h>

#include "robot_state.h"
#include "sensors_shared.h"
#include "task_sensors.h"
#include "task_json.h"
#include "task_servos.h"

void setup() {
  Serial.begin(115200);

  robotStateInit();

  // Initialisation du matériel capteurs (I2C + IMU + BMP280)
  sensorsInit();

  // Création des tâches (comme pour la manette RTOS)
  xTaskCreatePinnedToCore(taskSensors, "Sensors", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskJson,    "JSON",    8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskServos,  "Servos",  6144, NULL, 2, NULL, 1);

  Serial.println(F("RTOS: tâches capteurs + JSON + servos"));
}

void loop() {
  // Rien ici, tout se fait dans les tâches
  vTaskDelay(pdMS_TO_TICKS(1000));
}