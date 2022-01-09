#include <Arduino.h>
#include <ArduinoJson.h>
#include <config.h>

#include "esp_task_wdt.h"
#include "service/CatEvent.hpp"
#include "service/CatNetwork.hpp"
#include "service/CatTest.hpp"

#if defined(CAT_TOOL_ESP32_CAM)
#include "service/CatCam.hpp"
#endif

#if defined(CAT_TOOL_ESP32_MOTOR)
#include "service/CatMove.hpp"
#endif

Test tt;
void setup() {
    // put your setup code here, to run once:
    esp_task_wdt_deinit();
    esp_task_wdt_init(60, true);
    Serial.begin(115200);
    Serial.setDebugOutput(true);

    CatNetwork.init();
    tt.init();

#if defined(CAT_TOOL_ESP32_CAM)
    CatCam.init();
#endif

#if defined(CAT_TOOL_ESP32_MOTOR)
    CatMove.init();
#endif

    Serial.printf("初始化完成\r\n");
}

void loop() {
    // Serial.printf("?\r\n");
    // put your main code here, to run repeatedly:
}