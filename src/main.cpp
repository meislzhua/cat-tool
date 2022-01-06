#include <Arduino.h>
#include <ArduinoJson.h>

#include "service/CatCam.hpp"
#include "service/CatEvent.hpp"
#include "service/CatMove.hpp"
#include "service/CatNetwork.hpp"
#include "service/CatTest.hpp"
Test tt;
void setup() {
    // put your setup code here, to run once:

    Serial.begin(115200);
    Serial.setDebugOutput(true);

    CatNetwork.init();
    CatCam.init();
    CatMove.init();
    tt.init();

    Serial.printf("初始化完成\r\n");
}

void loop() {
    // put your main code here, to run repeatedly:
}