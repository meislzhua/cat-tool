
#pragma once
#include <ArduinoJson.h>
#include <EventRegister.h>

#include "base/CatQueue.hpp"

class Test : public CatQueueBase {
    void handleQueue(DynamicJsonDocument &doc) {
        String json;
        serializeJson(doc, json);
        Serial.printf("接受到数据包:[%s] \n\n", json.c_str());
        Serial.printf("执行核心:[%d] \n\n", xPortGetCoreID());
        Serial.println(json.c_str());
    }

   public:
    uint8_t getQueueId() {
        return REGISTER_CATNETWORK_PING;
    }
};
