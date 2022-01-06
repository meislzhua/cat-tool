#pragma once
#include <ArduinoJson.h>
#include <EventRegister.h>
/**
 * @brief 事件注册分发类
 */
class CatEventClass {
    /**
     * @brief 已注册的时间列表
     */
    QueueHandle_t *regedit[REGISTER_MAX_NUM]{NULL};

   public:
    /**
     * @brief 注册事件队列
     * @param code 事件码,在<EventRegister.h>中定义
     * @param queue 事件队列
     */
    void registerEvent(int code, QueueHandle_t *queue) {
        this->regedit[code] = queue;
    }

    /**
     * @brief 触发事件
     * @param doc 触发数据JSON
     */
    void trigger(DynamicJsonDocument *doc) {
        int code = (*doc)["c"];

        if (code != 0 && this->regedit[code]) {  //发现对应注册事件
            // Serial.printf("发现注册:[%d] \n", doc);
            QueueHandle_t *queue = this->regedit[code];
            xQueueSend(*queue, &doc, 0);
        } else {  //无对应注册事件
            Serial.printf("无注册:%d\n", code);
            delete doc;
        }
    }
} CatEvent;
