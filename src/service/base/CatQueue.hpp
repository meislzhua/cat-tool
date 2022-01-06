#pragma once
#include <ArduinoJson.h>
#include <EventRegister.h>

#include "CatTask.hpp"

/**
 * @brief 消息对列基类,继承即可获得消息队列功能
 *
 */
class CatQueueBase : public CatTaskBase {
   protected:
    /**
     * @brief 获取事件ID
     *
     * @return uint8_t
     */
    virtual uint8_t getQueueId() = 0;

    /**
     * @brief 用于处理队列中的事件的函数
     *
     * @param doc 事件JSON对象
     */
    virtual void handleQueue(DynamicJsonDocument &doc) = 0;
    /**
     * @brief 消息队列
     *
     */
    QueueHandle_t queue = xQueueCreate(100, sizeof(void *));

    void handleTask() {
        DynamicJsonDocument *doc = NULL;
        if (xQueueReceive(this->queue, &doc, (TickType_t)10) == pdPASS) {
            this->handleQueue(*doc);
            delete doc;
        }
    }

   public:
    void init() {
        CatTaskBase::init();
        //在事件中心注册,当发生发生相应事件时,事件加入到队列
        CatEvent.registerEvent(this->getQueueId(), &this->queue);
    }
};
