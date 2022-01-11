#pragma once
#include <ArduinoJson.h>
#include <EventRegister.h>
#include <Ticker.h>

#include "CatTask.hpp"

#define QUEQU_FINISH_STOP_DELAY (20 * 1000)
#define QUEQU_FINISH_STOP_DELAY_HALF (QUEQU_FINISH_STOP_DELAY / 2)

// todo 修改cat依赖
/**
 * @brief 消息对列基类,继承即可获得消息队列功能
 *
 */
class CatQueue : public CatTask {
   protected:
    Ticker stopTaskTicker = Ticker();
    ulong lastMessageTime = 0;
    ulong lastCheckStopTime = 0;

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

            if (uxQueueMessagesWaiting(this->queue) == 0 && (!this->stopTaskTicker.active() || (millis() - this->lastCheckStopTime > QUEQU_FINISH_STOP_DELAY_HALF))) {
                Serial.printf("检测到队尾,准备执行清理任务定时器:%d\n", this->getQueueId());
                this->lastCheckStopTime = millis();
                this->stopTaskTicker.once_ms(QUEQU_FINISH_STOP_DELAY, &CatQueue::autoStopTask, this);
            }
        }
    }

    static void autoStopTask(CatQueue *queue) {
        Serial.printf("关闭任务:%d\n", queue->getQueueId());
        if (uxQueueMessagesWaiting(queue->queue) == 0) {
            queue->cleanTask();  //考虑一下要不要吧quequ也清理了
            queue->stopTaskTicker.detach();
        } else {
            queue->stopTaskTicker.once_ms(QUEQU_FINISH_STOP_DELAY, &CatQueue::autoStopTask, queue);
        }
    }

   public:
    /**
     * @brief 向队列添加时间
     *
     * @param event 指向消息的指针,运行完毕时,将会delete
     */
    void push(DynamicJsonDocument *event) {
        xQueueSend(this->queue, &event, 0);
        this->lastMessageTime = millis();
        if (!CatTask::isActive()) CatTask::init();
    }

    /**
     * @brief 获取事件ID
     *
     * @return uint8_t
     */
    virtual uint8_t getQueueId() = 0;
};
