#pragma once
#include <ArduinoJson.h>
#include <EventRegister.h>
#include <string.h>
/**
 * @brief 任务基类
 */
class CatTask {
   public:
    /**
     * @brief 任务线程句柄
     *
     */
    TaskHandle_t task = NULL;  //任务线程

    /**
     * @brief 任务优先级
     *
     */
    uint8_t priority = 0;

    /**
     * @brief 每次执行完任务时,停顿秒数
     *
     */
    String taskName = String(esp_random());

    /**
     * @brief 每次执行完任务时,停顿秒数
     *
     */
    uint32_t taskDelay = 1;

    /**
     * @brief 任务循环函数
     *
     * @param param
     */
    static void MainTask(void *param) {
        while (true) {
            // Serial.printf("执行:%s\r\n", ((CatTask *)param)->taskName);

            ((CatTask *)param)->handleTask();
            vTaskDelay(((CatTask *)param)->taskDelay / portTICK_RATE_MS);
        }
        // vTaskDelay(((CatTask *)param)->taskDelay / portTICK_RATE_MS);
    }
    /**
     * @brief 具体任务处理函数
     *
     */
    virtual void handleTask() = 0;

    /**
     * @brief 初始化任务,并新建线程
     *
     */
    void init() {
        Serial.printf("执行初始化:%s\n", this->taskName.c_str());

        cleanTask();
        xTaskCreate(&CatTask::MainTask, this->taskName.c_str(), 1024 * 8, this, this->priority, &this->task);
    }

   protected:
    bool isActive() {
        return this->task != NULL;
    }

    /**
     * @brief 清理任务
     *
     */
    void cleanTask() {
        Serial.printf("清理任务:%s\n", this->taskName.c_str());

        if (this->task != NULL) {
            vTaskDelete(this->task);
            this->task = NULL;
        }
    }
};
