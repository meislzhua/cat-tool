#pragma once
#include <ArduinoJson.h>
#include <EventRegister.h>

/**
 * @brief 任务基类
 */
class CatTaskBase {
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
    int priority = configMAX_PRIORITIES - 1;

    /**
     * @brief 每次执行完任务时,停顿秒数
     *
     */
    int taskDelay = 1;

    /**
     * @brief 任务循环函数
     *
     * @param param
     */
    static void MainTask(void *param) {
        while (true) {
            ((CatTaskBase *)param)->handleTask();
            vTaskDelay(((CatTaskBase *)param)->taskDelay / portTICK_RATE_MS);
        }
    }
    /**
     * @brief 具体任务处理函数
     *
     */
    virtual void handleTask() = 0;

    /**
     * @brief 清理任务
     *
     */
    void cleanTask() {
        if (this->task != NULL) {
            vTaskDelete(&this->task);
            this->task = NULL;
        }
    }

    /**
     * @brief 初始化任务,并新建线程
     *
     */
    void init() {
        cleanTask();
        xTaskCreate(&CatTaskBase::MainTask, NULL, 1024 * 8, this, configMAX_PRIORITIES - 1, &this->task);
    }
};
