#pragma once
#include <ArduinoJson.h>
#include <EventRegister.h>

#include "CatNetwork.hpp"
#include "base/CatQueue.hpp"
#include "config.h"
#include "esp_camera.h"

/**
 * @brief 摄像头控制类
 *
 */
class CatCamClass : public CatQueueBase {
   public:
    /**
     * @brief 是否循环发送图像信息
     *
     */
    bool isSendLoopOpen = false;
    unsigned long lastCamTime = 0;
    uint8_t imageMaxDelay = 50;
    void handleQueue(DynamicJsonDocument& doc) {
        Serial.printf("接受到摄像头事件\n");

        if (doc.containsKey("o")) {
            this->isSendLoopOpen = doc["o"].as<int>();
            Serial.printf("开关:%d\n", this->isSendLoopOpen);
        }

        if (doc.containsKey("q")) {
            sensor_t* s = esp_camera_sensor_get();
            s->set_quality(s, doc["q"].as<int>());
        }

        if (doc.containsKey("s")) {
            sensor_t* s = esp_camera_sensor_get();
            s->set_framesize(s, (framesize_t)doc["s"].as<int>());
        }

        if (doc.containsKey("d")) {
            this->imageMaxDelay = doc["d"].as<int>();
        }
    }

    uint8_t getQueueId() { return REGISTER_CATCAM; };

    void init() {
        CatQueueBase::init();
        camera_config_t config;
        config.ledc_channel = LEDC_CHANNEL_0;
        config.ledc_timer = LEDC_TIMER_0;
        config.pin_d0 = Y2_GPIO_NUM;
        config.pin_d1 = Y3_GPIO_NUM;
        config.pin_d2 = Y4_GPIO_NUM;
        config.pin_d3 = Y5_GPIO_NUM;
        config.pin_d4 = Y6_GPIO_NUM;
        config.pin_d5 = Y7_GPIO_NUM;
        config.pin_d6 = Y8_GPIO_NUM;
        config.pin_d7 = Y9_GPIO_NUM;
        config.pin_xclk = XCLK_GPIO_NUM;
        config.pin_pclk = PCLK_GPIO_NUM;
        config.pin_vsync = VSYNC_GPIO_NUM;
        config.pin_href = HREF_GPIO_NUM;
        config.pin_sscb_sda = SIOD_GPIO_NUM;
        config.pin_sscb_scl = SIOC_GPIO_NUM;
        config.pin_pwdn = PWDN_GPIO_NUM;
        config.pin_reset = RESET_GPIO_NUM;
        config.xclk_freq_hz = 20000000;
        config.pixel_format = PIXFORMAT_JPEG;
        config.frame_size = FRAMESIZE_HVGA;
        config.jpeg_quality = 20;
        config.fb_count = 2;

        esp_camera_init(&config);
    }

    /**
     * @brief 发送一帧图像信息
     *
     */
    void sendFrame() {
        camera_fb_t* fb = esp_camera_fb_get();

        if (fb) {
            CatNetwork.send(fb->buf, fb->len, MESSAGE_TYPE_JPEG);
            esp_camera_fb_return(fb);
        }
    }

    void handleTask() {
        CatQueueBase::handleTask();
        if (this->isSendLoopOpen && millis() - this->lastCamTime > this->imageMaxDelay) {
            this->lastCamTime = millis();
            this->sendFrame();
        }
    }
} CatCam;
