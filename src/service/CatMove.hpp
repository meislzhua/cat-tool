
#pragma once
#include <ArduinoJson.h>
#include <EventRegister.h>

#include "base/CatQueue.hpp"

/*
 * LEDC Chan to Group/Channel/Timer Mapping
 ** ledc: 0  => Group: 0, Channel: 0, Timer: 0
 ** ledc: 1  => Group: 0, Channel: 1, Timer: 0
 ** ledc: 2  => Group: 0, Channel: 2, Timer: 1
 ** ledc: 3  => Group: 0, Channel: 3, Timer: 1
 ** ledc: 4  => Group: 0, Channel: 4, Timer: 2
 ** ledc: 5  => Group: 0, Channel: 5, Timer: 2
 ** ledc: 6  => Group: 0, Channel: 6, Timer: 3
 ** ledc: 7  => Group: 0, Channel: 7, Timer: 3
 ** ledc: 8  => Group: 1, Channel: 0, Timer: 0
 ** ledc: 9  => Group: 1, Channel: 1, Timer: 0
 ** ledc: 10 => Group: 1, Channel: 2, Timer: 1
 ** ledc: 11 => Group: 1, Channel: 3, Timer: 1
 ** ledc: 12 => Group: 1, Channel: 4, Timer: 2
 ** ledc: 13 => Group: 1, Channel: 5, Timer: 2
 ** ledc: 14 => Group: 1, Channel: 6, Timer: 3
 ** ledc: 15 => Group: 1, Channel: 7, Timer: 3
 */

#define LEFT_LEDC_NUM 14  //左轮PWM调速通道
#define LEFT_PIN_PWM 12   //左轮PWM调速针脚
#define LEFT_PIN1 13      //左轮控制针脚-1
#define LEFT_PIN2 15      //左轮控制针脚-2

#define RIGHT_LEDC_NUM 15  //右轮PWM调速通道
#define RIGHT_PIN_PWM 4    //右轮PWM调速针脚
#define RIGHT_PIN1 2       //右轮控制针脚-1
#define RIGHT_PIN2 14      //右轮控制针脚-2

#define PWM_RESOLUTION 10                    // pwm分辨率
#define PWM_FREQUENCY 1000                   // pwm分辨率
#define PWM_MAX ((1 << PWM_RESOLUTION) - 1)  // pwm分辨率的最大值

/**
 * @brief 普通马达控制类(L298N)
 *
 */
class NormalMotor {
   public:
    uint8_t pin1;
    uint8_t pin2;
    uint8_t speedPin;
    uint8_t ledcNum;

    NormalMotor(uint8_t pin1, uint8_t pin2, uint8_t speedPin, uint8_t ledcNum) {
        this->pin1 = pin1;
        this->pin2 = pin2;
        this->speedPin = speedPin;
        this->ledcNum = ledcNum;
    }

    void init() {
        pinMode(this->pin1, OUTPUT);
        pinMode(this->pin2, OUTPUT);
        pinMode(this->speedPin, OUTPUT);

        ledcSetup(this->ledcNum, PWM_FREQUENCY, PWM_RESOLUTION);
        ledcAttachPin(this->speedPin, this->ledcNum);
    }

    void forward() {
        digitalWrite(this->pin1, HIGH);
        digitalWrite(this->pin2, LOW);
    }
    void back() {
        digitalWrite(this->pin1, LOW);
        digitalWrite(this->pin2, HIGH);
    }
    void stop() {
        digitalWrite(this->pin1, LOW);
        digitalWrite(this->pin2, LOW);
    }

    void speed(uint8_t speed) {
        ledcWrite(LEFT_LEDC_NUM, int(PWM_MAX * speed / 100));  // 输出PWM
    }
};

class CatMoveClass : public CatQueueBase {
   public:
    NormalMotor leftMotor = NormalMotor(LEFT_PIN1, LEFT_PIN2, LEFT_PIN_PWM, LEFT_LEDC_NUM);
    NormalMotor rightMotor = NormalMotor(RIGHT_PIN1, RIGHT_PIN2, RIGHT_PIN_PWM, RIGHT_LEDC_NUM);

    void init() {
        CatQueueBase::init();
        this->leftMotor.init();
        this->rightMotor.init();
    }

    uint8_t getQueueId() {
        return REGISTER_CATMOVE;
    };

    void handleQueue(DynamicJsonDocument &doc) {
        int8_t t;
        //处理左轮动作消息
        if (doc.containsKey("l")) {
            t = doc["l"].as<uint8_t>();
            if (t == 0) {
                this->leftMotor.stop();
            } else if (t == 1) {
                this->leftMotor.forward();
            } else if (t == 2) {
                this->leftMotor.back();
            }
        }
        //处理右轮动作消息
        if (doc.containsKey("r")) {
            t = doc["r"].as<uint8_t>();
            if (t == 2) {
                this->rightMotor.back();
            } else if (t == 0) {
                this->rightMotor.stop();
            } else if (t == 1) {
                this->rightMotor.forward();
            }
        }

        //处理左轮速度消息
        if (doc.containsKey("ls")) {
            t = doc["ls"].as<uint8_t>();
            if (t > 100) t = 100;
            if (t < 0) t = 0;
            this->leftMotor.speed(t);
        }
        //处理右轮速度消息
        if (doc.containsKey("rs")) {
            t = doc["rs"].as<uint8_t>();
            if (t > 100) t = 100;
            if (t < 0) t = 0;
            this->rightMotor.speed(t);
        }
    }

} CatMove;
