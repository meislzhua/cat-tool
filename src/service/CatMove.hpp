
#pragma once
#include <ArduinoJson.h>
#include <EventRegister.h>
#include <Stepper.h>
#include <config.h>

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
    Stepper foodMotor = Stepper(32 * 64, STEPPER_FOOD_PIN1, STEPPER_FOOD_PIN3, STEPPER_FOOD_PIN2, STEPPER_FOOD_PIN4);

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

        //处理食物步进电机速度消息
        if (doc.containsKey("fsp")) {
            Serial.printf("电机速度:%d\n", doc["fsp"].as<int>());

            this->foodMotor.setSpeed(doc["fsp"].as<int>());
        }

        //处理食物步进电机移动消息
        if (doc.containsKey("fs")) {
            Serial.printf("电机转动:%d\n", doc["fs"].as<int>());

            this->foodMotor.step(-doc["fs"].as<int>());
            digitalWrite(STEPPER_FOOD_PIN1, LOW);
            digitalWrite(STEPPER_FOOD_PIN2, LOW);
            digitalWrite(STEPPER_FOOD_PIN3, LOW);
            digitalWrite(STEPPER_FOOD_PIN4, LOW);
        }
    }

} CatMove;
