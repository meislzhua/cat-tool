
#pragma once
#include <ArduinoJson.h>
#include <EventRegister.h>
#include <Ticker.h>
#include <config.h>

#include "base/CatQueue.hpp"
#include "base/CatTask.hpp"

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
    u_long lastMoveTime = 0;
    u_long checkStopDelay = 500;

    Ticker checkStopTicker = Ticker();

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
        this->lastMoveTime = millis();
        NormalMotor::checkStop(this);
    }
    void back() {
        digitalWrite(this->pin1, LOW);
        digitalWrite(this->pin2, HIGH);
        this->lastMoveTime = millis();
        NormalMotor::checkStop(this);
    }
    void stop() {
        digitalWrite(this->pin1, LOW);
        digitalWrite(this->pin2, LOW);
        this->lastMoveTime = 0;
    }

    static void checkStop(NormalMotor *motor) {
        if (motor->checkStopDelay) {
            u_long now = millis();
            if (now - motor->lastMoveTime > motor->checkStopDelay) {
                motor->stop();
            } else {
                motor->checkStopTicker.once_ms(motor->checkStopDelay - (now - motor->lastMoveTime) + 1, &NormalMotor::checkStop, motor);
            }
        }
    }

    void speed(uint8_t speed) {
        ledcWrite(LEFT_LEDC_NUM, int(PWM_MAX * speed / 100));  // 输出PWM
    }
};

class StepperMotor : public CatQueue {
    int8_t pin1;
    int8_t pin2;
    int8_t pin3;
    int8_t pin4;
    int delay;
    int8_t status = 0;

    void _step(int count) {
        int8_t status = this->status;
        int8_t s[8][4] = {
            {HIGH, LOW, LOW, LOW},
            {HIGH, HIGH, LOW, LOW},
            {LOW, HIGH, LOW, LOW},
            {LOW, HIGH, HIGH, LOW},
            {LOW, LOW, HIGH, LOW},
            {LOW, LOW, HIGH, HIGH},
            {LOW, LOW, LOW, HIGH},
            {HIGH, LOW, LOW, HIGH},
        };
        while (count) {
            //状态移动
            digitalWrite(this->pin1, s[status][0]);
            digitalWrite(this->pin2, s[status][1]);
            digitalWrite(this->pin3, s[status][2]);
            digitalWrite(this->pin4, s[status][3]);

            //状态确定
            if (count > 0) {
                count--;
                status = (status + 1) & 0x07;
            } else {
                count++;
                status = (status - 1) & 0x07;
            }

            vTaskDelay(pdMS_TO_TICKS(this->delay));
        }
        digitalWrite(this->pin1, LOW);
        digitalWrite(this->pin2, LOW);
        digitalWrite(this->pin3, LOW);
        digitalWrite(this->pin4, LOW);

        this->status = status;
    }

    void handleQueue(DynamicJsonDocument &doc) {
        this->_step(doc["step"].as<int>());
    }
    uint8_t getQueueId() { return 0; }

   public:
    void step(int count) {
        DynamicJsonDocument *doc = new DynamicJsonDocument(20);
        (*doc)["step"] = count;
        this->push(doc);
    }

    void setDelay(int delay) {
        this->delay = delay;
    }

    StepperMotor(int8_t pin1, int8_t pin2, int8_t pin3, int8_t pin4, int delay) {
        this->pin1 = pin1;
        this->pin2 = pin2;
        this->pin3 = pin3;
        this->pin4 = pin4;

        this->delay = delay;
    }

    void init() {
        CatQueue::init();
        pinMode(this->pin1, OUTPUT);
        pinMode(this->pin2, OUTPUT);
        pinMode(this->pin3, OUTPUT);
        pinMode(this->pin4, OUTPUT);
    }
};

class CatMoveClass : public CatQueue {
   public:
    NormalMotor leftMotor = NormalMotor(LEFT_PIN1, LEFT_PIN2, LEFT_PIN_PWM, LEFT_LEDC_NUM);
    NormalMotor rightMotor = NormalMotor(RIGHT_PIN1, RIGHT_PIN2, RIGHT_PIN_PWM, RIGHT_LEDC_NUM);
    StepperMotor foodMotor = StepperMotor(STEPPER_FOOD_PIN1, STEPPER_FOOD_PIN2, STEPPER_FOOD_PIN3, STEPPER_FOOD_PIN4, 10);

    void init() {
        CatQueue::init();
        this->leftMotor.init();
        this->rightMotor.init();
        this->foodMotor.init();
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
        if (doc.containsKey("fsd")) {
            this->foodMotor.setDelay(doc["fsd"].as<int>());
        }

        //处理食物步进电机移动消息
        if (doc.containsKey("fs")) {
            this->foodMotor.step(-doc["fs"].as<int>());
        }
    }

} CatMove;
