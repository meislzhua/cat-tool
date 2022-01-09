#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <EventRegister.h>
#include <WiFi.h>
#include <config.h>

#include "CatEvent.hpp"
#include "base/CatTask.hpp"
#define serverIP "192.168.0.49"                          // 服务器的地址
#define serverPort 8431                                  // 服务器的地址
#define ConnectMaxRetryCount 10                          // 通信重连最大间隔秒数计数
#define WIFI_MAX_RETRY_COUNT 10                          // WIFI重连最大间隔秒数计数
#define WIFI_NO_MESSAGE_SLEEP (1000 / portTICK_RATE_MS)  // WIFI无信息睡眠毫秒数
#define WIFI_NO_MESSAGE_PING 60                          // WIFI无信息N次后,发送ping

#define MESSAGE_TYPE_JSON 0  // 传输数据类型:JSON
#define MESSAGE_TYPE_JPEG 1  // 传输数据类型:JPEG

class CatNetworkClass : public CatTaskBase {
    TaskHandle_t receiveTask = NULL;  // wifi信息接受器
    WiFiClient client;                // wifi连接

   public:
    // todo 改成配置式
    char const *ssid = "MeIsCLing";
    char const *password = "cl82266306";

    uint8_t ConnectRetryCount = 0;
    uint8_t WifiRetryCount = 0;
    uint8_t WifiNoMessageCount = 0;
    uint8_t isSending = 0;
    SemaphoreHandle_t wifiSendMutex = xSemaphoreCreateMutex();

    void log() {}

    inline bool isWifiConnected() { return WiFi.status() == WL_CONNECTED; }

    void connect() { WiFi.begin(this->ssid, this->password); }

    void init() {
        CatTaskBase::init();
        this->connect();
    }

    void send(DynamicJsonDocument doc) {
        String json;
        serializeJson(doc, json);
        this->send((void *)json.c_str(), json.length(), MESSAGE_TYPE_JSON);
    }
    inline void send(void *buffer, int length, uint8_t type) {
        xSemaphoreTake(this->wifiSendMutex, portMAX_DELAY);

        length += 1;
        uint8_t header[] = {(length >> 24) & 0xFF, (length >> 16) & 0xFF, (length >> 8) & 0xFF, length & 0xFF, type};
        this->client.write(header, 5);

        this->client.write((const uint8_t *)buffer, length - 1);

        xSemaphoreGive(this->wifiSendMutex);
    }

    void sendPing() {
        DynamicJsonDocument doc(60);
        doc["c"] = REGISTER_CATNETWORK_PING;
        doc["r"] = esp_get_free_heap_size();

        this->send(doc);
    }

    void sendDeviceId() {
        DynamicJsonDocument doc(60);
        doc["c"] = REGISTER_DEVICE_ID;
        doc["n"] = DEVICE_ID;

        this->send(doc);
    }
    //============== wifi任务触发器 ==============

    void handleTask() {
        Serial.printf("wifi - 任务开启\r\n");

        if (this->isWifiConnected()) {  // wifi - 已连接
            Serial.printf("wifi - 连接成功\r\n");
            Serial.println("IP address: ");
            Serial.println(WiFi.localIP());

            while (this->isWifiConnected()) {
                this->WifiRetryCount = 0;
                if (this->client.connected()) {
                    Serial.printf("通信 - 连接成功\r\n");

                    this->sendDeviceId();
                }

                while (this->client.connected()) {  //通信 - 已建立
                    // 摄像头数据发送

                    if (this->client.available()) {  //如果有数据可读取
                        String line = this->client.readStringUntil('\n');

                        DynamicJsonDocument *doc = new DynamicJsonDocument(line.length() * 3);
                        DeserializationError error = deserializeJson(*doc, line.c_str());
                        if (error) {
                            Serial.printf("数据包转换发生错误:[%s]\n", line.c_str());
                            Serial.println(error.c_str());
                            delete doc;

                        } else {
                            CatEvent.trigger(doc);
                            Serial.printf("成功分发数据包:%s\n", line.c_str());
                        }

                        vTaskDelay(1 / portTICK_RATE_MS);

                    } else {
                        vTaskDelay(WIFI_NO_MESSAGE_SLEEP);
                        if (this->WifiNoMessageCount > WIFI_NO_MESSAGE_PING) {
                            this->sendPing();
                            this->WifiNoMessageCount = 0;
                        }
                        this->WifiNoMessageCount++;
                    }
                    // freeRam();
                }

                Serial.printf("通信 - 连接失败\r\n");

                //尝试建立通信
                if (!this->client.connect(
                        serverIP, serverPort)) {  //建立通信失败时, 延迟线程
                    if (this->ConnectRetryCount < ConnectMaxRetryCount) {
                        this->ConnectRetryCount++;
                    }
                    vTaskDelay(this->ConnectRetryCount * 1000 / portTICK_RATE_MS);
                } else {
                    this->ConnectRetryCount = 0;
                }
            }

        } else {  // wifi - 没连接, 延迟线程
            Serial.printf("wifi - 连接失败\r\n");

            if (this->WifiRetryCount < WIFI_MAX_RETRY_COUNT) {
                this->WifiRetryCount++;
            }
            vTaskDelay(this->WifiRetryCount * 1000 / portTICK_RATE_MS);
        }
    }

    /**
     * @brief 清除wifi接收器任务
     *
     */
    void cleanTask() {
        CatTaskBase::cleanTask();
        client.stop();
    }

} CatNetwork;
