#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <EventRegister.h>
#include <WiFi.h>

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
void freeRam() {
    Serial.println("--------------- Get Systrm Info --------");
    //获取IDF版本
    // printf("     SDK version:%s\n", esp_get_idf_version());
    //获取芯片可用内存
    Serial.println(String(esp_get_free_heap_size()));
    //获取从未使用过的最小内存
    Serial.println(String(esp_get_minimum_free_heap_size()));

    Serial.println("----------------------");
}

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
        length += 1;

        this->client.write((length >> 24) & 0xFF);
        this->client.write((length >> 16) & 0xFF);
        this->client.write((length >> 8) & 0xFF);
        this->client.write(length & 0xFF);
        this->client.write(type);

        this->client.write((const uint8_t *)buffer, length - 1);
    }
    void wifiPing() {
        DynamicJsonDocument doc(20);
        doc["c"] = REGISTER_CATNETWORK_PING;
        doc["r"] = esp_get_free_heap_size();

        this->send(doc);
    }
    //============== wifi任务触发器 ==============

    void handleTask() {
        Serial.printf("wifi - 任务开启\r\n");

        if (this->isWifiConnected()) {  // wifi - 已连接
            Serial.printf("wifi - 连接成功\r\n");
            Serial.println("IP address: ");
            Serial.println(WiFi.localIP());

            this->WifiRetryCount = 0;
            while (this->client.connected()) {  //通信 - 已建立
                // Serial.printf("通信 - 连接成功\r\n");

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
                        this->wifiPing();
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
