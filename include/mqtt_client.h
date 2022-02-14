#pragma once 
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiSTA.h>
#include "my_helper.h"
#define MQTT_SERIAL_PUBLISH_CH "/testtopic/1"

static SemaphoreHandle_t bin_sem; 

class MqttClient{
public:
    MqttClient()
    {
        initialize_ = false; 
        m_wifiClient_ = new WiFiClient;
        m_pubSubClient_ = new PubSubClient(*m_wifiClient_);
        init();
    }
    void run()
    {
        m_pubSubClient_->loop();
        serialListener();
    }

    void publishSerialData(const char *serialData){
        
        if(!initialize_) return; 

        if (!m_pubSubClient_->connected()) {
            reconnect();
        }
        // bin_sem = xSemaphoreCreateBinary();
        // xSemaphoreTake(bin_sem, portMAX_DELAY);
        printf("%s", serialData);
        m_pubSubClient_->publish(MQTT_SERIAL_PUBLISH_CH, serialData);
        // xSemaphoreGive(bin_sem);
    }

    void serialListener()
    {
        if (Serial.available() > 0) {
            char mun[501];
            memset(mun,0, 501);
            Serial.readBytesUntil( '\n',mun,500);
            publishSerialData(mun);
        }
    }



private:
    WiFiClient *m_wifiClient_;
    PubSubClient *m_pubSubClient_;
    bool initialize_;
protected:
    void init()
    {
        vTaskDelay(10);
        // We start by connecting to a WiFi network
        // // Serial.println();
        // // Serial.print("Connecting to ");
        // // Serial.println(SSID);
        WiFi.begin(SSID, PASSWORD);
        while (WiFi.status() != WL_CONNECTED) {
            vTaskDelay(500);
            // // Serial.print(".");
        }
        randomSeed(micros());
        // Serial.println("");
        // Serial.println("WiFi connected");
        // Serial.println("IP address: ");
        // Serial.println(WiFi.localIP());

        m_pubSubClient_->setServer(mqtt_server, mqtt_port);
        m_pubSubClient_->setCallback(callback);
        reconnect();
        initialize_ = true;
    }

    void reconnect() {
        // Loop until we're reconnected
        while (!m_pubSubClient_->connected()) {
            // Serial.print("Attempting MQTT connection...");
            // Create a random client ID
            String clientId = "ESP32But-";
            clientId += String(random(0xffff), HEX);
            // Attempt to connect
            if (m_pubSubClient_->connect(clientId.c_str())) {
            // Serial.println("connected");
            //Once connected, publish an announcement...
                m_pubSubClient_->publish(MQTT_SERIAL_PUBLISH_CH, "testing...");
            } else {
            // Serial.print("failed, rc=");
            // Serial.print(m_pubSubClient_->state());
            // Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            vTaskDelay(5000);
            }
        }
    }

    static void callback(char* topic, byte *payload, unsigned int length) {
        // Serial.println("-------new message from broker-----");
        // Serial.print("channel:");
        // Serial.println(topic);
        // Serial.print("data:");  
        Serial.write(payload, length);
        // Serial.println();
    }

};