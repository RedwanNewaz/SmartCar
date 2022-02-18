#pragma once 
#include <BLEAdvertisedDevice.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include "BrainTree.h"
#include "state_machine.h"
// #include "mqtt_client.h"



#ifndef SERVICE_UUID
  // SERVICE_UUID can be generated from this website https://www.uuidgenerator.net/version1
  #define SERVICE_UUID        "f230ca5c-804c-11ec-a8a3-0242ac120002"
#endif


const int BLED_PIN = 2;
const int RELAY_PIN = GPIO_NUM_32; 
const int BUTTON_PIN = GPIO_NUM_33; 

BLEScan* pBLEScan;

// MqttClient *mqtt; 

// BLED -> Bluetooth Low Energy Device 
volatile long BLED_detect_time = 0;
volatile long BLED_scan_time = 1e6;

class BLEScanner:public BrainTree::Node, public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice advertisedDevice) 
  {
    int rssi = advertisedDevice.getRSSI();
    // if the signal strength of current device within acceptable region
    if(rssi > m_CUTOFF_)
    {
      // Check a BLED's UUID. Note that UUID can be shared to multiple device. So keep your UUID in safe place
      BLEUUID adder = advertisedDevice.getServiceUUID();
      BLEUUID Redme(SERVICE_UUID);

      //Update detection time as well as display device info if a match is found 
      if(adder.equals(Redme))
      {        
        // Serial.printf("[+] rssi = %d Advertised Device: %s \n", rssi, advertisedDevice.toString().c_str());
        BLED_detect_time = millis();
      }
    }
    //Record last scan time. If there is no BLED detected, it helps to turn off the switch
    BLED_scan_time = millis();  
  }

  Status update() override
  {
    BLEScanResults results = pBLEScan->start(m_scanTime_, false);
    pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory
    return Node::Status::Success;
  }

  private:
    const int m_scanTime_ = 5; //In seconds
    const int m_CUTOFF_ = -75;
};

/**
 * @brief Relay will toggle subject to the button press 
 * use this tutorial for button interface 
 * https://www.arduino.cc/en/Tutorial/BuiltInExamples/Button
 * 
 */
class CanModeControl: public BrainTree::Node
{
    Status update() override
    {    

        //toggle status based on button press 
        if(digitalRead(BUTTON_PIN) == HIGH)
        {
            String msg; 
            if (shared_msg->can == can_mode::HS)
            {
              shared_msg->can = can_mode::MS;
              msg = "switch to can_mode::MS\n";
            }
            else
            {
              shared_msg->can = can_mode::HS;
              msg = "switch to can_mode::HS\n";
            }
            
            // bin_sem = xSemaphoreCreateBinary();
            // xSemaphoreTake(bin_sem, portMAX_DELAY);
            // mqtt->publishSerialData(msg.c_str());
            // wait 250 ms to avoid debounce noise
            vTaskDelay(250 / portTICK_PERIOD_MS);
        }
        digitalWrite(RELAY_PIN, shared_msg->can == can_mode::HS ? LOW:HIGH);
        return Node::Status::Success; 
    }
};

class AccessControl: public BrainTree::Node
{
  Status update() override
  {
    auto elapsed = BLED_scan_time - BLED_detect_time; 
    digitalWrite(BLED_PIN, elapsed > m_debounce_time_ ? LOW:HIGH);
    return (elapsed > m_debounce_time_) ? Node::Status::Failure:Node::Status::Success; 
  }
  private:
    const long m_debounce_time_ = 15e3;
};


//Task2code: blinks an LED every 700 ms
inline void onVehicleControl( void * pvParameters )
{
  pinMode(BLED_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);

  // mqtt = new MqttClient;
  
  // BLEDevice::init("");
  // pBLEScan = BLEDevice::getScan(); //create new scan
  // pBLEScan->setAdvertisedDeviceCallbacks(new BLEScanner());
  // pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  // pBLEScan->setInterval(100);
  // pBLEScan->setWindow(99);  // less or equal setInterval value


  auto tree = BrainTree::Builder()
      .composite<BrainTree::Sequence>()
          // .leaf<BLEScanner>()
          .leaf<CanModeControl>()
          .leaf<AccessControl>()
      .end()
      .build();

  while(true)
  {
    tree->update();
    // mqtt->run();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}