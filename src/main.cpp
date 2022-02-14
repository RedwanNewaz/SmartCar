#include <Arduino.h>

#include "vehicle_control.h"
#include "can_sniffer.h"
#include "mqtt_client.h"



TaskHandle_t task_actuation_control; 
CanSniffer *mazdaCAN;




void setup() {
  Serial.begin(250000);
 
 
  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    onVehicleControl,   /* Task function. */
                    "Taskvehicle_control",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &task_actuation_control,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
    delay(500); 
    
    mazdaCAN = new CanSniffer;

}


void loop() {
  mazdaCAN->update();
  
}