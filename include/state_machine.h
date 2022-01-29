#pragma once 
#include "shared_ptr.h"

enum class door_state 
{
  LOCKED, 
  UNLOCKED
};

enum class car_state
{
  ACTIVE, 
  BATTERY, 
  SLEEP
};

enum class key_state
{
  IN, 
  OUT
};

enum class controller_state
{
    IDLE, 
    RUNNING
};

enum class can_mode 
{
    HS, 
    MS
}; 

struct SharedData
{
  door_state door; 
  car_state car;
  key_state key;
  controller_state cntrl; 
  can_mode can;
   
  SharedData()
  {
    door  = door_state::LOCKED; 
    car   = car_state::SLEEP; 
    key   = key_state::OUT;
    cntrl = controller_state::IDLE;
    can   = can_mode::HS;
  }
};

shared_ptr<SharedData> shared_msg(new SharedData); 