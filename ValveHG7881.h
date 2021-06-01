/*
  ValveHG7881.h - Custom library for water valve control with HG7881 motor driver.
  Created by Beta-User, Mai, 2021.
*/

#ifndef ValveHG7881_h
#define ValveHG7881_h

#include "Arduino.h"

class ValveHG7881
{
  public:
    ValveHG7881();
    void setPins(uint8_t pin_dir, uint8_t pin_pwm, long duration);
    ValveHG7881(uint8_t pin_dir, uint8_t pin_pwm, long duration);
    int loop(bool button_state);
    void open(int state);
    void close(int state);
    
  private:
    int _pin_dir;
    int _pin_pwm;
  long _duration;
  bool _disable;
  int _state;
  long _finish_time;
  long _mute_time;
};

#endif

