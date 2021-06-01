/*
  ValveHG7881.cpp - Custom library for water valve control with HG7881 motor driver.
  Created by Beta-User, Mai, 2021.
*/


//#include "Arduino.h"
#include "ValveHG7881.h"


const int STATE_UNKNOWN = 0;
const int STATE_CLOSED = 1;
const int STATE_OPEN = 2;
const int STATE_MOVING = 3;

ValveHG7881::ValveHG7881() {}

void ValveHG7881::setPins(uint8_t pin_on, uint8_t pin_down, long duration)
{
  _pin_dir  = pin_dir;
  _pin_pwm  = pin_pwm;
  _duration = duration;
}
  
ValveHG7881::ValveHG7881(uint8_t pin_dir, uint8_t pin_pwm, long duration)
{
  //pinMode(pin, OUTPUT);
  setPins(pin_dir, pin_down, duration);
  _disable = false;
}


int ValveHG7881::loop(bool button_state)
{

  if(_mute_time > millis()){
    return _state;
  }
  

  if(button_state){
    if(_state == STATE_CLOSED){ //Already disabled/disabling
      return _state;
    }
    if(_state == STATE_MOVING){
    #ifdef MY_DEBUG_LOCAL
    debug("Stop enabling");
    #endif
    _mute_time = millis() + 400;
      stopMovement(STATE_UNKNOWN);
      return _state;
    }
    #ifdef MY_DEBUG_LOCAL
    debug("Start disabling");
    #endif
    startMovement(STATE_DISABLING);

  }else if(button_enable){
    if(_state == STATE_ENABLED || _state == STATE_ENABLING){ //Already enabled/enabling
    #ifdef MY_DEBUG_LOCAL
    debug("Already enabling/enabled");
    #endif
      return _state;
    }
    if(_state == STATE_DISABLING){
    #ifdef MY_DEBUG_LOCAL
    debug("Stop disabling");
    #endif
    _mute_time = millis() + 400;
      stopMovement(STATE_UNKNOWN);
      return _state; //Welcher Rückgabecode?
    }
    #ifdef MY_DEBUG_LOCAL
    debug("Start enabling");
  #endif
    startMovement(STATE_ENABLING);
  }
  
  
  if (_finish_time <= millis() && _finish_time > 0) {
    #ifdef MY_DEBUG_LOCAL
   debug("reached finish time");
   #endif
    switch (_state) {
      case STATE_DISABLING:
        stopMovement(STATE_DISABLED);
        break;
      case STATE_ENABLING:
        stopMovement(STATE_ENABLED);
        break;
    }
  }
return _state;  
}

void ValveHG7881::setDisable(boolean b)
{
  _disable = b;
}


void ValveHG7881::stopMovement(int state) {
  _finish_time = 0; //Set destination time to 0 -> it's not active anymore
  digitalWrite(_pin_on, HIGH);
  delay(150);
  digitalWrite(_pin_down, HIGH);
  _state = state;
}


void ValveHG7881::debug(String text) {
/*String prefix = String();
prefix = "["+ _pin_on;
prefix = prefix+" ";
prefix = prefix + _pin_down;
prefix = prefix +"] ";
String goal = String();
goal = prefix + text;*/

Serial.println(_pin_on + text);
}

void ValveHG7881::startMovement(int state)
{
  setState(state);
  
  if(_state == STATE_DISABLING){ 
    digitalWrite(_pin_down, HIGH);//Activate relais and make it ready to disable this component
  }else if(_state == STATE_ENABLING){ 
    digitalWrite(_pin_down, LOW); //Activate relais and make it ready to enable this component
  }
  
  delay(150);
  digitalWrite(_pin_on, LOW); //Activate motor
  _finish_time = millis() + _duration; //Set destination time
}

void ValveHG7881::setState(int i)  //-1 = unknown. 0 = enabled; 1 = move_disable; 2 = disabled; 3 = move_enable;
{
  _state = i;
}

int ValveHG7881::getState()
{
  return _state;
}

