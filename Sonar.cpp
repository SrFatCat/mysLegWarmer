#include "Sonar.h"

CSonar *CSonar::_thisSonar = nullptr;

void CSonar::attachVar(bool *var, uint16_t minDist, uint32_t timeout){
    _var = var;
    _var_minDist = minDist;
    _var_timeout = timeout;

}

bool CSonar::get(uint16_t &dist){
    if (_state != ESonarState::READY) return false;
    dist = _dist;
    _state = ESonarState::IDLE;
    return true;
}

void CSonar::update(){
    if (!_thisSonar || _state == ESonarState::START || _state == ESonarState::FINISH ) return;
    if (millis() - _start_ms < _interval) return;
    _start_ms = millis();
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
    delayMicroseconds(5);
    digitalWrite(_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_pin, LOW);
    pinMode(_pin, INPUT);
    _state = ESonarState::START;
    attachInterrupt(digitalPinToInterrupt(_pin), CSonar::sonarInt, RISING );
}

void CSonar::sonarInt(){
    static uint32_t start = 0;
    const uint32_t m = micros();
    detachInterrupt(digitalPinToInterrupt(_thisSonar->_pin));
    if (_thisSonar){
        if (_thisSonar->_state == ESonarState::START) {
            start = m;
            _thisSonar->_state = ESonarState::FINISH;
            attachInterrupt(digitalPinToInterrupt(_thisSonar->_pin), CSonar::sonarInt, FALLING );    
        } else if (_thisSonar->_state == ESonarState::FINISH) { 
            _thisSonar->_dist = (((m-start) / 2) * 100) / 291;
            _thisSonar->_state = ESonarState::READY;
            if (_thisSonar->_var){
                if (_thisSonar->_dist <= _thisSonar->_var_minDist) {
                    _thisSonar->_var_set_ms = millis();
                    if ( !(*_thisSonar->_var) ) (*_thisSonar->_var) = true;
                }    
                else {
                    if ( millis() - _thisSonar->_var_set_ms > _thisSonar->_var_timeout && (*_thisSonar->_var) ) (*_thisSonar->_var) = false;   
                }
            }
        }
    }
}
