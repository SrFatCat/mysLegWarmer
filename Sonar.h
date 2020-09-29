#ifndef SONAR_H
#define SONAR_H

#include <Arduino.h>
class CSonar{
    const uint8_t _pin;
    const uint32_t _interval;
    uint32_t _start_ms = 0;
    uint32_t _var_set_ms = 0;
    uint32_t _var_timeout;
    uint16_t _var_minDist;
    bool *_var = nullptr;
    uint16_t _dist;
    enum ESonarState {IDLE, START, FINISH, READY} _state = ESonarState::IDLE;
    
    static void sonarInt();
    static CSonar *_thisSonar;

public:
    CSonar(uint8_t pin, uint32_t interval) : _pin(pin), _interval(interval) { _thisSonar=this; }
    void update();
    bool get(uint16_t &dist);
    void attachVar(bool *var, uint16_t minDist, uint32_t timeout);
    inline void setVarTimeout(uint32_t timeout) {_var_timeout = timeout;}
};
#endif
