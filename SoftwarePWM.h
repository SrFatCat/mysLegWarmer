#ifndef SOFTWAREPWM_H
#define SOFTWAREPWM_H

#include <Arduino.h>

class CSoftwarePWM {
    const uint8_t *_pins;
    const uint8_t _pinCount;
    uint8_t *_pwmValue;
    uint8_t **_pinReg;
    uint8_t *_pinMask;
    uint8_t _pwmCounter = 0;
public:
    CSoftwarePWM(const uint8_t *pins, const uint8_t pinCount);
    ~CSoftwarePWM();
    void analogWrite(uint8_t pin, uint8_t val);
    inline bool digitalRead(uint8_t pin) {return (getPWM(pin) > 0);}
    uint8_t getPWM(uint8_t pin);
    void init();
    volatile static CSoftwarePWM* thisObj;    
    void isr();
};

#endif