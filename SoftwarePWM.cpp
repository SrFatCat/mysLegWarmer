#include "SoftwarePWM.h"
#include <MyArduino.h>

volatile CSoftwarePWM* CSoftwarePWM::thisObj = nullptr;

CSoftwarePWM::CSoftwarePWM(const uint8_t *pins, const uint8_t pinCount) : _pins(pins), _pinCount(pinCount) {
    _pwmValue = new uint8_t[pinCount];
    _pinReg = new uint8_t*[pinCount];
    _pinMask = new uint8_t[pinCount];
    for (int i=0; i < _pinCount; i++){
        _pwmValue[i] = 0;
        _pinReg[i] = portOutputRegister(digitalPinToPort(_pins[i]));
        _pinMask[i] = digitalPinToBitMask(_pins[i]);
    }
    thisObj = this;
}

void CSoftwarePWM::analogWrite(uint8_t pin, uint8_t val){
    for (int i=0; i < _pinCount; i++){
        if (_pins[i] == pin) {
            _pwmValue[i] = val;
            break;
        }
    }
}

uint8_t CSoftwarePWM::getPWM(uint8_t pin){
    for (int i=0; i < _pinCount; i++){
        if (_pins[i] == pin) {
            return _pwmValue[i];
        }
    }
}


CSoftwarePWM::~CSoftwarePWM(){
    delete _pinMask;
    delete _pinReg;
    delete _pwmValue;
}

void CSoftwarePWM::init(){
    for (uint8_t i = 0; i < _pinCount; i++) pinMode( _pins[i], OUTPUT);
    TCCR2B=0;
    TCCR2A=0;
    //TCNT2 = 0;
    TCCR2A |= (1<<COM2B0) | (1<<WGM21);
    TCCR2B = 1<<CS20;
    TIMSK2 = (1<<OCIE2A);
    OCR2A=1;
    // TCCR2B = (0 << CS22) | (0 << CS21) | (0 << CS20);
    // TIMSK2 |= (1 << OCIE2B); 
}

void CSoftwarePWM::isr(){
    _pwmCounter++;
    //_pwmCounter > _pwmValue[0] ? *(_pinReg[0]) &= ~(_pinMask[0]) : *(_pinReg[0]) |= _pinMask[0];
    for (uint8_t i = 0; i<_pinCount; i++){
          _pwmCounter > _pwmValue[i] ? *(_pinReg[i]) &= ~(_pinMask[i]) : *(_pinReg[i]) |= _pinMask[i];
    }
}


// ISR(TIMER2_COMPA_vect){
//     //CSoftwarePWM::thisObj->isr();
//     digitalWrite(A2, !digitalRead(A2));
// }
