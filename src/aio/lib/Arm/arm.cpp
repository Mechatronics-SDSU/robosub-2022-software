#include "Arduino.h"
#include "arm.h"

Arm::Arm(unsigned int pin) {
    _arm_state = false;
    _open_pwm = 1600;
    _close_pwm = 1300;
    arm.attach(pin);
}

bool Arm::getState() {
    return _arm_state;
}

void Arm::setState(bool state) {
    if(state == LOW){
        arm.writeMicroseconds(_close_pwm);
        //delay
    }
    else if(state == HIGH) {
        arm.writeMicroseconds(_open_pwm);
        //delay
    }
    _arm_state = state;
}