#ifndef ARM_h
#define ARM_h

#include <Servo.h>

class Arm {
    public:
        Arm(unsigned int);
        bool getState();
        void setState(bool);
        Servo arm;

    private:
        bool _arm_state;
        unsigned int _open_pwm;
        unsigned int _close_pwm;
};

#endif