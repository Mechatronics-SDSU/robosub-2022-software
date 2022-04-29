#ifndef Killswitch_h
#define Killswitch_h

#include "Arduino.h"

class Killswitch {
    public:
        Killswitch();
        void serialUpdate();
        bool getState();
        void setState(bool state);
    private:
        bool _killswitch_state = 0;
        String _serial_in;
};

#endif
