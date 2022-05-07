#ifndef Killswitch_h
#define Killswitch_h

#include "Arduino.h"

class Killswitch {
    public:
        Killswitch();
        void serialUpdate();
        bool getState();
        void setState(bool state);
		bool _killswitch_state = 0;	//Moved to public to (hopefully) make it work with Ken's code
    private:
        String _serial_in;
};

#endif
