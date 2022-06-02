#ifndef Depthsensor_h
#define Depthsensor_h

#include "Arduino.h"

class Depthsensor {
    public:
        Depthsensor();
        void serialUpdate();
        float getDepth();
	    float _depth_value;	//Moved to public to (hopefully) make it work with Ken's code
    private:
        String _serial_in;
};

#endif