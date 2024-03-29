#include "Arduino.h"
#include "depth-serial.h"

Depthsensor::Depthsensor() {
	_depth_value = 0;
	_serial_in = "";
	Serial.print("Depth Sensor Online...");
}

void Depthsensor::serialUpdate() {
    if(Serial.available() > 0) {
        
        char rec = Serial.read();
        _serial_in += rec;
        if(rec == '\n') {        
            if(_serial_in[0] == 'g') {
	            Serial.print('r');
	            Serial.print(_depth_value);
	            Serial.print('\n');
	        }
	        _serial_in = "";
        }
    }
}

float Depthsensor::getDepth() {
    return _depth_value;
}