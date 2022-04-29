#include "Arduino.h"
#include "killswitch-serial.h"

Killswitch::Killswitch() {
	_killswitch_state = 0;
	_serial_in = "";
}

void Killswitch::serialUpdate() {
    if(Serial.available() > 0) {
        
        char rec = Serial.read();
        _serial_in += rec;
        if(rec == '\n') {        
            if(_serial_in[0] == 'g') {
	            Serial.print('r');
	            Serial.print(_killswitch_state);
	            Serial.print('\n');
	        }
	        else if(_serial_in[0] == 's') {
	            _killswitch_state = (_serial_in[1]);
	            Serial.print('r');
	            Serial.print(_killswitch_state);
	            Serial.print('\n');
            }
	        _serial_in = "";
        }
    }
}

bool Killswitch::getState() {
    return _killswitch_state;
}

void Killswitch::setState(bool state) {
    _killswitch_state = state;
}
