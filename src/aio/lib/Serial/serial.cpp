#include "serial.h"

String serial_in = "";

uint8_t serial_listen() {

    uint8_t msg = 0xFF;
    
    while(Serial.available() > 0) {
        char rec = Serial.read();
        serial_in += rec;
        if(rec == '\n') {        
            if(serial_in[0] == 'i') {
                msg = serial_in[1];
	        }
	        serial_in = "";
        }
    }
    return msg; // Empty message
}

void serial_send(uint8_t type, uint8_t message) {
    if(Serial.availableForWrite()) {
        Serial.print(type);
        Serial.print(message,HEX);
        Serial.print('\n');
    }
}