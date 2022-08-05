#ifndef SERIAL_h
#define SERIAL_h

#include <Arduino.h>

uint8_t serial_listen();
void serial_send(uint8_t, uint8_t);

#endif