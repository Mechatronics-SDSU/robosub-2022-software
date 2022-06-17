#include "Arduino.h"
#include "leak.h"

Leak::Leak() {
    _leak_state = false;
}

bool Leak::getState() {
    return _leak_state;
}

void Leak::setState(bool state) {
    _leak_state = state;
}