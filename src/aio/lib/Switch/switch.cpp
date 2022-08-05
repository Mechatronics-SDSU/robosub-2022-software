#include "Arduino.h"
#include "switch.h"

Switch::Switch(uint8_t pin, uint16_t sensitivity, bool start_state)
{
	_switch_state = start_state;
	_pin_num = pin;
	_button_sensitivity = sensitivity;
	_debounce_delay = 200;
}

bool Switch::getState()
{
	return _switch_state;
}

void Switch::setState(bool state)
{
	_switch_state = state;
}

bool Switch::readSwitch()
{

	if ((millis() - _last_debounce_time) > _debounce_delay)
	{
		uint16_t force = analogRead(_pin_num);
		if (force >= _button_sensitivity)
		{
			_switch_state = !_switch_state; // flip value of buttonstate
			_last_debounce_time = millis(); // reset last button timer

			return HIGH;
		}
	}
	return LOW;
}