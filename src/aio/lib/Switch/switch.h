#ifndef SWITCH_h
#define SWITCH_h

class Switch {
    public:
        Switch(uint8_t, uint16_t);
        bool getState();
        void setState(bool);
        bool readSwitch();

        uint8_t _pin_num;
        uint16_t _button_sensitivity;
        unsigned long _last_debounce_time;
        uint16_t _debounce_delay;

    private:
        bool _switch_state;
};

#endif