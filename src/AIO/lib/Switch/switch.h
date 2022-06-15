#ifndef SWITCH_h
#define SWITCH_h

class Switch {
    public:
        Switch(uint8_t, uint16_t);
        bool getState();
        void setState(bool);
        void readSwitch();

        uint8_t _pin_num;
        uint16_t _button_sensitivity;
        unsigned long _last_debounce_time;

    private:
        bool _switch_state;
};

#endif