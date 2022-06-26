#ifndef LEAK_h
#define LEAK_h

class Leak {
    public:
        Leak();
        bool getState();
        void setState(bool);

    private:
        bool _leak_state;
};

#endif