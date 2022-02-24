#ifndef TIMER_H
#define TIMER_H

class Timer {
public:
    float get();
    void reset();
private:
    float m_start_time = 0;
};

#endif