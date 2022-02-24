#include "FEHUtility.h"

#include "timer.h"

void Timer::reset() {
    m_start_time = TimeNow();
}

float Timer::get() {
    return TimeNow() - m_start_time;
}