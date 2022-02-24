#ifndef PID_H
#define PID_H

#include "timer.h"

/**
 * General PID controller for all applications
*/
class PIDController {
public:
    PIDController() = default;
    PIDController(float kp, float ki, float kd) : m_kp(kp), m_ki(ki), m_kd(kd) {}
    
    float update(float current, float target);
    void reset_err();
private:
    float m_kp = 0, m_ki = 0, m_kd = 0;
    float m_prev_err = 0, m_integral_err = 0;
    Timer m_timer;
};

/**
 * Specialized PID feedforeward controller used for driving control
*/
class PIDVAController : PIDController {
public:
    PIDVAController() = default;
    PIDVAController(float kp = 0, float ki = 0, float kd = 0) : PIDController(kp, ki, kd) {}
    
    float update(float current_pos, float target_pos);
    void set_vel(float vel);
    void set_acc(float acc);
    float get_vel(float vel);
    float get_acc(float acc);
private:
    float m_kv, m_ka, m_k_static;
    float m_vel, m_acc;
};

#endif