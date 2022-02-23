#include <FEHUtility.h>

#include "pid.h"

/**
 * Implements a PID update loop
 * 
 * See https://www.ni.com/en-us/innovations/white-papers/06/pid-theory-explained.html for more details on pid loops
*/
float PIDController::update(float current, float target) {
    float error = target - current;
    float dt = TimeNow() - m_prev_time;
    m_prev_time = TimeNow();

    float result = m_kp * error + m_ki * m_integral_err + m_kd * (error - m_prev_err) / dt;

    m_prev_err = error;
    m_integral_err += error * dt;

    return result;
}


void PIDVAController::set_vel(float vel) {
    m_vel = vel;
}

void PIDVAController::set_acc(float acc) {
    m_acc = acc;
}

float PIDVAController::get_vel(float vel){
    return m_vel;
}

float PIDVAController::get_acc(float acc){
    return m_acc;
}



/**
 * Implements a PIDVA update loop
 * 
 * See https://learnroadrunner.com/drive-constants.html#kv-ka-kstatic for more details on pid loops
*/
float PIDVAController::update(float current_pos, float target_pos) {
    return PIDController::update(current_pos, target_pos) + m_kv * m_vel + m_ka * m_acc + m_k_static;
}