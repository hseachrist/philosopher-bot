#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H

#include "timer.h"

/**
 * Used to Generate a Trapezoidal motion Profile
 * 
 * This is used to make smoother motor and servo moments, and to minimize the amount
 * of error in the system.
 * 
 * For more information:
 * https://www.motioncontroltips.com/what-is-a-motion-profile/#:~:text=A%20motion%20profile%20provides%20the,to%20send%20to%20the%20motor.
 * 
 * */

struct MotionState {
    float time, pos, vel, accel;
};

class MotionProfile {
public:
    MotionProfile(float max_vel = 0, float max_accel = 0) : m_max_vel(max_vel), m_max_accel(max_accel) {}

    MotionState get_motion_state(float time);

    void set_max_vel(float);
    void set_max_accel(float);
    void set_target_pos(float);
    float get_max_vel();
    float get_max_accel();
    float get_target_pos();

private:
    float m_max_vel, m_max_accel, m_target_pos = 0;
};

#endif