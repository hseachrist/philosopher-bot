#include "motion_profile.h"
#include "util.h"

// This motion profile should generate a velocity curve that looks like this:
//
//    |
//  V |
//  e |      ___________________
//  l |     /                   \
//  o |    /                     \
//  c |   /                       \
//  i |  /                         \
//  t | /                           \
//  y |/                             \
//    |--------------------------------------------
//               Time
MotionState MotionProfile::get_motion_state(float time) {
    // It's possible during the profile that the robot never reaches it's max velocity
    // due to having a short distance to travel, so we have a check for that;

    MotionState state;
    state.time = time;
    if (m_max_vel / m_max_accel * m_max_vel >= m_target_pos) {
        // Motor will reach max velocity
        float time_accelerating = m_max_vel / m_max_accel;
        float dist_traveled_at_max = m_target_pos - time_accelerating * m_max_vel;
        float time_traveled_at_max = dist_traveled_at_max / m_max_vel;

        float final_time = 2 * time_accelerating + time_traveled_at_max;
        if (time < time_accelerating) {
            // Motor accelerating
            state.accel = m_max_accel;
            state.vel = state.accel * state.time;
            state.pos = .5 * state.vel * state.time;
        } else if (time < time_traveled_at_max - time_accelerating) {
            // Motor at max velocity
            state.accel = 0;
            state.vel = m_max_vel;
            state.pos = (.5 * time_accelerating * m_max_vel) + (time - time_accelerating) * m_max_vel;
        } else if (time < time_traveled_at_max) {
            // Motor decelerating
            state.accel = -m_max_accel;
            state.vel = m_max_vel + state.time * state.accel;
            float time_decelerating = time - time_traveled_at_max - time_accelerating;
            // Integrals amirite
            //
            //              ----- Time spent accelerating           ----- Time spent at max             ----- Time spent decelerating 
            //              |                                       |                                   |
            //              v                                       v                                   v
            state.pos = (.5 * time_accelerating * m_max_vel) + (time_traveled_at_max * m_max_vel) + (time_decelerating * state.vel + .5 * time_decelerating * (m_max_vel - state.vel));
        } else {
            // Motor is DONE
            state.accel = 0;
            state.vel = 0;
            state.pos = m_target_pos;
        }
    } else {
        float time_accelerating = sqrt_approx(m_target_pos / m_max_accel);
        
        if (time < time_accelerating) {
            // Motor Accelerating
            state.accel = m_max_accel;
            state.vel = m_max_accel * time;
            state.pos = state.vel * time;
        } else if (time < 2 * time_accelerating) {
            // Motor Decelerating
            float highest_vel_reached = m_max_accel * time_accelerating;
            state.accel = -m_max_accel;
            state.vel = m_max_accel * time;

            float time_decelerating = time - time_accelerating;
            state.pos = (.5 * highest_vel_reached * time) + (time_decelerating * state.vel + .5 * time_decelerating * (highest_vel_reached - state.vel));
        } else {
            // Motor is DONE
            state.accel = 0;
            state.vel = 0;
            state.pos = m_target_pos;
        }
    }

    return state;
}

void MotionProfile::set_max_vel(float max_vel){
    m_max_vel = max_vel;
}

void MotionProfile::set_max_accel(float max_accel){
    m_max_accel = max_accel;
}

void MotionProfile::set_target_pos(float target_pos) {
    m_target_pos = target_pos;
}

float MotionProfile::get_max_vel(){
    return m_max_vel;
}

float MotionProfile::get_max_accel(){
    return m_max_accel;
}

float MotionProfile::get_target_pos() {
    return m_target_pos;
}