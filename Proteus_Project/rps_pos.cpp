#include <FEHLCD.h>
#include <FEHRPS.h>
#include <cstdio>

#include "timer.h"
#include "rps_pos.h"

RPSPose RPSPositions::pos_list[NUM_RPS_POSE];

const char *RPSPositions::pos_names[NUM_RPS_POSE] = {
    "RPS_FIRST_TURN",
    "RPS_BASKET_LINEUP",
    "RPS_START"
};

float RPSPose::x() {
    return m_x;
}

float RPSPose::y() {
    return m_y;
}

float RPSPose::angle() {
    return m_theta;
}


enum ScreenTapState {
    LAST_PRESS,
    NOT_PRESSED,
    PRESSED,
    RELEASED,
};

void RPSPositions::calibrate() {
    const float refresh_rate = .2;
    float x, y;

    LCD.WriteLine("RPS Calibration\n---------------");
    LCD.WriteLine("Tap screen for\n each position.");

    for (int i = 0; i < NUM_RPS_POSE; ++i) {
        ScreenTapState tap_state = LAST_PRESS;
        LCD.WriteRC("                  ", 2, 0);
        LCD.WriteRC(get_name((RPSPoseType) i), 2, 0);
        Timer refresh;

        refresh.reset();
        while (tap_state != RELEASED) {
            if (refresh.get() > refresh_rate) {
                // Clear Lines
                LCD.WriteRC("                  ", 3, 0);
                LCD.WriteRC("                  ", 4, 0);
                LCD.WriteRC("                  ", 5, 0);

                LCD.WriteRC("X: ", 3, 0);
                LCD.WriteRC(RPS.X(), 3, 2);
                LCD.WriteRC("Y: ", 4, 0);
                LCD.WriteRC(RPS.Y(), 4, 2);
                LCD.WriteRC("Heading: ", 5, 0);
                LCD.WriteRC(RPS.Heading(), 5, 10);
            }

            switch (tap_state) {
            case LAST_PRESS:
                if (!LCD.Touch(&x, &y)) {
                    tap_state = NOT_PRESSED;
                }
                break;

            case NOT_PRESSED:
                if (LCD.Touch(&x, &y)) {
                    tap_state = PRESSED;
                }
                break;
            
            case PRESSED:
                if (!LCD.Touch(&x, &y)) {
                    tap_state = RELEASED;
                }
                break;
            default:
                break;
            }
        }

        RPSPositions::pos_list[i] = RPSPose(RPS.X(), RPS.Y(), RPS.Heading());
        
    }

    LCD.WriteLine("Done Calibrating\n---------------");
}

void RPSPositions::print(RPSPoseType t) {
    const size_t BUF_SIZE = 1024;
    char buf[BUF_SIZE];
    RPSPose pose = RPSPositions::get(t);
    snprintf(buf, BUF_SIZE, "%s:\n\t(%.1f, %.1f, %.1f)", RPSPositions::get_name(t), pose.x(), pose.y(), pose.angle());
    LCD.WriteLine(buf);
}

const char * RPSPositions::get_name(RPSPoseType t) {
    return RPSPositions::pos_names[t];
}

RPSPose RPSPositions::get(RPSPoseType t) {
    return RPSPositions::pos_list[t];
}
