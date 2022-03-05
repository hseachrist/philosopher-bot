#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHMotor.h>
#include <FEHUtility.h>

#include "pid.h"
#include "timer.h"

#define PI 3.14159


#define WHEEL_RADIUS (2.5/2)
// number of encoders per inch
#define ENC_PER_INCH (316/(2*WHEEL_RADIUS*(PI)))
// The distance between the wheels
#define TRACK_WIDTH (7.3)
#define RED_CUTOFF (1.1)
#define BLACK_CUTOFF (2.7)

#define DEBUG_LOG (true)

#define PHIL_LOG(x)     do { \
                            if (DEBUG_LOG) {\
                                LCD.Write(__LINE__); \
                                LCD.Write(": "); \
                                LCD.Write(x); \
                                LCD.WriteLine(""); \
                            } \
                        } while(0);

#define POS_INF (1.0 / 0.0)


// TODO Voltage Rating 
FEHMotor left_motor(FEHMotor::Motor0, 9.0);
FEHMotor right_motor(FEHMotor::Motor1, 9.0);

// Encoders for Motors
DigitalEncoder left_enc(FEHIO::P0_1);
DigitalEncoder right_enc(FEHIO::P1_0);

// Color sensor
AnalogInputPin cds_cell(FEHIO::P2_1);

FEHMotor left_lift(FEHMotor::Motor2, 5.0);
FEHMotor right_lift(FEHMotor::Motor2, 5.0);

// PID Controller Values
float KP = 1;
float KI = .00;
float KD = .00;

// Bump Switches
enum bump_switch_loc {
    BS_FRONT_RIGHT = 0,
    BS_FRONT_LEFT,
    BS_BACK_RIGHT,
    BS_BACK_LEFT,
    BS_LIFT_DOWN,
    BS_LIFT_UP,
    NUM_BS
};

DigitalInputPin bump_switches[NUM_BS] = {
    DigitalInputPin(FEHIO::P0_4),   // BS_FRONT_RIGHT
    DigitalInputPin(FEHIO::P3_7),   // BS_FRONT_LEFT
    DigitalInputPin(FEHIO::P1_7),   // BS_BACK_RIGHT
    DigitalInputPin(FEHIO::P3_0),   // BS_BACK_LEFT
    DigitalInputPin(FEHIO::P3_7),   // BS_LIFT_DOWN
    DigitalInputPin(FEHIO::P2_1),   // BS_LIFT_UP (NON-EXISTANT)
};

enum drive_direction {
    DD_FORE = 1,
    DD_BACK = -1
};

/**
 * Drive forward or backward some distance
 * 
 * - dir    -> Direction to drive in  (either forward or backward)
 * - inches -> distance to drive 
*/
void drive_inch(drive_direction dir, float inches, float power_percent = 25.0, float timeout = POS_INF) {
    PIDController controller(KP, KI, KD);


    float target_pos = inches * ENC_PER_INCH;

    left_enc.ResetCounts();
    right_enc.ResetCounts();

    Timer timer;
    timer.reset();

    PHIL_LOG("Start drive_fore_inch");

    while (left_enc.Counts() < target_pos && timer.get() < timeout) {
        // Find difference in encoder distance and update powers to correct it.
        // TODO: Implement motion profile?
        float power_difference = controller.update(right_enc.Counts(), left_enc.Counts());

        left_motor.SetPercent(power_percent * dir);
        right_motor.SetPercent(-(power_percent + power_difference) * dir);
    }

    PHIL_LOG("End drive_fore_inch");

    left_motor.Stop();
    right_motor.Stop();
    Sleep(.1);
}

void wait_until_touch() {
    LCD.WriteLine("Touch Screen. . .");
    float x, y;
    while(LCD.Touch(&x, &y));
    while(!LCD.Touch(&x, &y));
    while(LCD.Touch(&x, &y));
}

enum turn_direction {
    TD_LEFT = 1,
    TD_RIGHT = -1
};

void turn_degrees(turn_direction turn_dir, float degrees, float power_percent = 30.0, float timeout = POS_INF) {
    float radians = degrees * PI / 180.0;
    float circumference = radians * (TRACK_WIDTH / 2);
    float target_pos = circumference * ENC_PER_INCH;

    PIDController controller(KP/10 + .1, KI, KD);

    left_enc.ResetCounts();
    right_enc.ResetCounts();

    Timer timer;
    timer.reset();

    PHIL_LOG("Start turn_degrees");
    
    while (left_enc.Counts() < target_pos && timer.get() < timeout) {
        float power_difference = controller.update(right_enc.Counts(), left_enc.Counts());

        left_motor.SetPercent(-power_percent * turn_dir);
        right_motor.SetPercent(-(power_percent + power_difference) * turn_dir);
    }

    PHIL_LOG("End turn_degrees");

    left_motor.Stop();
    right_motor.Stop();
    Sleep(.1);
}

bool detected_black() {
    if (cds_cell.Value() > BLACK_CUTOFF) {
        PHIL_LOG("Black Detected")
        PHIL_LOG("Voltage Below")
        PHIL_LOG(cds_cell.Value());
    }

    return cds_cell.Value() > BLACK_CUTOFF;
}



void drive_until_black(float inches) {
    PIDController controller(KP, KI, KD);

    const float TARGET_PERCENT = 25.;

    float target_pos = inches * ENC_PER_INCH;

    left_enc.ResetCounts();
    right_enc.ResetCounts();

    PHIL_LOG("Start drive_until_black");

    while (!detected_black() && left_enc.Counts() < target_pos) {
        float power_difference = 0;

        left_motor.SetPercent(TARGET_PERCENT);
        right_motor.SetPercent(-(TARGET_PERCENT + power_difference));
    }

    PHIL_LOG("End drive_until_black");

    left_motor.Stop();
    right_motor.Stop();
    Sleep(.1);
}

// drive until both bump switches are pressed
void drive_until_bump(drive_direction dir, float inches_cutoff, float percent_power = 30.0, float timeout = POS_INF) {
    PIDController controller(KP, KI, KD);
    const float TARGET_PERCENT = 30.;

    float target_pos = inches_cutoff * ENC_PER_INCH;

    // Check whether to check for front or back switches
    bump_switch_loc right_bs = (dir == DD_FORE) ? BS_FRONT_RIGHT : BS_BACK_RIGHT;
    bump_switch_loc left_bs = (dir == DD_FORE) ? BS_FRONT_LEFT : BS_BACK_LEFT;

    PHIL_LOG("Start drive_until_bump");

    Timer timer;
    timer.reset();

    left_enc.ResetCounts();
    right_enc.ResetCounts();
    
    // While less than tick threshold and the front bumpers aren't pressed
    while (left_enc.Counts() < target_pos && (bump_switches[right_bs].Value() || bump_switches[left_bs].Value()) && timer.get() < timeout) {
        float power_difference = controller.update(right_enc.Counts(), left_enc.Counts());

        if (bump_switches[left_bs].Value()){
            left_motor.SetPercent(percent_power * dir);
        } else {
            left_motor.Stop();
            power_difference = 0;
        }

        if (bump_switches[right_bs].Value()) {
            right_motor.SetPercent(-(percent_power + power_difference) * dir);
        } else {
            right_motor.Stop();
        }
        
    }

    PHIL_LOG("End drive_until_bump");

    left_motor.Stop();
    right_motor.Stop();
    Sleep(.1);
}

void log_cds_cell() {
    while (true) {
        LCD.Clear();
        LCD.Write("Voltage: ");
        LCD.WriteLine(cds_cell.Value());
        Sleep(.1);
    }
}

void log_bump() {
    while (true) {
        LCD.Clear();
        LCD.Write("FrontLeft: ");
        LCD.WriteLine(bump_switches[BS_FRONT_LEFT].Value());
        LCD.Write("FrontRight: ");
        LCD.WriteLine(bump_switches[BS_FRONT_RIGHT].Value());
        LCD.Write("BackLeft: ");
        LCD.WriteLine(bump_switches[BS_BACK_LEFT].Value());
        LCD.Write("BackRight: ");
        LCD.WriteLine(bump_switches[BS_BACK_RIGHT].Value());
        Sleep(.1);
    }
}

void ramp() {
    drive_direction dir = DD_BACK;
    float target_pos = 30.0 * ENC_PER_INCH;
    float power_percent = 80;

    left_enc.ResetCounts();
    right_enc.ResetCounts();

    PHIL_LOG("Start ramp");

    float start_time = TimeNow();

    while (TimeNow() - start_time < .2) {
        left_motor.SetPercent(power_percent * dir);
    }

    while (left_enc.Counts() < target_pos) {
        // Find difference in encoder distance and update powers to correct it.
        // TODO: Implement motion profile?
        float power_difference = 16;

        left_motor.SetPercent(power_percent * dir);
        right_motor.SetPercent(-(power_percent - power_difference) * dir);
    }

    PHIL_LOG("End ramp");

    left_motor.Stop();
    right_motor.Stop();
    Sleep(.1);
}

void drop_basket(float timeout = POS_INF) {
    const float TARGET_POWER = 50.;
    left_lift.SetPercent(TARGET_POWER);
    right_lift.SetPercent(-TARGET_POWER);
    Timer timer;
    timer.reset();
    while (bump_switches[BS_LIFT_DOWN].Value() && timer.get() < timeout);
    left_lift.Stop();
    right_lift.Stop();
}

void lift_basket(float timeout = POS_INF) {
    const float TARGET_POWER = 40;
    left_lift.SetPercent(-TARGET_POWER);
    right_lift.SetPercent(TARGET_POWER + 40);
    Timer timer;
    timer.reset();
    while (bump_switches[BS_LIFT_UP].Value() && timer.get() < timeout);
    left_lift.Stop();
    right_lift.Stop();
}

int main(void)
{
    // Wait for user input
    LCD.SetBackgroundColor(BLACK);
    LCD.Clear();
    LCD.SetFontColor(WHITE);

    PHIL_LOG("Waiting for Start");
    while(cds_cell.Value() > RED_CUTOFF);
    PHIL_LOG("Starting");

    // get ticket
    drive_until_bump(DD_FORE, 20, 30, 5);
    turn_degrees(TD_LEFT, 30);

    // Go to sink
    drive_inch(DD_BACK, 7, 30, 2);
    turn_degrees(TD_LEFT, 90);
    drive_until_bump(DD_BACK, 10);
    drive_inch(DD_FORE, 15, 30, 7);
    turn_degrees(TD_RIGHT, 90, 30, 5);
    drive_inch(DD_FORE, 34, 40, 5);
    turn_degrees(TD_LEFT, 90, 30, 5);
    drive_inch(DD_FORE, 6, 30, 5);
    turn_degrees(TD_LEFT, 90, 30, 5);
    drive_until_bump(DD_FORE, 6, 80, 3);
    drive_inch(DD_BACK, 6, 80, 3);
    drive_until_bump(DD_FORE, 6, 80, 3);
    drop_basket(2);
    LCD.WriteLine("I think. Therefore. I AM!");
    drive_inch(DD_BACK, 10, 80);
    turn_degrees(TD_RIGHT, 90);
    drive_inch(DD_BACK, 22);
    turn_degrees(TD_RIGHT, 90);
    drive_inch(DD_FORE, 22);

/*
    drive_inch(DD_BACK, 6);
    turn_degrees(TD_LEFT, 90);
    drive_until_bump(DD_BACK, 10);
    drive_inch(DD_FORE, 15);
    turn_degrees(TD_RIGHT, 92);
    drive_until_bump(DD_BACK, 30);
    drive_inch(DD_FORE, 12);
    turn_degrees(TD_RIGHT, 180);s
    drive_until_bump(DD_BACK, 50, 75);

    // Navigate to Sink
    drive_inch(DD_FORE, 8);
    turn_degrees(TD_LEFT, 90);
    drive_until_bump(DD_BACK, 25);
    drive_inch(DD_FORE, 2);
    turn_degrees(TD_RIGHT, 90);
    drop_basket(4);


    // Go to burger flipper
    drive_inch(DD_BACK, 3);
    turn_degrees(TD_RIGHT, 45);
    drive_inch(DD_BACK, 22);
    */
    return 0;
}