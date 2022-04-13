#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHMotor.h>
#include <FEHUtility.h>
#include <FEHRPS.h>
#include <FEHBuzzer.h>
#include <cmath>

#include "pid.h"
#include "rps_pos.h"
#include "timer.h"
#include "util.h"

#define PI 3.14159

#define WHEEL_RADIUS (2.5/2)
// number of encoders per inch
#define ENC_PER_INCH (316/(2*WHEEL_RADIUS*(PI)))
// The distance between the wheels
#define TRACK_WIDTH (7.3)
#define QR_CENTER_OF_ROT_DIST (2.5)
#define RED_CUTOFF (0.6)
#define BLACK_CUTOFF (3.1)

#define DEBUG_LOG (true)

#define PHIL_LOG(x)     do { \
                            if (DEBUG_LOG) {\
                                LCD.Write(__LINE__); \
                                LCD.Write(": "); \
                                LCD.Write(x); \
                                LCD.WriteLine(""); \
                            } \
                        } while(0);


#define RPS_WAIT_TIME_IN_SEC 0.35

// Shaft encoding counts for CrayolaBots
#define COUNTS_PER_INCH 40.5
#define COUNTS_PER_DEGREE 2.48

// Defines for pulsing the robot
#define PULSE_TIME .08
#define PULSE_POWER 20

// Define for the motor power
#define POWER 30

// Orientation of QR Code
#define PLUS 0
#define MINUS 1

// TODO Voltage Rating 
FEHMotor left_motor(FEHMotor::Motor1, 9.0);
FEHMotor right_motor(FEHMotor::Motor0, 9.0);

// Encoders for Motors
DigitalEncoder left_enc(FEHIO::P1_0);
DigitalEncoder right_enc(FEHIO::P0_1);

// Color sensor

enum cds_cell_loc {
    CDS_LEFT = 0,
    CDS_RIGHT,
    CDS_CENTER,
    NUM_CDS
};

AnalogInputPin cds_cell[NUM_CDS] = {
    AnalogInputPin(FEHIO::P3_3), // CDS_LEFT
    AnalogInputPin(FEHIO::P1_4), // CDS_RIGHT
    AnalogInputPin(FEHIO::P2_2), // CDS_CENTER
};

FEHMotor left_lift(FEHMotor::Motor2, 5.0);
FEHMotor right_lift(FEHMotor::Motor3, 5.0);

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
    DigitalInputPin(FEHIO::P0_7),   // BS_BACK_RIGHT
    DigitalInputPin(FEHIO::P3_0),   // BS_BACK_LEFT
    DigitalInputPin(FEHIO::P3_7),   // BS_LIFT_DOWN
    DigitalInputPin(FEHIO::P2_1),   // BS_LIFT_UP (NON-EXISTANT)
};

void rps_log_once() {
    LCD.Write("X: ");
    LCD.WriteLine(RPS.X());
    LCD.Write("Y: ");
    LCD.WriteLine(RPS.Y());
    LCD.Write("Heading: ");
    LCD.WriteLine(RPS.Heading());
}

void rps_log() {
    while(true) {
        LCD.Clear();
        LCD.Write("X: ");
        LCD.WriteLine(RPS.X());
        LCD.Write("Y: ");
        LCD.WriteLine(RPS.Y());
        LCD.Write("Heading: ");
        LCD.WriteLine(RPS.Heading());
        Sleep(.2);
    }
}

/*
 * Pulse forward a short distance using time
 */
void pulse_forward(int percent, float seconds) 
{
    // Set both motors to desired percent
    right_motor.SetPercent(-percent);
    left_motor.SetPercent(percent);

    // Wait for the correct number of seconds
    Sleep(seconds);

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

void invalid_rps_tone() {
        Buzzer.Tone(FEHBuzzer::E4, .125);
        Buzzer.Tone(FEHBuzzer::Bf4, .125);
}

/*
 * Pulse counterclockwise a short distance using time
 */
void pulse_counterclockwise(int percent, float seconds) 
{
    // Set both motors to desired percent
    right_motor.SetPercent(-percent);
    left_motor.SetPercent(-percent);

    // Wait for the correct number of seconds
    Sleep(seconds);

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

/*
 * Pulse clockwise a short distance using time
 */
void pulse_clockwise(int percent, float seconds) 
{
    // Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);

    // Wait for the correct number of seconds
    Sleep(seconds);

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

bool rps_valid() {
    return RPS.X() >= 0 && RPS.Y() >= 0 && RPS.Heading() >= 0;
}

bool in_deadzone() {
    return RPS.X() <= -1.5 || RPS.Y() <= -1.5 || RPS.Heading() <= -1.5;
}

/* 
 * Use RPS to move to the desired heading
 */
void check_heading(float heading, float timeout = INFINITY)
{

    const float threshold = 1.5;
    const float NO_DETECT = 1.0;

    //You will need to fill out this one yourself and take into account
    //checking for proper RPS data and the edge conditions
    //(when you want the robot to go to 0 degrees or close to 0 degrees)

    /*
        SUGGESTED ALGORITHM:
        1. Check the current orientation of the QR code and the desired orientation of the QR code
        2. Check if the robot is within the desired threshold for the heading based on the orientation
        3. Pulse in the correct direction based on the orientation
    */

    Timer no_detect_timer, timeout_timer;
    no_detect_timer.reset();
    timeout_timer.reset();
    while (no_detect_timer.get() < NO_DETECT && timeout_timer.get() < timeout) {

        LCD.Clear();
        if (RPS.Heading() < 0) {
            LCD.WriteLine("Can't Detect QR Code");
            continue;
        }
        no_detect_timer.reset();

        float current_heading = RPS.Heading();
        float current_heading_phased_right = current_heading - 360;
        float current_heading_phased_left = current_heading + 360;

        float diff = heading - current_heading;
        float diff_phased_right = heading - current_heading_phased_right;
        float diff_phased_left = heading - current_heading_phased_right;

        if (abs(diff_phased_right) < abs(diff)) {
            diff = diff_phased_right;
        }

        if (abs(diff_phased_left) < abs(diff)) {
            diff = diff_phased_left;
        }

        
        if (diff > threshold) {
            // target heading is to the left of current heading
            pulse_counterclockwise(PULSE_POWER, PULSE_TIME);
        } else if (diff < -threshold) {
            //target heading is to the right of current heading
            pulse_clockwise(PULSE_POWER, PULSE_TIME);
        } else {
            break;
        }

        Sleep(RPS_WAIT_TIME_IN_SEC);
    }
}

/* 
 * Use RPS to move to the desired x_coordinate based on the orientation of the QR code
 */
void check_x(float x_coordinate, int orientation, float heading = INFINITY, float timeout = INFINITY)
{

    const float NO_DETECT = 2.0;

    // Determine the direction of the motors based on the orientation of the QR code 
    int power = PULSE_POWER;
    if(orientation == MINUS){
        power = -PULSE_POWER;
    }

    float threshold = .5, target_heading;

    if (heading > 360) {
        target_heading = (RPS.Heading() < 270 && RPS.Heading() > 90) ? 180 : 0;
    } else {
        target_heading = heading;
    }

    Timer no_detect_timer, timeout_timer;
    no_detect_timer.reset();
    timeout_timer.reset();

    // Check if receiving proper RPS coordinates and whether the robot is within an acceptable range
    while((RPS.X() < x_coordinate - threshold || RPS.X() > x_coordinate + threshold) && no_detect_timer.get() < NO_DETECT && timeout_timer.get() < timeout)
    {
        if (!rps_valid()) {
            no_detect_timer.reset();
            continue;
        }

        rps_log_once();
        if(RPS.X() < x_coordinate)
        {
            LCD.WriteLine("Going Backwards");
            // Pulse the motors for a short duration in the correct direction
            pulse_forward(-power, PULSE_TIME);
        }
        else if(RPS.X() > x_coordinate)
        {
            LCD.WriteLine("Going Forwards");
            // Pulse the motors for a short duration in the correct direction
            pulse_forward(power, PULSE_TIME);
        }

        check_heading(target_heading, 8);

        Sleep(RPS_WAIT_TIME_IN_SEC);
    }
}


/* 
 * Use RPS to move to the desired y_coordinate based on the orientation of the QR code
 */
void check_y(float y_coordinate, int orientation, float heading = INFINITY, float timeout = INFINITY)
{
    const float NO_DETECT = 2.0;

    // Determine the direction of the motors based on the orientation of the QR code
    int power = PULSE_POWER;
    if(orientation == MINUS){
        power = -PULSE_POWER;
    }

    float threshold = .25, target_heading;

    if (heading > 360) {
        target_heading = (RPS.Heading() < 360 && RPS.Heading() > 180) ? 270 : 90;
    } else {
        target_heading = heading;
    }

    Timer no_detect_timer, timeout_timer;
    no_detect_timer.reset();
    timeout_timer.reset();

    // Check if receiving proper RPS coordinates and whether the robot is within an acceptable range
    while((RPS.Y() < y_coordinate - threshold || RPS.Y() > y_coordinate + threshold) && no_detect_timer.get() < NO_DETECT && timeout_timer.get() < timeout)
    {
        if (!rps_valid()) {
            LCD.Clear();
            LCD.WriteLine("RPS Invalid");
            continue;
        }

        if(RPS.Y() > y_coordinate)
        {
            // Pulse the motors for a short duration in the correct direction
            pulse_forward(-power, PULSE_TIME);
        }
        else if(RPS.Y() < y_coordinate)
        {
            // Pulse the motors for a short duration in the correct direction
           pulse_forward(power, PULSE_TIME);
        }

        check_heading(target_heading, 8);
        
        Sleep(RPS_WAIT_TIME_IN_SEC);

        LCD.Write("Target: ");
        LCD.WriteLine(y_coordinate);
        LCD.Write("Current: ");
        LCD.WriteLine(RPS.Y());

    }
}

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
void drive_inch(drive_direction dir, float inches, float power_percent = 40.0, float timeout = INFINITY, float angle = INFINITY) {
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
        power_difference = 5 * sigmoid(power_difference);

        left_motor.SetPercent(power_percent * dir);
        right_motor.SetPercent(-(power_percent + power_difference) * dir);
    }

    if (angle <= 360 && rps_valid()) {
        check_heading(angle, 8);
    }

    PHIL_LOG("End drive_fore_inch");

    left_motor.Stop();
    right_motor.Stop();
    Sleep(.1);
}

float min_cds() {
    float min = INFINITY;
    for (int i = 0; i < NUM_CDS; ++i) {
        min = (min > cds_cell[i].Value()) ? cds_cell[i].Value() : min;
    }

    return min;
}

float drive_inch_min_cds(drive_direction dir, float inches, float power_percent = 40.0, float timeout = INFINITY, float angle = INFINITY) {
    PIDController controller(KP, KI, KD);


    float target_pos = inches * ENC_PER_INCH;

    left_enc.ResetCounts();
    right_enc.ResetCounts();
    float min_cds_val;

    Timer timer;
    timer.reset();

    PHIL_LOG("Start drive_fore_inch");

    while (left_enc.Counts() < target_pos && timer.get() < timeout) {
        // Find difference in encoder distance and update powers to correct it.
        // TODO: Implement motion profile?
        float power_difference = controller.update(right_enc.Counts(), left_enc.Counts());
        power_difference = 5 * sigmoid(power_difference);

        left_motor.SetPercent(power_percent * dir);
        right_motor.SetPercent(-(power_percent + power_difference) * dir);

        min_cds_val = (min_cds() < min_cds_val) ? min_cds() : min_cds_val;
    }

    if (angle <= 360 && rps_valid()) {
        check_heading(angle, 8);
    }

    PHIL_LOG("End drive_fore_inch");

    left_motor.Stop();
    right_motor.Stop();
    Sleep(.1);

    return min_cds_val;
}

float max_cds() {
    float max = 0.0;
    for (int i = 0; i < NUM_CDS; ++i) {
        max = (max < cds_cell[i].Value()) ? cds_cell[i].Value() : max;
    }

    return max;
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

void turn_degrees(turn_direction turn_dir, float degrees, float power_percent = 30.0, float timeout = INFINITY) {
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
        power_difference = 5 * sigmoid(power_difference);

        left_motor.SetPercent(-power_percent * turn_dir);
        right_motor.SetPercent(-(power_percent + power_difference) * turn_dir);
    }

    PHIL_LOG("End turn_degrees");

    left_motor.Stop();
    right_motor.Stop();
    Sleep(.1);
}

void turn_to_angle(float angle, float modifier = 1.0, float power_percent = 30.0, float backup_angle = -1, float timeout = INFINITY) {
    float angle_to_turn = 0.0;
    if (!rps_valid()) {
        invalid_rps_tone();
        if (backup_angle > 0) {
            angle_to_turn = backup_angle;
        } else {
            angle_to_turn = angle;
        }
    } else {
        float current_heading = RPS.Heading();
        float current_heading_phased_right = current_heading - 360;
        float current_heading_phased_left = current_heading + 360;

        float diff = angle - current_heading;
        float diff_phased_right = angle - current_heading_phased_right;
        float diff_phased_left = angle - current_heading_phased_right;

        if (abs(diff_phased_right) < abs(diff)) {
            diff = diff_phased_right;
        }

        if (abs(diff_phased_left) < abs(diff)) {
            diff = diff_phased_left;
        }

        diff *= modifier;

        angle_to_turn = diff;
    }

    turn_direction dir = TD_LEFT;
    if (angle_to_turn < 0) {
        angle_to_turn *= -1;
        dir = TD_RIGHT;
    }

    turn_degrees(dir, angle_to_turn, power_percent, timeout);
    if (rps_valid()) {
        check_heading(angle, 8);
    } else {
        
    }
}

bool detected_black() {
    if (min_cds() > BLACK_CUTOFF) {
        PHIL_LOG("Black Detected")
        PHIL_LOG("Voltage Below")
        PHIL_LOG(min_cds());
    }

    return min_cds() > BLACK_CUTOFF;
}



void drive_until_black(float inches) {
    PIDController controller(KP, KI, KD);

    const float TARGET_PERCENT = 40;

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
void drive_until_bump(drive_direction dir, float inches_cutoff, float percent_power = 30.0, float timeout = INFINITY) {
    PIDController controller(KP, KI, KD);
    const float TARGET_PERCENT = 30.;

    float target_pos = inches_cutoff * ENC_PER_INCH;

    // Check whether to check for front or back switches
    bump_switch_loc right_bs = (dir == DD_FORE) ? BS_FRONT_RIGHT : BS_BACK_RIGHT;
    bump_switch_loc left_bs = (dir == DD_FORE) ? BS_FRONT_LEFT : BS_BACK_LEFT;

    PHIL_LOG("Start drive_until_bump");

    Timer timer;
    timer.reset();

    bool left_pressed = false, right_pressed = false;

    left_enc.ResetCounts();
    right_enc.ResetCounts();
    
    // While less than tick threshold and the front bumpers aren't pressed
    while (left_enc.Counts() < target_pos && !(right_pressed && left_pressed) && timer.get() < timeout) {
        float power_difference = controller.update(right_enc.Counts(), left_enc.Counts());
        power_difference = 5 * sigmoid(power_difference);

        if (bump_switches[left_bs].Value()){
            left_motor.SetPercent(percent_power * dir);
        } else {
            left_motor.Stop();
            left_pressed = true;
        }

        if (left_pressed) power_difference = 0;

        if (bump_switches[right_bs].Value()) {
            right_motor.SetPercent(-(percent_power + power_difference) * dir);
        } else {
            right_motor.Stop();
            power_difference = 0;
            right_pressed = true;
        }
        
    }

    left_enc.ResetCounts();
    right_enc.ResetCounts();

    PHIL_LOG("End drive_until_bump");

    left_motor.Stop();
    right_motor.Stop();
    Sleep(.1);
}

void log_cds_cell() {
    const char cds_names [NUM_CDS][20] = {"CDS_LEFT", "CDS_RIGHT", "CDS_CENTER"};
    while (true) {
        LCD.Clear();
        for (int i = 0; i < NUM_CDS; ++i) {
            LCD.Write(cds_names[i]);
            LCD.Write(": ");
            LCD.WriteLine(cds_cell[i].Value());
        }
        Sleep(.2);
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

void drop_basket(float timeout = INFINITY, float power = 50., bool check_switch = true) {
    left_lift.SetPercent(-power);
    right_lift.SetPercent(power);
    Timer timer;
    timer.reset();
    while (timer.get() < timeout) {
        if (check_switch && !bump_switches[BS_LIFT_DOWN].Value()) {
            break;
        }
    }
    left_lift.Stop();
    right_lift.Stop();
}

void drop_basket_async(float power = 60.) {
    left_lift.SetPercent(-power);
    right_lift.SetPercent(power);
}


void lift_basket(float timeout = INFINITY, float power = 60.) {
    left_lift.SetPercent(power);
    right_lift.SetPercent(-power);
    Timer timer;
    timer.reset();
    while (bump_switches[BS_LIFT_UP].Value() && timer.get() < timeout);
    left_lift.Stop();
    right_lift.Stop();
}

void lift_basket_async(float power = 60.) {
    left_lift.SetPercent(power);
    right_lift.SetPercent(-power);
}

enum basket_direction {
    BD_LIFT = 1,
    BD_DROP = -1,
};

void idle_basket(basket_direction dir = BD_DROP) {
    const float TARGET_POWER = 4;
    left_lift.SetPercent(TARGET_POWER * dir);
    right_lift.SetPercent(-TARGET_POWER * dir);
}

void move_basket_away_from_qr() {
    const float TARGET_POWER = 50.;
    while (RPS.X() < 0) {
        left_lift.SetPercent(-TARGET_POWER);
        right_lift.SetPercent(TARGET_POWER);
    }
    Sleep(.2);
    idle_basket(BD_LIFT);
}

void stop_lift() {
    left_lift.Stop();
    right_lift.Stop();
}

void drive_until_deadzone(drive_direction dir, float inches, float angle = INFINITY) {
    PIDController controller(KP, KI, KD);

    const float TARGET_PERCENT = 35;

    float target_pos = inches * ENC_PER_INCH;

    left_enc.ResetCounts();
    right_enc.ResetCounts();

    PHIL_LOG("Start drive_until_deadzone");

    while (!in_deadzone() && left_enc.Counts() < target_pos) {
        float power_difference = 0;

        left_motor.SetPercent(TARGET_PERCENT);
        right_motor.SetPercent(-(TARGET_PERCENT + power_difference));
    }

    if (angle <= 360 && rps_valid()) {
        check_heading(angle, 8);
    }

    PHIL_LOG("End drive_until_deadzone");

    left_motor.Stop();
    right_motor.Stop();
    Sleep(.1);
}

void drive_until_no_deadzone(drive_direction dir, float inches = INFINITY, float angle = INFINITY) {
    PIDController controller(KP, KI, KD);

    const float TARGET_PERCENT = 25.;

    float target_pos = inches * ENC_PER_INCH;

    left_enc.ResetCounts();
    right_enc.ResetCounts();

    PHIL_LOG("Start drive_until_black");

    while (in_deadzone() && left_enc.Counts() < target_pos) {
        float power_difference = 0;

        left_motor.SetPercent(TARGET_PERCENT * dir);
        right_motor.SetPercent(-(TARGET_PERCENT + power_difference) * dir);
    }

    PHIL_LOG("End drive_until_black");

    left_motor.Stop();
    right_motor.Stop();
    Sleep(.1);
}

/*
* --------------
*   GO UP RAMP
* --------------
*/

void ramp() {
    RPSPose target_pose = RPSPositions::get(RPS_FIRST_TURN);
    drive_inch(DD_FORE, 14, 50);
    check_x(target_pose.x(), PLUS, RPSPositions::get(RPS_START).angle());
    turn_to_angle(target_pose.angle(), .75, 40);
}

void sink() {
    RPSPose target_pose = RPSPositions::get(RPS_BASKET_LINEUP);
    drive_inch(DD_FORE, 34, 40);
    if (in_deadzone()) {
        drive_until_no_deadzone(DD_BACK, 2);
    }
    check_y(target_pose.y(), PLUS);
    turn_to_angle(target_pose.angle(), .8, 40);

    float angle_turn;
    if (rps_valid()) {
        angle_turn = (230 - RPS.Heading()) * 85./90;
    } else {
        invalid_rps_tone();
        angle_turn = 50;
    }
    drive_inch(DD_FORE, 3);
    
    turn_degrees(TD_LEFT, angle_turn);
    if (rps_valid()) {
        check_heading(240, 8);
    } else {
        invalid_rps_tone();
    }
    drive_until_bump(DD_FORE, 5, 30, 1);
    stop_lift();
    LCD.WriteLine("Dropping basket");
    drop_basket(2, 70, false);

    LCD.WriteLine("Lifting Basket");
    lift_basket_async();
}


/*
* --------------
*   GO TO HOTPLATE
* --------------
*/
void hotplate() {
    Timer timer;
    timer.reset();
    RPSPose target_pose = RPSPositions::get(RPS_STOVE_LIFT); 
    float angle_turn;
    if (rps_valid()) {
        angle_turn = (RPS.Heading() - 180) * 85./90;
    } else {
        angle_turn = 80;
        invalid_rps_tone();
    }
    // drive back from sink and lift the arm
    drive_inch(DD_BACK, 4, 40);
    while (timer.get() < 2.5);
    stop_lift();

    // Turn and go to right wall
    turn_degrees(TD_RIGHT, angle_turn);
    drop_basket_async();
    timer.reset();
    drive_inch(DD_BACK, 10);
    check_heading(180);
    while (timer.get() < 2);
    drive_until_bump(DD_BACK, 40, 50, 4);
    if (RPS.Heading() > 210) {
        turn_to_angle(180);
    }
    
    // Line up with Hot plate
    float current_pose = RPS.X();
    float inches_to_drive = (current_pose - (target_pose.x() - QR_CENTER_OF_ROT_DIST)) * .9;
    drive_inch(DD_FORE, inches_to_drive);
    check_x(target_pose.x() - QR_CENTER_OF_ROT_DIST, PLUS);
    turn_to_angle(target_pose.angle(), 1, 40);

    current_pose = RPS.Y();
    inches_to_drive = (target_pose.y() - current_pose) * .65;
    drive_inch(DD_FORE, inches_to_drive);
    check_y(target_pose.y(), PLUS);
    
    check_heading(target_pose.angle());

    // Lift hotplate
    lift_basket(.5, 80);
    drive_inch(DD_FORE, 3, 25, 2);
    turn_degrees(TD_RIGHT, 30, 80.0);
    lift_basket(.5, 90);
    drive_inch(DD_FORE, 3, 25, 1);
}


/*
* --------------
*   JUKEBOX
* --------------
*/
void jukebox() {
    Timer timer;

    float current_heading = RPS.Heading();
    drive_inch(DD_BACK, .5, 40);

    drop_basket_async(20);
    timer.reset();
    turn_degrees(TD_LEFT, 30, 70, 2);
    drive_inch(DD_BACK, 6);

    // Turn and go to right wall
    if (in_deadzone()) {
        drive_until_no_deadzone(DD_FORE, 2);
    }
    
    RPSPose target_pose = RPSPositions::get(RPS_BASKET_LINEUP);
    turn_to_angle(RPSPositions::get(RPS_STOVE_LIFT).angle());
    float inches_to_drive = RPS.Y() - (target_pose.y() - 4);
    drive_inch(DD_BACK, inches_to_drive);
    check_y(target_pose.y() - 4, PLUS, RPSPositions::get(RPS_STOVE_LIFT).angle());
    turn_to_angle(target_pose.angle(), .9, 70, -50);
    drive_until_no_deadzone(DD_BACK);

    drop_basket_async();
    timer.reset();
    drive_inch(DD_BACK, 8);
    check_heading(180, 8);
    while (timer.get() < 2);
    drive_until_bump(DD_BACK, 40, 50, 10);

    drive_inch(DD_FORE, 1);
    check_heading(target_pose.angle());
    target_pose = RPSPositions::get(RPS_FIRST_TURN); // We want to go to the same location as the initial turn, so we use that position
    drive_until_no_deadzone(DD_BACK);
    drive_inch(DD_FORE, RPS.X() - (target_pose.x() - .5), 50);
    check_x(target_pose.x() - .5, PLUS, INFINITY, 4);
    turn_to_angle(target_pose.angle());

    // Drive to bottom of ramp
    drive_until_no_deadzone(DD_BACK);
    inches_to_drive = (RPS.Y() - target_pose.y() + QR_CENTER_OF_ROT_DIST) * .80;
    drive_inch(DD_BACK, inches_to_drive);
    check_y(target_pose.y() + QR_CENTER_OF_ROT_DIST, PLUS);

    // turn to the black line
    turn_to_angle(target_pose.angle() + 90);
    drop_basket_async();
    drive_until_black(8);
    stop_lift();
    drive_inch(DD_FORE, QR_CENTER_OF_ROT_DIST);
    target_pose = RPSPositions::get(RPS_JUKEBOX);
    check_x(target_pose.x() - 1, PLUS);

    // turn towards the jukebox
    float angle_turn = (target_pose.angle() - RPS.Heading()) * 80./90;
    turn_degrees(TD_LEFT, angle_turn);
    check_heading(target_pose.angle(), 8);

    // go to light
    inches_to_drive = (RPS.Y() - target_pose.y()) * .9;
    drive_inch(DD_FORE, inches_to_drive);
    turn_to_angle(target_pose.angle() + 3, .9, 30, 3);
    drive_inch(DD_FORE, 5, 60, 2);
}


/*
* --------------
*   TICKET
* --------------
*/
void ticket() {
    // drive to other wall
    RPSPose target_pose = RPSPositions::get(RPS_JUKEBOX);
    drive_inch(DD_BACK, .5);
    turn_to_angle(target_pose.angle());
    drive_inch(DD_BACK, 3);

    turn_to_angle(180, .9);
    drive_until_bump(DD_BACK, 40, 60, 6);
    lift_basket(.3);
    
    // turn to ticket
    target_pose = RPSPositions::get(RPS_TICKET);
    drive_inch(DD_FORE, RPS.X() - target_pose.x());
    check_x(target_pose.x(), PLUS);
    turn_to_angle(target_pose.angle());
    
    // drive to ticket
    drive_until_bump(DD_FORE, 8, 60, 1.5);
    drive_inch(DD_BACK, 2.5);
    lift_basket(.3);
    
    // move ticket
    idle_basket(BD_DROP);
    turn_degrees(TD_LEFT, 80, 50, .8);
    turn_degrees(TD_RIGHT, 90, 50, .8);
    drive_inch(DD_BACK, 1, 60);
    turn_to_angle(target_pose.angle(), 1, 30, -1, 5);
    drive_inch(DD_BACK, 1, 60);
    check_heading(RPSPositions::get(RPS_FIRST_TURN).angle(), 4);
    drive_inch(DD_BACK, 3, 60);
}


/*
* -------------------
*   FINAL BUTTON
* -------------------
*/
void final_button() {
    RPSPose target_pose = RPSPositions::get(RPS_TICKET);
    while (true) {
        drive_until_bump(DD_BACK, 20, 70, 3);
        drive_inch(DD_FORE, RPS.X() - target_pose.x());
        check_x(target_pose.x(), PLUS);
        turn_to_angle(target_pose.angle());
    }
}

int main(void)
{
    LCD.SetBackgroundColor(BLACK);
    LCD.Clear();
    LCD.SetFontColor(WHITE);
    RPS.InitializeTouchMenu();

    RPSPositions::calibrate();
    idle_basket(BD_DROP);
    wait_until_touch(); // Final Action

    Timer timer;
    timer.reset();
    PHIL_LOG("Waiting for Start");
    while(cds_cell[CDS_LEFT].Value() > RED_CUTOFF && timer.get() < 30.0);
    PHIL_LOG("Starting");
    
    ramp();
    sink();
    hotplate();
    jukebox();
    ticket();
    final_button();


    return 0;
}
