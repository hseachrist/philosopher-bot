#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <FEHSD.h>

#include <cmath>

// RPS Delay time
#define RPS_WAIT_TIME_IN_SEC 0.35

// Shaft encoding counts for CrayolaBots
#define COUNTS_PER_INCH 40.5
#define COUNTS_PER_DEGREE 2.48

// Defines for pulsing the robot
#define PULSE_TIME .1
#define PULSE_POWER .3

// Define for the motor power
#define POWER .3

// Orientation of QR Code
#define PLUS 0
#define MINUS 1

//Declarations for encoders & motors
DigitalEncoder right_encoder(FEHIO::P0_0);
DigitalEncoder left_encoder(FEHIO::P0_1);
FEHMotor right_motor(FEHMotor::Motor1, 9.0);
FEHMotor left_motor(FEHMotor::Motor0, 9.0);

/*
 * Pulse forward a short distance using time
 */
void pulse_forward(int percent, float seconds) 
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

/*
 * Pulse counterclockwise a short distance using time
 */
void pulse_counterclockwise(int percent, float seconds) 
{
    // Set both motors to desired percent
    right_motor.SetPercent(percent);
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
    right_motor.SetPercent(-percent);
    left_motor.SetPercent(percent);

    // Wait for the correct number of seconds
    Sleep(seconds);

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

/*
 * Move forward using shaft encoders where percent is the motor percent and counts is the distance to travel
 */
void move_forward(int percent, int counts) //using encoders
{
    // Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    // Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);

    // While the average of the left and right encoder are less than counts,
    // keep running motors
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts);

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

/*
 * Turn counterclockwise using shaft encoders where percent is the motor percent and counts is the distance to travel
 */
void turn_counterclockwise(int percent, int counts) 
{
    // Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    // Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(-percent);

    // While the average of the left and right encoder are less than counts,
    // keep running motors
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts);

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

bool rps_valid() {
    return RPS.X() >= 0 && RPS.Y() >= 0 && RPS.Heading() >= 0;
}
/* 
 * Use RPS to move to the desired x_coordinate based on the orientation of the QR code
 */
void check_x(float x_coordinate, int orientation)
{
    // Determine the direction of the motors based on the orientation of the QR code 
    int power = PULSE_POWER;
    if(orientation == MINUS){
        power = -PULSE_POWER;
    }

    // Check if receiving proper RPS coordinates and whether the robot is within an acceptable range
    while(rps_valid() && (RPS.X() < x_coordinate - 1 || RPS.X() > x_coordinate + 1))
    {
        if(RPS.X() > x_coordinate)
        {
            // Pulse the motors for a short duration in the correct direction
            pulse_forward(-power, PULSE_TIME);
        }
        else if(RPS.X() < x_coordinate)
        {
            // Pulse the motors for a short duration in the correct direction
            pulse_forward(power, PULSE_TIME);
        }
        Sleep(RPS_WAIT_TIME_IN_SEC);
    }
}


/* 
 * Use RPS to move to the desired y_coordinate based on the orientation of the QR code
 */
void check_y(float y_coordinate, int orientation)
{
    // Determine the direction of the motors based on the orientation of the QR code
    int power = PULSE_POWER;
    if(orientation == MINUS){
        power = -PULSE_POWER;
    }

    // Check if receiving proper RPS coordinates and whether the robot is within an acceptable range
    while(rps_valid() && (RPS.Y() < y_coordinate - 1 || RPS.Y() > y_coordinate + 1))
    {
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
        Sleep(RPS_WAIT_TIME_IN_SEC);
    }
}

/* 
 * Use RPS to move to the desired heading
 */
void check_heading(float heading)
{

    const float threshold = .4;

    //You will need to fill out this one yourself and take into account
    //checking for proper RPS data and the edge conditions
    //(when you want the robot to go to 0 degrees or close to 0 degrees)

    /*
        SUGGESTED ALGORITHM:
        1. Check the current orientation of the QR code and the desired orientation of the QR code
        2. Check if the robot is within the desired threshold for the heading based on the orientation
        3. Pulse in the correct direction based on the orientation
    */

    while (true) {
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
            // target heading is to the right of current heading
            pulse_clockwise(PULSE_POWER, PULSE_TIME);
        } else {
            break;
        }

        Sleep(RPS_WAIT_TIME_IN_SEC);
    }
}

int main(void)
{
    float touch_x,touch_y;
    float A_x, A_y, B_x, B_y, C_x, C_y, D_x, D_y;
    float A_heading, B_heading, C_heading, D_heading;
    int B_C_counts, C_D_counts, turn_90_counts;

    RPS.InitializeTouchMenu();

    LCD.WriteLine("RPS & Data Logging Test");
    LCD.WriteLine("Press Screen To Start");
    while(!LCD.Touch(&touch_x,&touch_y));
    while(LCD.Touch(&touch_x,&touch_y));

    A_x = 29.1;
    A_y = 51.1;
    B_x = 29.1;
    B_y = 61.1;
    C_x = -2;
    C_y = -2;
    D_x = 13;
    D_y = 51;
    
    
    // WRITE CODE HERE TO SET THE HEADING DEGREES AND COUNTS VALUES
    A_heading = 90;
    B_heading = 180;
    C_heading = 270;
    D_heading = 0;

    B_C_counts = COUNTS_PER_INCH * 16;
    C_D_counts = COUNTS_PER_INCH * 10;

    turn_90_counts = COUNTS_PER_DEGREE * 90;

    // A --> B
    LCD.WriteLine("A -> B: Check Y");
    check_y(B_y, PLUS);
    LCD.WriteLine("A -> B: Check Heading");
    check_heading(B_heading);

    // B --> C
    LCD.WriteLine("B -> C: Move forward");
    move_forward(POWER, B_C_counts);
    LCD.WriteLine("B -> C: Check X");
    check_x(C_x, MINUS);
    LCD.WriteLine("B -> C: Turn");
    turn_counterclockwise(POWER, turn_90_counts);
    LCD.WriteLine("B -> C: Check Heading");
    check_heading(C_heading);

    // C --> D
    LCD.WriteLine("C -> D: Move forward");
    move_forward(POWER, C_D_counts);
    LCD.WriteLine("C -> D: Check Y");
    check_y(D_y, MINUS);
    LCD.WriteLine("C -> D: T");
    turn_counterclockwise(POWER, turn_90_counts);
    LCD.WriteLine("C -> D: Check Heading");
    check_heading(D_heading);

    return 0;
}