#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHMotor.h>
#include <FEHUtility.h>

#define PI 3.14159

#define KP .00
#define WHEEL_RADIUS (2.5/2)
// number of encoders per inch
#define ENC_PER_INCH (316/(2*WHEEL_RADIUS*(PI)))
// The distance between the wheels
#define TRACK_WIDTH (6.90625)
#define RED_CUTOFF (1.1)
#define BLACK_CUTOFF (2.7)

#define PHIL_LOG(x)     do { \
                            LCD.Write(__LINE__); \
                            LCD.Write(": "); \
                            LCD.Write(x); \
                            LCD.WriteLine(""); \
                        } while(0);

// AKA a Proportional controller
// Use with the drivetrain to make sure the the wheels have driven the about the same distance
// So if for whatever reason one motor is going slower than the other, it will speed up that motor
class p_controller {
public:
    p_controller() {};
    p_controller(float kP) : m_kP(kP) {}

    float update(float current, float target) {
        float error = target - current;
        return m_kP * error;
    }

private:
    float m_kP = 0;

};

// TODO Voltage Rating 
FEHMotor left_motor(FEHMotor::Motor0, 9.0);
FEHMotor right_motor(FEHMotor::Motor1, 9.0);

// Encoders for Motors
DigitalEncoder left_enc(FEHIO::P0_1);
DigitalEncoder right_enc(FEHIO::P1_0);

// Color sensor
AnalogInputPin cds_cell(FEHIO::P2_1);

// Bump Switches
enum bump_switch_loc {
    BS_FRONT_RIGHT = 0,
    BS_FRONT_LEFT,
    BS_BACK_RIGHT,
    BS_BACK_LEFT,
    NUM_BS
};
DigitalInputPin bump_switches[NUM_BS] = {DigitalInputPin(FEHIO::P1_0), DigitalInputPin(FEHIO::P2_0), DigitalInputPin(FEHIO::P0_2), DigitalInputPin(FEHIO::P3_0)};

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
void drive_inch(drive_direction dir, float inches, float power_percent = 30.0) {
    p_controller controller(KP);


    float target_pos = inches * ENC_PER_INCH;

    left_enc.ResetCounts();
    right_enc.ResetCounts();

    PHIL_LOG("Start drive_fore_inch");

    while (left_enc.Counts() < target_pos) {
        // Find difference in encoder distance and update powers to correct it.
        // TODO: Implement motion profile?
        float power_difference = controller.update(right_enc.Counts(), left_enc.Counts());

        left_motor.SetPercent(power_percent * dir);
        right_motor.SetPercent(-(power_percent + power_difference) * dir);
    }

    PHIL_LOG("End drive_fore_inch");

    left_motor.Stop();
    right_motor.Stop();
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

void turn_degrees(turn_direction turn_dir, float degrees, float power_percent = 40.0) {
    float radians = degrees * PI / 180.0;
    float circumference = radians * (TRACK_WIDTH / 2);
    float target_pos = circumference * ENC_PER_INCH;

    p_controller controller(KP);

    left_enc.ResetCounts();
    right_enc.ResetCounts();

    PHIL_LOG("Start turn_degrees");
    
    while (left_enc.Counts() < target_pos) {
        float power_difference = controller.update(right_enc.Counts(), left_enc.Counts());

        left_motor.SetPercent(-power_percent * turn_dir);
        right_motor.SetPercent(-(power_percent + power_difference) * turn_dir);
    }

    PHIL_LOG("End turn_degrees");

    left_motor.Stop();
    right_motor.Stop();
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
    p_controller controller(KP);

    const float TARGET_PERCENT = 30.;

    float target_pos = inches * ENC_PER_INCH;

    left_enc.ResetCounts();
    right_enc.ResetCounts();

    PHIL_LOG("Start drive_until_black");

    while (!detected_black() && left_enc.Counts() < target_pos) {
        float power_difference = controller.update(right_enc.Counts(), left_enc.Counts());

        left_motor.SetPercent(TARGET_PERCENT);
        right_motor.SetPercent(-(TARGET_PERCENT + power_difference));
    }

    PHIL_LOG("End drive_until_black");

    left_motor.Stop();
    right_motor.Stop();
}

// drive until both bump switches are pressed
void drive_until_bump(float inches_cutoff) {
    p_controller controller(KP);
    const float TARGET_PERCENT = 30.;

    PHIL_LOG("Start drive_until_bump");

    // While less than tick threshold and the front bumpers aren't pressed
    while ((left_enc.Counts() + right_enc.Counts()) / 2 < inches_cutoff && (bump_switches[BS_FRONT_RIGHT].Value() || bump_switches[BS_FRONT_LEFT].Value())) {
        float power_difference = controller.update(right_enc.Counts(), left_enc.Counts());

        if (bump_switches[BS_FRONT_LEFT].Value()){
            left_motor.SetPercent(TARGET_PERCENT);
        } else {
            left_motor.Stop();
        }
        
        if (bump_switches[BS_FRONT_RIGHT].Value()){
            right_motor.SetPercent(-(TARGET_PERCENT + power_difference));
        } else {
            right_motor.Stop();
        }
    }

    PHIL_LOG("End drive_until_bump");

    left_motor.Stop();
    right_motor.Stop();
}

void log_cds_cell() {
    while (true) {
        LCD.Clear();
        LCD.Write("Voltage: ");
        LCD.WriteLine(cds_cell.Value());
        Sleep(.1);
    }
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

    drive_inch(DD_FORE, 12.5);
    turn_degrees(TD_LEFT, 60.0);

    // Turn to face juke box
    drive_until_black(8.0);
    turn_degrees(TD_LEFT, 92.0);
    drive_inch(DD_FORE, 2.5);
    
    wait_until_touch();
    float voltage = cds_cell.Value();
    if (voltage > RED_CUTOFF) {
        // Press BLUE Button
        turn_degrees(TD_LEFT, 7.0);
        drive_until_black(2.0);
        turn_degrees(TD_RIGHT, 7.0);
        drive_inch(DD_FORE, 3.0);
        drive_inch(DD_BACK, 3.0);
        // Turn back
        turn_degrees(TD_RIGHT, 30.0);
        drive_inch(DD_BACK, 12.0);
        turn_degrees(TD_LEFT, 30.0);
    } else {
        // Press RED Button
        turn_degrees(TD_RIGHT, 7.0);
        drive_until_black(2.0);
        turn_degrees(TD_LEFT, 7.0);
        drive_inch(DD_FORE, 3.0);
        drive_inch(DD_BACK, 3.0);
        // Turn back
        turn_degrees(TD_RIGHT, 45.0);
        drive_inch(DD_BACK, 15.0);
        turn_degrees(TD_LEFT, 45.0);
    }
        // Drive up and back down the ramp
        drive_inch(DD_BACK, 40.0, 70.0);
        drive_inch(DD_FORE, 40.0, 70.0);
	return 0;
}
