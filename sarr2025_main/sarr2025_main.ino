// import servo library for PWM control
#include <Servo.h>

// debug flags
#define MODE_DEBUG 0
#define DRIVE_DEBUG 0
#define PHOTO_DEBUG 1

// constants - receiver channels
const int CH_1_PIN = 12;
const int CH_2_PIN = 11;
const int CH_3_PIN = 10;
const int CH_4_PIN = 9;
const int CH_5_PIN = 8;
const int CH_6_PIN = 7;

// constants - motor controller channels
const int L_SERVO_PIN = 1;
const int R_SERVO_PIN = 2;

// constants - motor inversions
const bool L_MOTOR_INVERTED = true; 
const bool R_MOTOR_INVERTED = false;

// constants - motor PWM bounds
const int MOTOR_PWM_CENTER = 1500;
const int MOTOR_PWM_AMP = 500;

// constants - RC Controller in bounds
const int RC_IN_CENTER = 1500;
const int RC_IN_AMP = 500;

// constant - joystick deadzone (stick drift)
const double RC_DEADZONE_PCT = 0.06;

// constants - peripheral pins
const int L_PHOTO_PIN = A1;    // Left Photoresistor
const int R_PHOTO_PIN = A0;    // Right Photoresistor
const int M_SHARP_PIN = A2;  // Sharp Sensor
const int L_SHARP_PIN = A3;  // Sharp Sensor
const int R_SHARP_PIN = A4;  // Sharp Sensor
const int LED = 13;       // Onboard LED location

// constants - CdS Photosensor Tuning
const int L_PHOTO_DEFAULT = 260;
const int R_PHOTO_DEFAULT = 600;
const double L_TO_R_SCALE = 67/24; // empirical - tune out differences in voltage dividers

// auton parameter constants
const int SHARP_VAL_MAX = 600; // max distance sensor value before stopping
const int PHOTO_VAL_DIFF_TARGET = -50; // target sensor delta
const int PHOTO_VAL_TARGET_THRESH = 50; // min difference to start driving forward 
const int PHOTO_VAL_START_THRESH = -300; // intensity before starting homing
const double PHOTO_STEER_kP = 0.4/150; // kP for steering controller

// **************************************************************************
// HELPER FUNCTIONS
// **************************************************************************

// simple clamping function
double clamp(double in, double min, double max) {
    if (in < min) {
        return min;
    }
    if (in > max) {
        return max;
    }
    return in;
}

// rolling avg sharp sensor readout - from Brian
int getSharpVal(int sharp_pin) {
    int total = analogRead(sharp_pin);
    for (int i = 0; i <= 3; i++) {
        total = total + analogRead(sharp_pin);
    }
    
    // return 5-frame rolling average
    return total / 5;
}

int getRPhotoVal() {
    int rPhotoVal = analogRead(R_PHOTO_PIN) - R_PHOTO_DEFAULT;

    #if (PHOTO_DEBUG == 1)
    Serial.print("R Photo Val: ");
    Serial.println(rPhotoVal);
    #endif

    return rPhotoVal;
}

int getLPhotoVal() {
    int lPhotoVal = int(L_TO_R_SCALE*(analogRead(L_PHOTO_PIN) - L_PHOTO_DEFAULT));

    #if (PHOTO_DEBUG == 1)
    Serial.print("L Photo Val: ");
    Serial.println(lPhotoVal);
    #endif

    return lPhotoVal;
}

int getPhotoValDifference() {
    return (getRPhotoVal() - getLPhotoVal());
}

// wrappers for RC Controller input - including a deadzone
double getRStickXPct() {
    double rawValue = double(int(pulseIn(CH_1_PIN, HIGH, 21000)) - RC_IN_CENTER) / RC_IN_AMP;

    if (abs(rawValue) < RC_DEADZONE_PCT) {
        return 0.0;
    }

    return rawValue;
}

double getRStickYPct() {
    double rawValue = double(int(pulseIn(CH_2_PIN, HIGH, 21000)) - RC_IN_CENTER) / RC_IN_AMP;

    if (abs(rawValue) < RC_DEADZONE_PCT) {
        return 0.0;
    }

    return rawValue;
}

// return whether or not the shoulder switches are pushed out
bool getLShoulderSwitchIn() {
    return (pulseIn(CH_6_PIN, HIGH, 21000) > 1600);
}

bool getRShoulderSwitchIn() {
    return (pulseIn(CH_5_PIN, HIGH, 21000) > 1600);
}

// controller debug helper
void printRC()
{
    Serial.print("Value Ch1 = ");
    Serial.println(pulseIn(CH_1_PIN, HIGH, 21000));
    Serial.print("Value Ch2 = ");
    Serial.println(pulseIn(CH_2_PIN, HIGH, 21000));
    Serial.print("Value Ch3 = ");
    Serial.println(pulseIn(CH_3_PIN, HIGH, 21000));
    Serial.print("Value Ch4 = ");
    Serial.println(pulseIn(CH_4_PIN, HIGH, 21000));
    Serial.print("Value Ch5 = ");
    Serial.println(pulseIn(CH_5_PIN, HIGH, 21000));
    Serial.print("Value Ch6 = ");
    Serial.println(pulseIn(CH_6_PIN, HIGH, 21000));
    delay(400);
}

// **************************************************************************
// CLASS DEFINITIONS
// **************************************************************************

// enum governing main robot behavior
enum RobotMode {
    DISABLED = 0,
    TELEOP = 1,
    NAV_BRIDGE_LIGHT = 2
};

enum RobotMode g_robotMode = DISABLED;

// helper to keep robotMode up to date
void updateRobotMode() {
    // use paddles to check robot state
    if (!getLShoulderSwitchIn()) {
        g_robotMode = DISABLED;
        digitalWrite(LED, LOW);
    } else if (getRShoulderSwitchIn()) {
        // if robot state is not an auto state, go to first auto state
        if ((g_robotMode == DISABLED) || (g_robotMode == TELEOP)) {
            g_robotMode = NAV_BRIDGE_LIGHT;
        }
        digitalWrite(LED, HIGH);
    } else {
        g_robotMode = TELEOP;
        digitalWrite(LED, HIGH);
    }
}

// class owning the two drivebase motor controllers
// handles math to convert from controller input -> motor output
class DifferentialDrive {
    private:
        Servo _l_servo, _r_servo;
        int _l_servo_pin, _r_servo_pin;
        int _kL_inv;
        int _kR_inv;
        
    public:
        DifferentialDrive(int l_servo_pin, int r_servo_pin);
        void init();
        void driveTank(double l_pct, double r_pct);
        void driveArcade(double fwd_pct, double rot_pct);
};

// on initialize: init PWM pins and determine which motors are inverted
DifferentialDrive::DifferentialDrive(int l_servo_pin, int r_servo_pin) {
    _l_servo_pin = l_servo_pin;
    _r_servo_pin = r_servo_pin;

    _kL_inv = 1 - 2 * L_MOTOR_INVERTED;
    _kR_inv = 1 - 2 * R_MOTOR_INVERTED;

}

// attach pins to servos
void DifferentialDrive::init() {
    _l_servo.attach(_l_servo_pin);
    _r_servo.attach(_r_servo_pin);
}

// pass L, R setpoints as percentage of max
void DifferentialDrive::driveTank(double l_pct, double r_pct) {
    l_pct = clamp(l_pct, -1, 1);
    r_pct = clamp(r_pct, -1, 1);

    _l_servo.writeMicroseconds(MOTOR_PWM_CENTER + int(MOTOR_PWM_AMP * l_pct * _kL_inv));
    _r_servo.writeMicroseconds(MOTOR_PWM_CENTER + int(MOTOR_PWM_AMP * r_pct * _kR_inv));

    #if (DRIVE_DEBUG == 1)
    Serial.print("L Motor PWM Request: ");
    Serial.println(MOTOR_PWM_CENTER + int(MOTOR_PWM_AMP * l_pct * _kL_inv));

    Serial.print("R Motor PWM Request: ");
    Serial.println(MOTOR_PWM_CENTER + int(MOTOR_PWM_AMP * r_pct * _kR_inv));
    #endif

}

// pass FWD, ROT setpoints as a percentage. ROT uses CCW positive.
void DifferentialDrive::driveArcade(double fwd_pct, double rot_pct) {
    fwd_pct = clamp(fwd_pct, -1 , 1);
    rot_pct = clamp(rot_pct, -1 , 1);

    // CCW positive standard
    double l_pct = fwd_pct - rot_pct;
    double r_pct = fwd_pct + rot_pct;

    // scale to PWM maxes
    double scale = 1/max(abs(l_pct), max(abs(r_pct), 1.0));

    _l_servo.writeMicroseconds(MOTOR_PWM_CENTER + int(MOTOR_PWM_AMP * l_pct * _kL_inv * scale));
    _r_servo.writeMicroseconds(MOTOR_PWM_CENTER + int(MOTOR_PWM_AMP * r_pct * _kR_inv * scale));

    #if (DRIVE_DEBUG == 1)
    Serial.print("L Motor PWM Request: ");
    Serial.println(MOTOR_PWM_CENTER + int(MOTOR_PWM_AMP * l_pct * _kL_inv * scale));

    Serial.print("R Motor PWM Request: ");
    Serial.println(MOTOR_PWM_CENTER + int(MOTOR_PWM_AMP * r_pct * _kR_inv * scale));
    #endif
}

// DifferentialDrive subsystem
DifferentialDrive drive (L_SERVO_PIN, R_SERVO_PIN);

// **************************************************************************
// ROBOT CODE
// **************************************************************************
void setup() {
    // Set the pins that the transmitter will be connected to all to input
    pinMode(CH_1_PIN, INPUT);
    pinMode(CH_2_PIN, INPUT);
    pinMode(CH_3_PIN, INPUT);
    pinMode(CH_4_PIN, INPUT);
    pinMode(CH_5_PIN, INPUT);
    pinMode(CH_6_PIN, INPUT);
    // diagnostic LED
    pinMode(LED, OUTPUT);

    // init drive subsystem
    drive.init();

    //Flash the LED on and Off 10x before entering main loop
    for (int i = 0; i < 10; i++) {
        digitalWrite(13, HIGH);
        delay(200);
        digitalWrite(13, LOW);
        delay(200);
    }
    //Flash the LED on and Off 10x End
    Serial.begin(9600);
}

void loop() {
    updateRobotMode();

    Serial.print("Photo net sensor read: ");
    Serial.println(getPhotoValDifference());

    switch (g_robotMode) {
        case 0: // DISABLED
            #if (MODE_DEBUG == 1)
            Serial.println("DISABLED");
            #endif

            // turn off chassis
            drive.driveTank(0, 0);
            break;
        case 1: // TELEOP
            #if (MODE_DEBUG == 1)
            Serial.println("TELEOP");
            #endif

            // teleop main loop
            teleop();
            break;
        case 2: // NAV_BRIDGE_LIGHT
            #if (MODE_DEBUG == 1)
            Serial.println("NAV_BRIDGE_LIGHT");
            #endif

            navBridgeLight();
            break;
    }

}

void teleop() {
    drive.driveArcade(getRStickYPct(), -getRStickXPct());
}

// helper function to handle navigate to light tasks
void goToLight() {
    if (getRPhotoVal() > PHOTO_VAL_START_THRESH) {
        // slow turn to find target
        drive.driveArcade(0, 0.2);
    } else {
        // drive towards target, proportional turn
        int error = getPhotoValDifference() - PHOTO_VAL_DIFF_TARGET;
        if (abs(error) > PHOTO_VAL_TARGET_THRESH) {
            drive.driveArcade(0, PHOTO_STEER_kP * error);
        } else {
            drive.driveArcade(0.225, PHOTO_STEER_kP * error);
        }
    }
}

void navBridgeLight() {
    // main command: go to bridge light
    goToLight();

    // stop when near walls and go to next state
    if (getSharpVal(M_SHARP_PIN) > SHARP_VAL_MAX) {
        Serial.println("Autonomous completed!");
        drive.driveTank(0, 0);
        g_robotMode = DISABLED;
    }
}