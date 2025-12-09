// import servo library for PWM control
#include <Servo.h>
#include <vector>

// debug flags
#define MODE_DEBUG 0
#define DRIVE_DEBUG 0
#define PHOTO_DEBUG 1
#define SHARP_DEBUG 0
#define RC_DEBUG 0

using std::vector;

// constants - receiver channels
const int CH_1_PIN = 7;
const int CH_2_PIN = 8;
const int CH_3_PIN = 9;
const int CH_4_PIN = 10;
const int CH_5_PIN = 11;
const int CH_6_PIN = 12;

// constants - motor controller channels
const int L_SERVO_PIN = 2;
const int R_SERVO_PIN = 1;
const int ARM_SERVO_PIN = 3;

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
const int L_PHOTO_PIN = A0;    // Left Photoresistor
const int R_PHOTO_PIN = A1;    // Right Photoresistor
const int M_SHARP_PIN = A7;  // Sharp Sensor
const int L_SHARP_PIN = A8 ;  // Sharp Sensor
const int R_SHARP_PIN = A9;  // Sharp Sensor
const int LED = 13;       // Onboard LED location

// constants - CdS Photosensor Tuning
const int L_PHOTO_DEFAULT = 295;
const int R_PHOTO_DEFAULT = 980;
const double R_SCALE = 1.0;
const double L_TO_R_SCALE = 1.0; // empirical - tune out differences in voltage dividers

// linear interp tree settings - DATA VECTORS MUST BE ASCENDING, SAME SIZE
const vector<double> L_DATA = {-135, -87, -70, -60, -50, -44, -25, 0};
const vector<double> R_DATA = {-870, -823, -756, -710, -677, -640, -460, 0};
const bool USE_INTERP = true;

// auton parameter constants
const int BRIDGE_SHARP_VAL = 240;
const int SHARP_VAL_MAX = 305; // max distance sensor value before stopping
const int PHOTO_VAL_DIFF_TARGET = 0; // target sensor delta
const int PHOTO_VAL_TARGET_THRESH = 95; // min difference to start driving forward 
const double PHOTO_STEER_kP = 0.135/150; // kP for steering controller

// bridge auto segmaent
const int BRIDGE_START_THRESH = -600; // intensity before starting homing

const int BUCKET_START_THRESH = -600; // intensity before starting homing
const int BUCKET_SHARP_MAX = 420;

// chute nav constants
const int L_SHARP_CHUTE_THRESH = 300;
const int R_SHARP_CHUTE_THRESH = 260;
const double CHUTE_kP = 0.00025;

// logic for clearing the wall
bool g_floorSeen = false;
const int M_SHARP_CLEAR_THRESH = 140;

Servo g_armServo;

// **************************************************************************
// HELPER FUNCTIONS
// ***************************************E**********************************

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

// map from one set of nonlinear data to another w. interpolating tree
// VERY IMPORTANT: ONLY WORKS IF BOTH DATASETS ARE ASCENDING AND OF EQUAL SIZE
double interpTree(double value, vector<double> from_data, vector<double> to_data) {
    // number of brackets value can fall into
    int num_brackets = from_data.size() - 1;

    double from_upper;
    double from_lower;

    double to_upper;
    double to_lower;

    // check all brackets within dataset
    for (int i = 0; i < num_brackets - 1; i++) {

        from_lower = from_data[i];
        from_upper = from_data[i+1];

        to_lower = to_data[i];
        to_upper = to_data[i+1];

        if (value < from_upper) {
            return ((value - from_lower)/(from_upper - from_lower))*(to_upper - to_lower) + to_lower;
        }
    }

    // if not returned yet value is greater than largest mapped value, so extend that bracket
    from_lower = from_data[num_brackets - 1];
    from_upper = from_data[num_brackets];

    to_lower = to_data[num_brackets - 1];
    to_upper = to_data[num_brackets];

    return ((value - from_lower)/(from_upper - from_lower))*(to_upper - to_lower) + to_lower;

}

// rolling avg sharp sensor readout - from Brian
double getSharpVal(int sharp_pin) {
    int total = 0.0;
    for (int i = 0; i < 25; i++) {
        total = total + analogRead(sharp_pin);
    }

    #if (SHARP_DEBUG == 1)
    Serial.print("Pin Val - ");
    Serial.print(sharp_pin);
    Serial.print(": ");
    Serial.println(total / 25.0);
    #endif
    
    // return 5-frame rolling average
    return total / 25.0;
}

double getRPhotoVal() {

    double rPhotoVal = 0.0;

    for (int i = 0; i < 25; i++) {
        rPhotoVal = rPhotoVal + (analogRead(R_PHOTO_PIN) - R_PHOTO_DEFAULT)*R_SCALE;
    }

    rPhotoVal = rPhotoVal/25.0;

    #if (PHOTO_DEBUG == 1)
    Serial.print("R Photo Val: ");
    Serial.println(rPhotoVal);
    #endif

    return rPhotoVal;
}

double getLPhotoVal() {
    double lPhotoVal = 0;

    for (int i = 0; i < 25; i++) {
        lPhotoVal = lPhotoVal + L_TO_R_SCALE*(analogRead(L_PHOTO_PIN) - L_PHOTO_DEFAULT);
    }

    lPhotoVal = lPhotoVal/25.0;

    if (USE_INTERP) {
        lPhotoVal = interpTree(lPhotoVal, L_DATA, R_DATA);
    }

    #if (PHOTO_DEBUG == 1)
    Serial.print("L Photo Val: ");
    Serial.println(lPhotoVal);
    #endif

    return lPhotoVal;
}

double getPhotoValDifference() {
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

double getLStickXPct() {
    double rawValue = double(int(pulseIn(CH_4_PIN, HIGH, 21000)) - RC_IN_CENTER) / RC_IN_AMP;

    if (abs(rawValue) < RC_DEADZONE_PCT) {
        return 0.0;
    }

    return rawValue;
}


double getLStickYPct() {
    double rawValue = double(int(pulseIn(CH_3_PIN, HIGH, 21000)) - RC_IN_CENTER) / RC_IN_AMP;

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

void driveArmPct(double drive_pct) {
    drive_pct = clamp(drive_pct, -1 , 1);

    g_armServo.writeMicroseconds(MOTOR_PWM_CENTER + int(MOTOR_PWM_AMP * drive_pct));

    #if (DRIVE_DEBUG == 1)
    Serial.print("ARM Motor PWM Request: ");
    Serial.println(MOTOR_PWM_CENTER + int(MOTOR_PWM_AMP * drive_pct));
    #endif
}

// **************************************************************************
// CLASS DEFINITIONS
// **************************************************************************

// enum governing main robot behavior
enum RobotMode {
    DISABLED = 0,
    TELEOP = 1,
    NAV_BRIDGE_LIGHT = 2,
    CROSS_BRIDGE = 3,
    NAV_CHUTE = 4,
    BREACH_WALL = 5,
    NAV_BUCKET = 6,
    PLACE_MEDKIT = 7,
    ALIGN_WALL = 8,
    AUTO_FINISHED = 99
};

enum RobotMode g_robotMode = DISABLED;
enum RobotMode g_lastRobotMode = DISABLED;

// helper to keep robotMode up to date
void updateRobotMode() {
    // use paddles to check robot state
    if (!getLShoulderSwitchIn()) {
        g_robotMode = DISABLED;
        digitalWrite(LED, LOW);
    } else if (getRShoulderSwitchIn()) {
        // if robot state is not an auto state, go to first auto state
        if ((g_robotMode == DISABLED) || (g_robotMode == TELEOP)) {
            // manual wall override
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

// Timer helper class for auto modes
// consolidate a bunch of global variables into one 
class Timer {
    private:
        long _startTime;
    public:
        Timer();
        void reset();
        double secondsElapsed();
        boolean hasElapsed(double seconds);
};

// set start time on initialization
Timer::Timer() {
    _startTime = millis();
}

// reset _startTime
void Timer::reset() {
    _startTime = millis();
}

// return time since _startTime
double Timer::secondsElapsed() {
    return double(millis() - _startTime)/1000.0;
}

// check if certain time has elapsed
boolean Timer::hasElapsed(double seconds) {
    return (double(millis() - _startTime)/1000.0) > seconds;
}

// global timers
Timer g_alignWallTimer;
Timer g_bridgeDriveTimer;
Timer g_breachDriveTimer;

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

    // set sharp pins to input
    pinMode(R_SHARP_PIN, INPUT);
    pinMode(L_SHARP_PIN, INPUT);
    pinMode(M_SHARP_PIN, INPUT);

    // init arm
    g_armServo.attach(ARM_SERVO_PIN);

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

    getPhotoValDifference();
    // getLPhotoVal();

    #if (RC_DEBUG == 1)
    printRC();
    #endif
    
    #if (MODE_DEBUG == 1)
    Serial.print("LAST ROBOT MODE: ");
    Serial.println(g_lastRobotMode);
    #endif

    switch (g_robotMode) {
        case 0: // DISABLED
            #if (MODE_DEBUG == 1)
            Serial.println("DISABLED");
            #endif

            // turn off chassis and arm
            drive.driveTank(0, 0);
            driveArmPct(0.0);

            // track last robot mode for logic
            g_lastRobotMode = DISABLED;
            break;
        case 1: // TELEOP
            #if (MODE_DEBUG == 1)
            Serial.println("TELEOP");
            #endif

            // teleop main loop
            teleop();

            // track last robot mode for logic
            g_lastRobotMode = TELEOP;
            break;
        case 2: // NAV_BRIDGE_LIGHT
            #if (MODE_DEBUG == 1)
            Serial.println("NAV_BRIDGE_LIGHT");
            #endif

            navBridgeLight();

            // track last robot mode for logic
            g_lastRobotMode = NAV_BRIDGE_LIGHT;
            break;
        case 3: // CROSS_BRIDGE
            #if (MODE_DEBUG == 1)
            Serial.println("CROSS_BRIDGE");
            #endif

            crossBridge();

            // track last robot mode for logic
            g_lastRobotMode = CROSS_BRIDGE;
            break;
        case 4: // NAV_CHUTE
            #if (MODE_DEBUG == 1)
            Serial.println("NAV_CHUTE");
            #endif

            navChute();

            // track last robot mode for logic
            g_lastRobotMode = NAV_CHUTE;
            break;
        case 5:
            #if (MODE_DEBUG == 1)
            Serial.println("BREACH_WALL");
            #endif

            breachWall();

            // track last robot mode for logic
            g_lastRobotMode = BREACH_WALL;
            break;
        case 6:
            #if (MODE_DEBUG == 1)
            Serial.println("NAV_BUCKET");
            #endif

            navBucket();

            // track last robot mode for logic
            g_lastRobotMode = NAV_BUCKET;
            break;
        case 7:
            #if (MODE_DEBUG == 1)
            Serial.println("PLACE_MEDKIT");
            #endif

            placeMedkit();

            // track last robot mode for logic
            g_lastRobotMode = PLACE_MEDKIT;
            break;
        case 8:
            #if (MODE_DEBUG == 1)
            Serial.println("ALIGN_WALL");
            #endif

            alignWall();

            // track last robot mode for logic
            g_lastRobotMode = ALIGN_WALL;
            break;
        case 99:
            #if (MODE_DEBUG == 1)
            Serial.println("AUTO_FINISHED");
            #endif

            // turn off chassis and arm
            drive.driveTank(0, 0);
            driveArmPct(0.0);

            // track last robot mode for logic
            g_lastRobotMode = AUTO_FINISHED;
            break;
    }

    

}

void teleop() {
    drive.driveArcade(getRStickYPct(), -getRStickXPct());
    driveArmPct(getLStickXPct());
}

// helper function to handle navigate to light tasks
void goToLight(int start_thresh) {

    double turnMax = 0.325;

    if (getRPhotoVal() > start_thresh) {
        // slow turn to find target
        drive.driveArcade(0, turnMax*1.2);
    } else {
        // drive towards target, proportional turn
        int error = getPhotoValDifference() - PHOTO_VAL_DIFF_TARGET;
        if (abs(error) > PHOTO_VAL_TARGET_THRESH) {
            drive.driveArcade(0, clamp(PHOTO_STEER_kP * error, -turnMax, turnMax));
        } else {
            drive.driveArcade(0.4, clamp(PHOTO_STEER_kP * error, -turnMax, turnMax));
        }
    }
}

void navBridgeLight() {
    // main command: go to bridge light
    goToLight(BRIDGE_START_THRESH);

    // stop when near walls and go to next state
    if (getSharpVal(M_SHARP_PIN) > BRIDGE_SHARP_VAL) {
        Serial.println("Autonomous completed!");
        g_robotMode = CROSS_BRIDGE;
    }
}

void crossBridge() {
    // reset timer on first call
    if (g_lastRobotMode != g_robotMode) {
        g_bridgeDriveTimer.reset();
    }

    if (g_bridgeDriveTimer.hasElapsed(9.5)) {
        drive.driveTank(0, 0);
        g_robotMode = NAV_CHUTE;
    } else {
        drive.driveTank(0.3, 0.3);
    }

}

void navChute() {
    double l_sharp_val = getSharpVal(L_SHARP_PIN);
    double r_sharp_val = getSharpVal(R_SHARP_PIN);

    // stop when near walls and go to next state
    if (getSharpVal(M_SHARP_PIN) > SHARP_VAL_MAX) {
        drive.driveTank(0, 0);
        g_robotMode = ALIGN_WALL;
        return;
    }

    if (l_sharp_val > L_SHARP_CHUTE_THRESH) {
        drive.driveArcade(0.125, -0.3);
        delay(400);
    } else if (r_sharp_val > R_SHARP_CHUTE_THRESH) {
        drive.driveArcade(0.125, 0.325);
        delay(400);
    } else {
        drive.driveArcade(0.3, (r_sharp_val - l_sharp_val)*CHUTE_kP);
    }


}

// line up wheel cleats with the wall before initiating climb
void alignWall() {

    double alignPct = 0.26;

    // reset timer on first call and turn off drive
    if (g_lastRobotMode != g_robotMode) {
        g_alignWallTimer.reset();
        drive.driveTank(0.0, 0.0);
    }

    // alternate between pulsing left and right wheels every 0.5 sec
    bool wheel_pick = bool(int(g_alignWallTimer.secondsElapsed()/0.5)%2);
    if (wheel_pick) {
        drive.driveTank(-alignPct/4.0, alignPct);
    } else {
        drive.driveTank(alignPct, -alignPct/4.0);
    }

    // after certain time, go to breachWall()
    if (g_alignWallTimer.hasElapsed(5.0)) {
        g_robotMode = BREACH_WALL;
    }

}

void breachWall() {


    double breachTimeS = 12;

    // reset timer on first call and fix flags
    if (g_lastRobotMode != g_robotMode) {
        Serial.println("RESET");
        g_breachDriveTimer.reset();
    }

    // if timer up go to nav_bucket
    if (g_breachDriveTimer.hasElapsed(breachTimeS)) {
        drive.driveTank(0, 0);
        g_robotMode = NAV_BUCKET;
    } else {
        drive.driveArcade(0.25, 0.0);
        driveArmPct(0.05);
    }
}

void navBucket() {
    // main command: go to bridge light
    goToLight(BUCKET_START_THRESH);

    // stop when near walls and go to next state
    if (getSharpVal(M_SHARP_PIN) > BUCKET_SHARP_MAX) {
        Serial.println("Autonomous completed!");
        drive.driveTank(0, 0);
        g_robotMode = PLACE_MEDKIT;
    }
}

// experimenting with non-global timer structure as well
void placeMedkit() {

    // force robotMode at start
    g_robotMode = PLACE_MEDKIT;

    // first, lower arm
    long init_time_ms = millis();
    drive.driveArcade(0.0, 0.0);

    while ((millis() - init_time_ms)/1000.0 < 1.5) {
        updateRobotMode();
        if (g_robotMode != PLACE_MEDKIT) {
            break;
        }

        driveArmPct(-1.0);
    }

    driveArmPct(0.0);
    init_time_ms = millis();

    // pause for 1 second
    while ((millis() - init_time_ms)/1000.0 < 1.0) {
        updateRobotMode();
        if (g_robotMode != PLACE_MEDKIT) {
            break;
        }
    }

    init_time_ms = millis();

    // back up slowly
    while ((millis() - init_time_ms)/1000.0 < 2.0) {
        updateRobotMode();
        if (g_robotMode != PLACE_MEDKIT) {
            break;
        }
        drive.driveArcade(-0.35, 0.0);
    }

    drive.driveArcade(0.0, 0.0);

    g_robotMode = AUTO_FINISHED;

}