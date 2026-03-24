/*Drivers needed to get positioning data and drive motion for robot*/

#include <DualMAX14870MotorShield.h>
#include <Pixy2.h>
#include <SharpIR.h>
#include <Arduino.h>

// ---------------------------------------------------------
// HARDWARE DEFINITIONS
// ---------------------------------------------------------
#define PING_PIN 14 
const int encoderA_L;
const int encoderB_L;
const int encoderA_R;
const int encoderB_R;

// Encoder Counts
volatile long count_R;
volatile long count_L;



DualMAX14870MotorShield motors;
Pixy2 pixy;

/* Left and right is defined looking from the back of the robot:
        (front)
        camera
        ball
        Wheel

    left      right 
    motor     motor
    (M1)      (M2)
*/
SharpIR sharpL(SharpIR::GP2Y0A41SK0F, A2);  
SharpIR sharpR(SharpIR::GP2Y0A41SK0F, A3);

// ---------------------------------------------------------
// PHYSICAL DIMENSIONS, KINEMATICS, & TUNING
// ---------------------------------------------------------
const float RADIUS; // Wheel outer radius in cm (3.475 cm)
const float TRACK_WIDTH;         // Distance between wheel centers in cm
const float PI_VAL;
const float COUNTS_PER_WHEEL_REV; // Calibrated encoder counts per revolution

float robot_turn_circumference;

float wheel_circumference;
  

const float S;     // Grid square side length (cm)
const float C;      // Off-center threshold (cm)
const int BASE_SPEED;
const int TURN_SPEED;


// Safety Constants
const float MIN_TURN_CLEARANCE; // Minimum cm needed on the side to safely pivot
const float WALL_THRESHOLD;    // Distance indicating a wall is present
const float WALL_DIST_TOO_BIG; // If the sum of the sensors reads a value larger than this, it won't center
// const float FAR_WALL_THRESHOLD;


// ---------------------------------------------------------
// Function Declarations
// ---------------------------------------------------------


void interruptA_R();
void interruptB_R();

void interruptA_L();
void interruptB_L();

void turn_deg(float deg);
void drive_dist(float dist_cm);




void getXbee();
void updateBNO055(class BNO);