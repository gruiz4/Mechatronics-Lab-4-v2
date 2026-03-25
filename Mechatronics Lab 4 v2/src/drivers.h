#ifndef DRIVERS_H
#define DRIVERS_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <DualMAX14870MotorShield.h>
#include <Pixy2.h>
#include <SharpIR.h>
#include <Arduino.h>

// ---------------------------------------------------------
// HARDWARE DEFINITIONS
// ---------------------------------------------------------
#define PING_PIN 14 
extern const int encoderA_L;
extern const int encoderB_L;
extern const int encoderA_R;
extern const int encoderB_R;

// Encoder Counts
extern volatile long count_R;
extern volatile long count_L;

// Object Declarations
extern DualMAX14870MotorShield motors;
extern Pixy2 pixy;
extern SharpIR sharpL;
extern SharpIR sharpR;
extern Adafruit_BNO055 bno;

extern uint16_t BNO055_SAMPLERATE_DELAY_MS;

// ---------------------------------------------------------
// PHYSICAL DIMENSIONS, KINEMATICS, & TUNING
// ---------------------------------------------------------
extern const float RADIUS; 
extern const float TRACK_WIDTH;         
extern const float PI_VAL;
extern const float COUNTS_PER_WHEEL_REV; 

extern float robot_turn_circumference;
extern float wheel_circumference;

extern const float S;     
extern const float C;      
extern const int BASE_SPEED;
extern const int TURN_SPEED;

// Safety Constants
extern const float MIN_TURN_CLEARANCE; 
extern const float WALL_THRESHOLD;    
extern const float WALL_DIST_TOO_BIG; 

// ---------------------------------------------------------
// Sensor & State Tracking Variables (Used in drivers.cpp)
// ---------------------------------------------------------

extern float prev_D;
extern float prev_D_L;
extern float prev_D_R;
extern float D_history[3];

extern unsigned long lastStuckCheck;
extern int stuckCounter;




// ---------------------------------------------------------
// Function Declarations
// ---------------------------------------------------------
void interruptA_R();
void interruptB_R();
void interruptA_L();
void interruptB_L();

void turn_deg(float deg);
void drive_dist(float dist_cm);
void updateSensors();

void setupIMU();
void updateIMU();

void smartDelay(unsigned long ms);


void updateBNO055(class BNO);

#endif