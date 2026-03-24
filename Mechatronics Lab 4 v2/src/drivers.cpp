#include "drivers.h"


/* PIN DEFINITIONS*/

// Encoders (Interrupt capable pins)
const int encoderA_L = 2;
const int encoderB_L = 3;
const int encoderA_R = 19;
const int encoderB_R = 18;

// Encoder Counts
volatile long count_R = 0;
volatile long count_L = 0;


// ---------------------------------------------------------
// PHYSICAL DIMENSIONS, KINEMATICS, & TUNING
// ---------------------------------------------------------
const float RADIUS = 69.5 / 2.0 / 10.0; // Wheel outer radius in cm (3.475 cm)
const float TRACK_WIDTH = 12.6;         // Distance between wheel centers in cm
const float PI_VAL = 3.1415;
const float COUNTS_PER_WHEEL_REV = 1633.0; // Calibrated encoder counts per revolution

float robot_turn_circumference = PI_VAL * TRACK_WIDTH;

float wheel_circumference = 2.0 * PI_VAL * RADIUS;
  

const float S = 20.0;     // Grid square side length (cm)
const float C = 2.0;      // Off-center threshold (cm)
const int BASE_SPEED = 90;
const int TURN_SPEED = 80;


// Safety Constants
const float MIN_TURN_CLEARANCE = 0; // Minimum cm needed on the side to safely pivot
const float WALL_THRESHOLD = 12.0;    // Distance indicating a wall is present
const float WALL_DIST_TOO_BIG = 30.0; // If the sum of the sensors reads a value larger than this, it won't center
// const float FAR_WALL_THRESHOLD = 12.0;


// ---------------------------------------------------------
// Function Definitions
// ---------------------------------------------------------

void interruptA_R() {
  if (digitalRead(encoderA_R) == digitalRead(encoderB_R)) count_R++;
  else count_R--;
}
void interruptB_R() {
  if (digitalRead(encoderB_R) != digitalRead(encoderA_R)) count_R++;
  else count_R--;
}
void interruptA_L() {
  if (digitalRead(encoderA_L) == digitalRead(encoderB_L)) count_L++;
  else count_L--;
}
void interruptB_L() {
  if (digitalRead(encoderB_L) != digitalRead(encoderA_L)) count_L++;
  else count_L--;
}

void updateSensors() {
  D_L = sharpL.getDistance();
  D_R = sharpR.getDistance();

  // PING))) Logic
  pinMode(PING_PIN, OUTPUT);
  digitalWrite(PING_PIN, LOW); 
  delayMicroseconds(2);
  digitalWrite(PING_PIN, HIGH); 
  delayMicroseconds(5);
  digitalWrite(PING_PIN, LOW);  
  pinMode(PING_PIN, INPUT);
  long dur = pulseIn(PING_PIN, HIGH, 25000);
  float new_D = (dur == 0) ? 999 : dur / 29.0 / 2.0;

    // FIX: Handle ultrasonic blind spot when touching a wall
  // float new_D;
  // if (dur == 0) {
  //   // If timeout, but previous read was < 10cm, we are touching the wall!
  //   new_D = (D_history[0] < 10.0) ? 1.0 : 999.0; 
  // } else {
  //   new_D = dur / 29.0 / 2.0;
  // }

  // Median filter for PING sensor to remove sporadic false reads
  D_history[2] = D_history[1];
  D_history[1] = D_history[0];
  D_history[0] = new_D;

  float a = D_history[0], b = D_history[1], c = D_history[2];
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  D = b; // 'b' is now the median value

  // Serial.print("Front Distance:");
  // Serial.print(D);
  // Serial.print(" | Right Distance:");
  // Serial.print(D_R);
  // Serial.print(" | Left Distance:");
  // Serial.println(D_L);


  // STUCK DETECTION LOGIC: Check every 500ms
  if (millis() - lastStuckCheck > 500) {
    float d_change = abs(D - prev_D);
    float l_change = abs(D_L - prev_D_L);
    float r_change = abs(D_R - prev_D_R);

    // If robot is in a moving state but distances barely changed (less than 3cm)
    if (currentState != SEARCHING && currentState != TURN_TAG && currentState != REVERSING) {
      if (d_change < 3.0 && l_change < 3.0 && r_change < 3.0) {
        stuckCounter++;
      } else {
        stuckCounter = 0; // Reset if movement is detected
      }

      // If stuck for 2 consecutive checks (1 second), trigger reverse
      if (stuckCounter >= 2) {
        currentState = REVERSING;
        stuckCounter = 0;
      }
    }

    prev_D = D; prev_D_L = D_L; prev_D_R = D_R;
    lastStuckCheck = millis();
  }
}


void drive_dist(float dist_cm) {
  long start_L, start_R;
  noInterrupts();
  start_L = count_L;
  start_R = count_R;
  interrupts();

  float wheel_circumference = 2.0 * PI_VAL * RADIUS;
  long target_counts = (abs(dist_cm) / wheel_circumference) * COUNTS_PER_WHEEL_REV;

  if (dist_cm > 0) {
    motors.setM1Speed(BASE_SPEED);   
    motors.setM2Speed(BASE_SPEED);  
  } else {
    motors.setM1Speed(-BASE_SPEED);  
    motors.setM2Speed(-BASE_SPEED);   
  }

  while (true) {
    long current_L, current_R;
    
    noInterrupts();
    current_L = count_L;
    current_R = count_R;
    interrupts();

    long diff_L = abs(current_L - start_L);
    long diff_R = abs(current_R - start_R);
    // long avg_diff = (diff_L + diff_R) / 2;

    // if (avg_diff >= target_counts) {
    //   break; 
    // }
    if (diff_L >= target_counts || diff_R >= target_counts){
      break;
    }
  }
  
  motors.setSpeeds(0, 0);
}
