#include "drivers.h"
#include "nav.h" 

// ---------------------------------------------------------
// OBJECT INSTANTIATIONS
// ---------------------------------------------------------
DualMAX14870MotorShield motors;
Pixy2 pixy;
SharpIR sharpL(SharpIR::GP2Y0A41SK0F, A2);  
SharpIR sharpR(SharpIR::GP2Y0A41SK0F, A3);

uint16_t BNO055_SAMPLERATE_DELAY_MS = 0;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// ---------------------------------------------------------
// PIN DEFINITIONS & GLOBALS
// ---------------------------------------------------------
const int encoderA_L = 2;
const int encoderB_L = 3;
const int encoderA_R = 19;
const int encoderB_R = 18;

volatile long count_R = 0;
volatile long count_L = 0;

const float RADIUS = 69.5 / 2.0 / 10.0; 
const float TRACK_WIDTH = 12.6;         
const float PI_VAL = 3.1415;
const float COUNTS_PER_WHEEL_REV = 1633.0; 

float robot_turn_circumference = PI_VAL * TRACK_WIDTH;
float wheel_circumference = 2.0 * PI_VAL * RADIUS;

const float S = 20.0;     
const float C = 2.0;      
const int BASE_SPEED = 90;
const int TURN_SPEED = 80;

const float MIN_TURN_CLEARANCE = 0;   
const float WALL_THRESHOLD = 12.0;    
const float WALL_DIST_TOO_BIG = 30.0; 

float D_L = 0.0;
float D_R = 0.0;
float D = 0.0;
float prev_D = 0.0;
float prev_D_L = 0.0;
float prev_D_R = 0.0;
float D_history[3] = {0.0, 0.0, 0.0};

unsigned long lastStuckCheck = 0;
int stuckCounter = 0;

// Local IMU tracking variables
float pos_x = 0, pos_y = 0, pos_z = 0;
float vel_x = 0, vel_y = 0, vel_z = 0;
unsigned long lastTimeIMU = 0;

// ---------------------------------------------------------
// SENSOR & HARDWARE FUNCTIONS
// ---------------------------------------------------------

void setupIMU() {
  Serial.println("Orientation Sensor Test\n");
  if (!bno.begin()) {
    Serial.print("no BNO055 detected ...");
    while (1);
  }
  delay(1000);
}

void updateIMU() {
  sensors_event_t orientationData;
  sensors_event_t linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTimeIMU) / 1000000.0;
  lastTimeIMU = currentTime;

  float roll   = orientationData.orientation.heading;
  float yaw  = orientationData.orientation.roll;
  float pitch = orientationData.orientation.pitch;

  float accel_x = linearAccelData.acceleration.x;
  float accel_y = linearAccelData.acceleration.y;
  float accel_z = linearAccelData.acceleration.z;
  
  pos_x += (vel_x * dt) + (0.5 * accel_x * dt * dt);
  pos_y += (vel_y * dt) + (0.5 * accel_y * dt * dt);
  pos_z += (vel_z * dt) + (0.5 * accel_z * dt * dt);

  vel_x += accel_x * dt;
  vel_y += accel_y * dt;
  vel_z += accel_z * dt;

  // Pass calculated hardware states back to the global navigation struct
  updatePosition(roll, pitch, yaw, accel_x, accel_y, accel_z);
}

void updateSensors() {
  D_L = sharpL.getDistance();
  D_R = sharpR.getDistance();

  pinMode(PING_PIN, OUTPUT);
  digitalWrite(PING_PIN, LOW); 
  delayMicroseconds(2);
  digitalWrite(PING_PIN, HIGH); 
  delayMicroseconds(5);
  digitalWrite(PING_PIN, LOW);  
  pinMode(PING_PIN, INPUT);
  long dur = pulseIn(PING_PIN, HIGH, 25000);
  float new_D = (dur == 0) ? 999 : dur / 29.0 / 2.0;

  D_history[2] = D_history[1];
  D_history[1] = D_history[0];
  D_history[0] = new_D;

  float a = D_history[0], b = D_history[1], c = D_history[2];
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  D = b; 

  if (millis() - lastStuckCheck > 500) {
    float d_change = abs(D - prev_D);
    float l_change = abs(D_L - prev_D_L);
    float r_change = abs(D_R - prev_D_R);

    if (pos.currentState != SEARCHING && pos.currentState != TURN_TAG && pos.currentState != REVERSING) {
      if (d_change < 3.0 && l_change < 3.0 && r_change < 3.0) {
        stuckCounter++;
      } else {
        stuckCounter = 0; 
      }

      if (stuckCounter >= 2) {
        pos.currentState = REVERSING;
        stuckCounter = 0;
      }
    }

    prev_D = D; prev_D_L = D_L; prev_D_R = D_R;
    lastStuckCheck = millis();
  }
}

// ---------------------------------------------------------
// MOVEMENT FUNCTIONS
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

void turn_deg(float deg) {
  long start_L, start_R;
  noInterrupts();
  start_L = count_L;
  start_R = count_R;
  interrupts();

  float wheel_distance_to_travel = (abs(deg) / 360.0) * robot_turn_circumference;
  long target_counts = (wheel_distance_to_travel / wheel_circumference) * COUNTS_PER_WHEEL_REV;

  if (deg > 0) {
    motors.setM1Speed(TURN_SPEED);   
    motors.setM2Speed(-TURN_SPEED);  
  } else {
    motors.setM1Speed(-TURN_SPEED);  
    motors.setM2Speed(TURN_SPEED);   
  }

  while (true) {
    long current_L, current_R;
    noInterrupts();
    current_L = count_L;
    current_R = count_R;
    interrupts();

    long diff_L = abs(current_L - start_L);
    long diff_R = abs(current_R - start_R);

    if (diff_L >= target_counts || diff_R >= target_counts){
      break;
    }
  }
  motors.setSpeeds(0, 0);
}

void drive_dist(float dist_cm) {
  long start_L, start_R;
  noInterrupts();
  start_L = count_L;
  start_R = count_R;
  interrupts();

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

    if (diff_L >= target_counts || diff_R >= target_counts){
      break;
    }
  }
  motors.setSpeeds(0, 0);
}