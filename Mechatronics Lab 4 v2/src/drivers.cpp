#include "drivers.h"
#include "nav.h" // Needed for State enum and currentState

// ---------------------------------------------------------
// OBJECT INSTANTIATIONS
// ---------------------------------------------------------
DualMAX14870MotorShield motors;
Pixy2 pixy;
SharpIR sharpL(SharpIR::GP2Y0A41SK0F, A4);  
SharpIR sharpR(SharpIR::GP2Y0A41SK0F, A3);

uint16_t BNO055_SAMPLERATE_DELAY_MS = 0;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// ---------------------------------------------------------
// PIN DEFINITIONS
// ---------------------------------------------------------
const int encoderA_L = 2;
const int encoderB_L = 3;
const int encoderA_R = 18;  
const int encoderB_R = 19;

// Xbee goes to UART2 bus (Serial 2), RX on 17, TX on 16. (RX = XBEE DOUT)
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
const int BASE_SPEED = 100;
const int TURN_SPEED = 80;


// Safety Constants
const float MIN_TURN_CLEARANCE = 0;   // Minimum cm needed on the side to safely pivot
const float WALL_THRESHOLD = 12.0;    // Distance indicating a wall is present
const float WALL_DIST_TOO_BIG = 30.0; // If the sum of the sensors reads a value larger than this, it won't center

// ---------------------------------------------------------
// SENSOR & STATE VARIABLES
// ---------------------------------------------------------

float prev_D = 0.0;
float prev_D_L = 0.0;
float prev_D_R = 0.0;
float D_history[3] = {30.0, 30.0, 30.0};

unsigned long lastStuckCheck = 0;
int stuckCounter = 0;
State currentState = SEARCHING;



// Local IMU tracking variables
float pos_x = 0, pos_y = 0, pos_z = 0;
float vel_x = 0, vel_y = 0, vel_z = 0;
unsigned long lastTimeIMU = 0;

// ---------------------------------------------------------
// SENSOR & HARDWARE FUNCTIONS
// ---------------------------------------------------------
void smartDelay(unsigned long ms) {
    unsigned long start = millis();
    while (millis() - start < ms) {
        updateIMU(); 
        // Optional: fetchXBeePosition(pos.xbeeX, pos.xbeeY); to keep XBee buffer empty
    }
}

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
  // updatePosition(roll, pitch, yaw, accel_x, accel_y, accel_z);
  pos.roll = orientationData.orientation.heading;
  pos.yaw  = orientationData.orientation.roll;
  pos.pitch = orientationData.orientation.pitch;

  pos.accel[0] = linearAccelData.acceleration.x;
  pos.accel[1] = linearAccelData.acceleration.y;
  pos.accel[2] = linearAccelData.acceleration.z;
}
void updateSensors() {
  pos.D_L = sharpL.getDistance();
  pos.D_R = sharpR.getDistance();

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
  pos.D = b; // 'b' is now the median value


  Serial.print("Front Distance:");
  Serial.print(pos.D);
  Serial.print(" | Right Distance:");
  Serial.print(pos.D_R);
  Serial.print(" | Left Distance:");
  Serial.println(pos.D_L);


  // STUCK DETECTION LOGIC: Check every 500ms
  if (millis() - lastStuckCheck > 500) {
    float d_change = abs(pos.D - prev_D);
    float l_change = abs(pos.D_L - prev_D_L);
    float r_change = abs(pos.D_R - prev_D_R);

    // If robot is in a moving state but distances barely changed (less than 3cm)
    if (pos.currentState != SEARCHING && pos.currentState != TURN_TAG && pos.currentState != REVERSING) {
      if (d_change < 3.0 && l_change < 3.0 && r_change < 3.0) {
        stuckCounter++;
      } else {
        stuckCounter = 0; // Reset if movement is detected
      }

      // If stuck for 2 consecutive checks (1 second), trigger reverse
       if (stuckCounter >= 2) {
         pos.currentState = REVERSING;
        stuckCounter = 0;
      }
    }

    prev_D = pos.D; prev_D_L = pos.D_L; prev_D_R = pos.D_R;
    lastStuckCheck = millis();
  }
}


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

