#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "nav.h"
#include "drivers.h"

uint16_t BNO055_SAMPLERATE_DELAY_MS = 00;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
float pos_x = 0, pos_y = 0, pos_z = 0;
float vel_x = 0, vel_y = 0, vel_z = 0;
unsigned long lastTime = 0;



int matchByte = 0;
int gameTime = 0;
int Xcoord = 0;
int Ycoord = 0; 

void setup(void) {
  Serial.begin(115200);
  Serial1.begin(115200);

  Serial.println("Orientation Sensor Test\n");

  if (!bno.begin()) {
    Serial.print("no BNO055 detected ...");
    while (1);
  }

  pixy.init();
  pixy.setLamp(0, 0); // Turns off the white LEDs, leaves the RGB LED off
  motors.enableDrivers();
  
  pinMode(encoderA_R, INPUT); pinMode(encoderB_R, INPUT);
  pinMode(encoderA_L, INPUT); pinMode(encoderB_L, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(encoderA_R), interruptA_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB_R), interruptB_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderA_L), interruptA_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB_L), interruptB_L, CHANGE);
  delay(1000);
}


void loop(void)
{
  sensors_event_t orientationData;
  sensors_event_t linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;

  // Accessing members directly: 
  // x = Yaw (Heading), y = Roll, z = Pitch
  float roll   = orientationData.orientation.heading;
  float yaw  = orientationData.orientation.roll;
  float pitch = orientationData.orientation.pitch;

  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(" | Pitch: ");
  Serial.print(pitch);
  Serial.print(" | Yaw: ");
  Serial.println(yaw);

  float accel_x = linearAccelData.acceleration.x;
  float accel_y = linearAccelData.acceleration.y;
  float accel_z = linearAccelData.acceleration.z;
  pos_x += (vel_x * dt) + (0.5 * accel_x * dt * dt);
  pos_y += (vel_y * dt) + (0.5 * accel_y * dt * dt);
  pos_z += (vel_z * dt) + (0.5 * accel_z * dt * dt);

  // Update Velocity: v = v0 + a*dt
  vel_x += accel_x * dt;
  vel_y += accel_y * dt;
  vel_z += accel_z * dt;
  delay(BNO055_SAMPLERATE_DELAY_MS);
  
}
