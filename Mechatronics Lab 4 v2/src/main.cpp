#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "nav.h"
#include "drivers.h"


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

void loop(void) {
  updateSensors();
  
  // This calculates the physics math and automatically syncs it back to the global `pos` struct
  updateIMU(); 

  navigate(); //does the navigation things.
  
  delay(BNO055_SAMPLERATE_DELAY_MS);
}