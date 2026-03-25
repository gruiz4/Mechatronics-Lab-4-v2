#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "nav.h"
#include "drivers.h"
#include "xbee.h"

bool XBEE_valid = false;


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


  setupXBee();
  delay(1000);
}

void loop(void) {

  updateIMU(); 

  if (XBEE_valid){
  updateSensors();
  
  
  
  fetchXBeePosition(pos.xbeeX, pos.xbeeY);

  navigate(); //does the navigation things.
  
  delay(BNO055_SAMPLERATE_DELAY_MS);
  }

  else{
    if (fetchXBeePosition(pos.xbeeX, pos.xbeeY)){
      XBEE_valid = true;
      updateIMU();
      targetHeading = pos.yaw;

    }
    
  }

}