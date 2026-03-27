#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "nav.h"
#include "drivers.h"
#include "xbee.h"

bool XBEE_valid = false;
long last_XBEE_game_signal = 0;
const int game_signal_hyst = 100;


void setup(void) {
  Serial.begin(115200);
  Serial2.begin(115200);

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
  updateIMU(); 
  

  targetHeading = pos.yaw;
  delay(1000);
}

void loop(void) {

  updateIMU(); 
  fetchXBeePosition(pos.xbeeX, pos.xbeeY, pos.gameByte);
  // updateSensors();//Should be updating pos struct
  
  // navigate(); //does the navigation things.
  

  //Debug

  // Serial.print("Roll:");
  // Serial.print(pos.roll);
  // Serial.print(" | Yaw:");
  // Serial.print(pos.yaw);
  // Serial.print(" | Pitch:");
  // Serial.println(pos.pitch);
  // updateSensors();
  // smartDelay(500);
  

  Serial.println(pos.currentState);

  if (pos.gameByte){
    XBEE_valid = true;
    last_XBEE_game_signal = millis();
  }
  else{
    if (last_XBEE_game_signal - millis() > game_signal_hyst){
    XBEE_valid = false;
    motors.setSpeeds(0,0);
    }
  }


  if (XBEE_valid){
    updateSensors();//Should be updating pos struct
  
    navigate(); //does the navigation things.
  
    delay(BNO055_SAMPLERATE_DELAY_MS);
  }

  else{
    
    if (pos.gameByte){
      XBEE_valid = true;
      Serial.println("Valid Signal");
      updateIMU();
      targetHeading = pos.yaw;

    }
    
  }



  // Serial.println(pos.currentState);

}