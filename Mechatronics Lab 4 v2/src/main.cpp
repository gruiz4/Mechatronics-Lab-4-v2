#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

uint16_t BNO055_SAMPLERATE_DELAY_MS = 00;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10); 

  Serial.println("Orientation Sensor Test\n");

  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
}
void loop(void)
{
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

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

  delay(BNO055_SAMPLERATE_DELAY_MS);
  
}