#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <DualMAX14870MotorShield.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
DualMAX14870MotorShield motors;

// PID gains
float Kp = 5.0;
float Ki = 0.0;
float Kd = 0.0;

// Desired angle
float setpoint = 0.0;

// PID variables
float error = 0.0;
float prevError = 0.0;
float integral = 0.0;
float derivative = 0.0;
float u = 0.0;

unsigned long prevTime = 0;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Orientation Sensor PID Test");

  if (!bno.begin()) {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
  motors.init();
  prevTime = millis();
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop(void) {

  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  float roll  = orientationData.orientation.heading;
  float yaw   = orientationData.orientation.roll;
  float pitch = orientationData.orientation.pitch;

  // Choose one axis only
  float measuredAngle = yaw;

  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  if (dt <= 0) dt = 0.001;

  error = setpoint - measuredAngle;
  integral += error * dt;
  derivative = (error - prevError) / dt;

  u = Kp * error + Ki * integral + Kd * derivative;
  prevError = error;

  int motorCmd = constrain((int)u, -100, 100);

  motors.setM1Speed(motorCmd);
  motors.setM2Speed(-motorCmd);

  Serial.print("Measured Angle: ");
  Serial.print(measuredAngle);
  Serial.print(" | Error: ");
  Serial.print(error);
  Serial.print(" | u: ");
  Serial.print(u);
  Serial.print(" | MotorCmd: ");
  Serial.println(motorCmd);

  delay(20);
}