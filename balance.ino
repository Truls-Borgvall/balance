/*
  File: balance.ino
  Author: Truls Borgvall
  Date: 2023-11-06
  Description: This program reads the acceleration value in the x, y and z direction. It then converts these measurements to the accelerometers inclination in the x and y direction. Finally the two servos move to counter act the inclination of the accelerometer in the x and y direction.
*/

//Includes all the libraries
#include <Wire.h>
#include <Adafruit_Sensor.h> 
#include <Adafruit_ADXL345_U.h>
#include <Servo.h>

//Creates the servo objects
Servo servoPitch;
Servo servoRoll;

//Creates the accelerometer object
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

//Initiate variables for acceleration
float ax;
float ay;
float az;

void setup() {
  servoPitch.attach(9);
  servoRoll.attach(10);
  //Set the servos in the middle
  servoPitch.write(90);
  servoRoll.write(90);
  delay(1000); //Wait for the servos to spin
  Serial.begin(9600); //ta bort

  //Check if the accelerometer is correctly connected 
  if(!accel.begin()){
    Serial.println("No valid sensor found");
    while(1);
  }
}

void loop() {
  updateAcceleration();
  updateServo();
}

/*
  Get the average value of the 10 last acceleration measurements in the x, y and z direction.
  Parameters:
  -N/A
  Returns: Void
*/
void updateAcceleration() {
  sensors_event_t event; 
  accel.getEvent(&event);
  float sumx = 0;
  float sumy = 0;
  float sumz = 0;

  for (int i = 0; i < 10; i++) {
      sumx += event.acceleration.x;
      sumy += event.acceleration.y;
      sumz += event.acceleration.z;
  }
  ax = sumx / 10;
  ay = sumy / 10;
  az = sumz / 10;
}

/*
  Calculates the pitch and roll using euler angles: Pitch = arcsin(ax/g) = arcsin(ax/sqrt(ax^2 + ay^2 + az^2)), Roll = arctan(ay/az). It then maps the pitch and roll from -90--90 to 0--180 to make it compatible with the servo motors. Finally it writes the mapped pitch to servo 1 and the mapped roll to servo 2.
  Parameters:
  -N/A
  Returns: Void
*/
void updateServo() {
  float pitch = degrees(asin(ax/(sqrt(ax*ax+ay*ay+az*az))));
  float roll = degrees(atan(ay/az));

  Serial.print("Pitch: "); Serial.print(String(pitch)); Serial.print("  "); //ta bort
  Serial.print("Roll: "); Serial.print(String(roll)); Serial.println("  "); //ta bort

  servoPitch.write(constrain((map(pitch,-90,90,0,180)), 0, 180));
  servoRoll.write(constrain((map(roll,90,-90,0,180)), 0, 180));
}