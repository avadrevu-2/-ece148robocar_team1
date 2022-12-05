#include "LSM6DS3.h"
#include "Wire.h"

#include "MadgwickAHRS.h"

LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

// setup filter and variables
Madgwick filter;
float accelX = 0; float accelY = 0; float accelZ = 0;
float gyroX = 0; float gyroY = 0; float gyroZ = 0;
int sampleTime = 0;
double roll = 0.00, pitch = 0.00, heading = 0.00; 

void setup() {
  Serial.begin(9600);
  while (!Serial);
  delay(500);
  status_t imu_status = myIMU.begin();
  if (imu_status != 0) {
    Serial.println("Device error");
    Serial.println(imu_status);
  } 
  else {
    Serial.println("Device OK!");
  }

  myIMU.settings.accelBandWidth = 200;
  myIMU.settings.accelSampleRate = 52;
  myIMU.settings.gyroSampleRate = 52;
  myIMU.settings.gyroBandWidth = 200;
  filter.begin(52);
  delay(500);
}
 
void loop() {
  // sampleSensors polls the sensor at the set refresh rate
  // and updates the accel and gryo global variables
  if (sampleSensors()) {
    // update filter
    filter.updateIMU(gyroX, gyroY, gyroZ, accelX, accelY, accelZ);

    // get roll pitch and heading
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();

    // add data to message and send
    String message = String(sampleTime) + ",";
    message += String(accelX, 15) + "," + String(accelY, 15) + "," + String(accelZ, 15) + ",";
    message += String(gyroX, 15) + "," + String(gyroY, 15) + "," + String(gyroZ, 15);
    message += "," + String(roll) + "," + String(pitch) + "," + String(heading);
    Serial.println(message);
  }
}
