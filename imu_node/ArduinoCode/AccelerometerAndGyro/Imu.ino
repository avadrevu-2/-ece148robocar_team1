void readIMU() {
    // Accel
    accelX = myIMU.readFloatAccelX();
    accelY = myIMU.readFloatAccelY();
    accelZ = myIMU.readFloatAccelZ();
 
    // Gyroscope
    gyroX = myIMU.readFloatGyroX();
    gyroY = myIMU.readFloatGyroY();
    gyroZ = myIMU.readFloatGyroZ();
}