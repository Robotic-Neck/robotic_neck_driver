// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project

#include "MPU6050.h"
#include "Wire.h"

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
float roll, pitch;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY;
float AccErrorX = -0.006211436819285154, AccErrorY = -0.027438277378678322; //calculated values
//float AccErrorX, AccErrorY; //calculated values
float GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

#define LED_PIN 13
bool blinkState = false;

void setup() {
    
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(57600);
    accelgyro.initialize();

    // verify connection
    //Serial.println("Testing device connections...");
    //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // use the code below to change accel/gyro offset values
    /*
    accelgyro.setXAccelOffset(220);
    accelgyro.setYAccelOffset(76);
    accelgyro.setZAccelOffset(-85);
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */

    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);

    // Call this function if you need to get the IMU error values for your module
    calculate_IMU_error();
    delay(20);
}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);
    if (ax != 0 || az != 0) { // for zero error
      accAngleX = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180/ PI) - AccErrorX;
    }
    if (ay !=0 || az != 0) {
      accAngleY = atan(-1 * ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180/ PI) - AccErrorY;
    }

    // === Read gyroscope data === //
    previousTime = currentTime;        // Previous time is stored before the actual time read
    currentTime = millis();            // Current time actual time read
    elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
    
    gyroAngleX = gx/131.0 - GyroErrorX; // GyroErrorX ~(-0.56)
    gyroAngleY = gy/131.0 - GyroErrorY; // GyroErrorY ~(2)
    
    roll = 0.96 * (roll + gyroAngleX * elapsedTime) + 0.04 * accAngleX;
    pitch = 0.96 * (pitch + gyroAngleY * elapsedTime) + 0.04 * accAngleY;

    // Print the values on the serial monitor
    Serial.print(roll); Serial.print("/"); Serial.println(pitch);

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    delay(100);
}

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  /*
  while (c < 200) {
    accelgyro.getAcceleration(&ax, &ay, &az);
    
    // Sum all readings
    AccErrorX = AccErrorX + atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180/ PI);
    AccErrorY = AccErrorY + atan(-1 * ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180/ PI);
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  */

  // Read gyro values 200 times
  while (c < 200) {
    accelgyro.getRotation(&gx, &gy, &gz);

    // Sum all readings
    GyroErrorX = GyroErrorX + (gx / 131.0);
    GyroErrorY = GyroErrorY + (gy / 131.0);
    GyroErrorZ = GyroErrorZ + (gz / 131.0);
    c++;
  }

  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  
  // Print the error values on the Serial Monitor
  /*
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
  */
}