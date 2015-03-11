#include <PID_v1.h>


/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include <Servo.h>

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

int leftPin = 9;
int rightPin = 6;
int frontPin = 5;
int rearPin = 3;

int counter= 0;

Servo left;
Servo right;
Servo front;
Servo rear;

int baseSpeed = 60;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

double SetpointX = 0, InputLeft=0,InputRight=0, OutputLeft,OutputRight;
double SetpointY = 0, InputRear=0 , InputFront=0, OutputRear, OutputFront;

PID LeftPID(&InputLeft, &OutputLeft, &SetpointX,0.5,0.3,0.2, DIRECT);
PID RightPID(&InputRight, &OutputRight, &SetpointX,0.5,0.3,0.2, DIRECT);
PID FrontPID(&InputFront, &OutputFront, &SetpointY,0.5,0.3,0.2, DIRECT);
PID RearPID(&InputRear, &OutputRear, &SetpointY,0.5,0.3,0.2, DIRECT);

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine

void setup() {
  
  left.attach(leftPin);
  right.attach(rightPin);
  front.attach(frontPin);
  rear.attach(rearPin);
  
  calibrateMotors();

  Serial.begin(115200);
  Wire.begin();
  
  LeftPID.SetMode(AUTOMATIC);
  RightPID.SetMode(AUTOMATIC);
  FrontPID.SetMode(AUTOMATIC);
  RearPID.SetMode(AUTOMATIC);
  
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
    

  
  }

  delay(100); // Wait for sensor to stabilize
  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}

void loop() {
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
                          
  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
#if 0 // Set to 1 to activate
  //Serial.print(accX); Serial.print("\t");
  //Serial.print(accY); Serial.print("\t");
  //Serial.print(accZ); Serial.print("\t");

  //Serial.print(gyroX); Serial.print("\t");
  //Serial.print(gyroY); Serial.print("\t");
  //Serial.print(gyroZ); Serial.print("\t");

  //Serial.print("\t");
#endif

//  Serial.print(roll); Serial.print("\t");
//  Serial.print(gyroXangle); Serial.print("\t");
//  Serial.print(compAngleX); Serial.print("\t");
//  Serial.print(kalAngleX); Serial.print("\t");
//  Serial.print("\t");
//
//  
//
//  Serial.print(pitch); Serial.print("\t");
//  Serial.print(gyroYangle); Serial.print("\t");
//  Serial.print(compAngleY); Serial.print("\t");
//  Serial.print(kalAngleY); Serial.print("\t");
//  
  InputLeft = kalAngleX;
  InputRight = -kalAngleX;
  
  InputRear = kalAngleY;
  InputFront = -kalAngleY;
  
  LeftPID.Compute();
  RightPID.Compute();
  
  RearPID.Compute();
  FrontPID.Compute();

  //left.write(baseSpeed + OutputLeft);
  //right.write(baseSpeed + OutputRight);
  front.write(baseSpeed + OutputFront);
  rear.write(baseSpeed + OutputRear);
  
  Serial.print(baseSpeed + OutputFront); Serial.print(",");Serial.println(baseSpeed + OutputRear);
//  Serial.print(kalAngleY); Serial.print("\t");
 
  if(counter++ > 3000){
    
  left.write(0);
  right.write(0);
  front.write(0);
  rear.write(0);
  
    while(true){
      delay(10000);
    }
   
    counter = 0;
  }
  

#if 0 // Set to 1 to print the temperature
  //Serial.print("\t");

  double temperature = (double)tempRaw / 340.0 + 36.53;
  //Serial.print(temperature); Serial.print("\t");
#endif

  //Serial.print("\r\n");
  delay(2);
}

void calibrateMotors(){
  left.write(170);
  right.write(170);
  front.write(170);
  rear.write(170);
  
  counter = 0;
  delay(10000);
  
  left.write(20);
  right.write(20);
  front.write(20);
  rear.write(20);
  
  delay(5000);
  
  left.write(0);
  right.write(0);
  front.write(0);
  rear.write(0);
  
}
