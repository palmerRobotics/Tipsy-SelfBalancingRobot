#include "SparkFunLSM6DS3.h"
#include "DRV8835MotorShield.h"
#include "math.h"

#define GYRO_WEIGHT .95
#define ACCEL_WEIGHT .05
#define ZERO_ANGLE 90
#define PRECISION 3
#define DEBUG true
#define LEDPIN 13

LSM6DS3 IMU;
DRV8835MotorShield motors;

int dt = 10; //in milliseconds

float gyroBiasX = 0;
float initialAx = 0;
float thetaPrev = 0;
float errorPrev = 0;
float errorSum = 0;

float kp = 80;
float ki = 0;
float kd = 35;

float radToDeg(float radian);
float getAngle();

void setup() {
  Serial.begin(9600);
  IMU.begin();
  pinMode(LEDPIN, OUTPUT);
  delay(100);
  if(DEBUG){
    Serial.println("BEGINNING CALIBRATION. DO NOT MOVE IMU");
  }
  float xSum_gyro = 0;
  float ySum_accel = 0; 
  float zSum_accel = 0;
  int i = 0;
  digitalWrite(LEDPIN, HIGH);
  //if(i%100 == 0){
  //  digitalWrite(LEDPIN, !digitalRead(LEDPIN));
  //}
  for(i; i < 200; i++){
    xSum_gyro += IMU.readFloatGyroX();
    ySum_accel += IMU.readFloatAccelY(); //THE ZERO ANGLE ADJUSTMENT NEEDS TO COME HERE, NOT AFTER THE TAN2 CALCULATION
    zSum_accel += IMU.readFloatAccelZ();
    delay(dt);
  }
  gyroBiasX = xSum_gyro/(i + 1);
  float accelAvgY = ySum_accel/(i + 1);
  float accelAvgZ = zSum_accel/(i + 1);

  thetaPrev = radToDeg(atan2(accelAvgY, accelAvgZ));

  if(DEBUG){
    Serial.print("GyroBiasX: ");
    Serial.println(gyroBiasX);
    Serial.print("Avg Angle X: ");
    Serial.println(thetaPrev);
  }

  motors.flipM1(true);
  digitalWrite(LEDPIN, LOW);
}

int i = 0;
void loop() {
  float theta = getAngle();
  float error = ZERO_ANGLE - theta;
  errorSum = errorSum + error;
  float u = kp*error + kd*((error-errorPrev)/dt) + ki*errorSum;
  if(u>400){
    u = 400;
  }
  else if( u<-400){
    u = -400;
  }
  
  motors.setSpeeds(u, u);

  if(i%10 == 0 && DEBUG){
    Serial.print("Theta: ");
    Serial.println(theta, PRECISION);
    Serial.print("u: ");
    Serial.println(u);
  }
  i++;
  thetaPrev = theta;
  errorPrev = error;
  delay(dt);
}

float radToDeg(float radian){
  return radian*(180/PI);
}

float getAngle(){
  float y_acc = IMU.readFloatAccelY(); //returns value in G's
  float z_acc = IMU.readFloatAccelZ();

  float x_angle_deg = radToDeg(atan2(y_acc, z_acc)); //returns angle in degrees
  
  float x_gyro = IMU.readFloatGyroX() - gyroBiasX; //returns angular velocity about x axis

  return GYRO_WEIGHT*(thetaPrev + x_gyro*dt/1000) + ACCEL_WEIGHT*x_angle_deg;
}
