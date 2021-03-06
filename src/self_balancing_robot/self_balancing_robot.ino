#include "SparkFunLSM6DS3.h"
#include "DRV8835MotorShield.h"
#include "math.h"

#define GYRO_WEIGHT .991
#define ACCEL_WEIGHT .009
#define ZERO_ANGLE 88.886
#define PRECISION 3
#define DEBUG false
#define LEDPIN 13
#define INTEGRAL_SATURATION 400
#define S_TO_MS 1000

LSM6DS3 IMU;
DRV8835MotorShield motors;

float dt = .002; //in seconds

float gyroBiasX = 0;
float initialAx = 0;
float thetaPrev = 0;
float errorPrev = 0;
float errorSum = 0;

float kp = 33;
float ki = 495;
float kd = .569;

float radToDeg(float radian);
float getAngle();


void setup() {
  Serial.begin(9600);
  IMU.begin();
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);
  delay(100);
  if(DEBUG){
    Serial.println("BEGINNING CALIBRATION. DO NOT MOVE IMU");
  }
  float xSum_gyro = 0;
  float ySum_accel = 0; 
  float zSum_accel = 0;
  int i = 0;
  for(i; i < 400; i++){
    xSum_gyro += IMU.readFloatGyroX();
    ySum_accel += IMU.readFloatAccelY();
    zSum_accel += IMU.readFloatAccelZ();
    delay(dt*S_TO_MS);
  }
  gyroBiasX = xSum_gyro/(i + 1);
  float accelAvgY = ySum_accel/(i + 1);
  float accelAvgZ = zSum_accel/(i + 1);

  thetaPrev = atan2(accelAvgY, accelAvgZ)*(180/PI);

  if(DEBUG){
    Serial.print("GyroBiasX: ");
    Serial.println(gyroBiasX);
    Serial.print("Avg Angle X: ");
    Serial.println(thetaPrev);
    Serial.println("");
  }

  motors.flipM1(true);
  digitalWrite(LEDPIN, LOW);
}

int i = 0;
void loop() {
  float theta = getAngle();
  float error = ZERO_ANGLE - theta;
  errorSum = errorSum + error;
  if(errorSum > INTEGRAL_SATURATION){
    errorSum = INTEGRAL_SATURATION;
  }
  else if(errorSum < -1*INTEGRAL_SATURATION){
    errorSum = -1*INTEGRAL_SATURATION;
  }
  float u = kp*error + ki*errorSum*dt + kd*((error-errorPrev)/dt);
  if(u>400){
    u = 400;
  }
  else if( u<-400){
    u = -400;
  }
  
  motors.setSpeeds(u, u);

  if(i%20 == 0 && DEBUG){
    Serial.print("Theta: ");
    Serial.println(theta, PRECISION);
    Serial.print("P: ");
    Serial.print(kp*error);
    Serial.print("\tI: ");
    Serial.print(ki*errorSum*dt);
    Serial.print("\tD: ");
    Serial.print(kd*((error-errorPrev)/dt));
    Serial.print("\tu: ");
    Serial.println(u);
    Serial.println("");
  }
  i++;
  thetaPrev = theta;
  errorPrev = error;
  delay(dt*S_TO_MS);
}

float radToDeg(float radian){
  return radian*(180/PI);
}

float getAngle(){
  float y_acc = IMU.readFloatAccelY(); //returns value in G's
  float z_acc = IMU.readFloatAccelZ();
  float x_angle_deg = atan2(y_acc, z_acc)*(180/PI);
  float x_gyro = IMU.readFloatGyroX() - gyroBiasX; //returns angular velocity about x axis
  return GYRO_WEIGHT*(thetaPrev + x_gyro*dt) + ACCEL_WEIGHT*x_angle_deg;
}
