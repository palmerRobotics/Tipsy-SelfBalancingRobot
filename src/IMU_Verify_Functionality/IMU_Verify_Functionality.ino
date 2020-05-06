/****************************************************************
 *             ACCELEROMETER Theta CALCULATION TEST             *
 * This code will calculate the angle of the IMU from the       *
 * vertical position (positive X points down.)  This code will  *
 * verify that my hand calculations are correct.  Data will be  *
 * read from the accelerometer.  The final implementation will  *
 * require data from the gyroscope as well.  Gyroscope data     *
 * will be examined in another program.                         *
 ***************************************************************/

#include "SparkFunLSM6DS3.h"
#include "math.h"

#define S_TO_MS 1000
#define PRECISION 2 //for printing; doesn't affect actual values.  Precision of 2 or less ensures printing is nice and organized
#define WEIGHT_GYRO .94
#define WEIGHT_ACCEL .06

LSM6DS3 IMU;
double dt = .01; //in seconds

float gyroBiasX = 0;
float gyroBiasY = 0;
float gyroBiasZ = 0;
double InitialAx = 0;
double InitialAy = 0;
double InitialAz = 0;
double prevBeta = 0;
double prevTheta = 0;
double prevZeta = 0;
bool isFirstPass = true;

float toDegrees(double radian){
  float degree = radian * (180/PI);
  return degree;
}

void printXYZ(float x, float y, float z){
  Serial.print(x, PRECISION);
  Serial.print("\t");
  Serial.print(y, PRECISION);
  Serial.print("\t");
  Serial.print(z, PRECISION);
  Serial.print("\t\t");
}

void printCalculatedAccelAngles(double Beta, double Theta, double Zeta){
  Serial.print(Beta, PRECISION);
  Serial.print("\t");
  Serial.print(Theta, PRECISION);
  Serial.print("\t");
  Serial.print(Zeta, PRECISION);
  Serial.print("\t\t");
}

void printAngleCalculations(float x_acc, float y_acc, float z_acc, double Ax, double Ay, double Az){
  Serial.println();
  Serial.print("InitialAx");
  Serial.print(": tan^-1(");
  Serial.print(y_acc, PRECISION);
  Serial.print("/");
  Serial.print(z_acc, PRECISION);
  Serial.print(") = ");
  Serial.println(Ax, PRECISION);
  Serial.print("InitialAy");
  Serial.print(": tan^-1(");
  Serial.print(z_acc, PRECISION);
  Serial.print("/");
  Serial.print(x_acc, PRECISION);
  Serial.print(") = ");
  Serial.println(Ay, PRECISION);
  Serial.print("InitialAz");
  Serial.print(": tan^-1(");
  Serial.print(x_acc, PRECISION);
  Serial.print("/");
  Serial.print(y_acc, PRECISION);
  Serial.print(") = ");
  Serial.println(Az, PRECISION);
  Serial.println();
}


void setup() {
  Serial.begin(9600);
  IMU.begin();
  Serial.println("BEGINNING CALIBRATION.  DO NOT MOVE IMU");
  float xSum_gyro = 0;
  float ySum_gyro = 0;
  float zSum_gyro = 0;                                          //I need to decide: float or double??
  float xSum_acc = 0;
  float ySum_acc = 0;
  float zSum_acc = 0;
  int i = 0;
  //Serial.println("x\ty\tz\t\txSum\tySum\tzSum");
  for(i; i < 100; i++){
    xSum_gyro += IMU.readFloatGyroX(); //rotation about the x axis    //returns float
    ySum_gyro += IMU.readFloatGyroY(); //rotation about the y axis
    zSum_gyro += IMU.readFloatGyroZ(); //rotation about the z axis
    float x_acc = IMU.readFloatAccelX(); //returns float  //these three can be removed when not doing initialization printing
    float y_acc = IMU.readFloatAccelY();
    float z_acc = IMU.readFloatAccelZ();
    xSum_acc += x_acc;
    ySum_acc += y_acc;
    zSum_acc += z_acc;
    //printXYZ(x_acc, y_acc, z_acc);
    //printXYZ(xSum_acc, ySum_acc, zSum_acc);
    //Serial.println();
    delay(dt * S_TO_MS);
  }
  gyroBiasX = xSum_gyro/i;
  gyroBiasY = ySum_gyro/i;
  gyroBiasZ = zSum_gyro/i;  
  float xAvg_acc = xSum_acc/i;
  float yAvg_acc = ySum_acc/i;
  float zAvg_acc = zSum_acc/i;

  InitialAx = toDegrees(atan2(yAvg_acc, zAvg_acc)); //Beta: YZ plane, rotation about X axis
  InitialAy = toDegrees(atan2(zAvg_acc, xAvg_acc)); //Theta: XZ plane, rotation about Y axis
  InitialAz = toDegrees(atan2(xAvg_acc, yAvg_acc)); //Zeta: XY plane, rotation about Z axis

  /*Serial.println();
  Serial.println("xAvg\tyAvg\tzAvg");
  printXYZ(xAvg_acc, yAvg_acc, zAvg_acc);
  Serial.println();
  printAngleCalculations(xAvg_acc, yAvg_acc, zAvg_acc, InitialAx, InitialAy, InitialAz);*/
 
  Serial.println("GYROSCOPE BIASES CALCULATED");        //prints can be replaced by print function
  Serial.println("X BIAS\t Y BIAS\t Z BIAS");
  printXYZ(gyroBiasX, gyroBiasY, gyroBiasZ);
  Serial.println();
  Serial.println("INITIAL ANGLES CALCULATED");
  Serial.println("AX\tAY\tAZ");
  printXYZ(InitialAx, InitialAy, InitialAz);  
  Serial.println();
  
  delay(1000);
  
  Serial.println("ACCELEROMETER\t\t\tCALC'D ACCEL ANGLES\t\tGYRO ANGULAR VELOCITY\t\tCORRECTED ANGLES");
  Serial.println("X\tY\tZ\t\tBeta\tTheta\tZeta\t\tBeta/s\tTheta/s\tZeta/s\t\tBeta\tTheta\tZeta");
}

int i = 0;
void loop() {
  if(isFirstPass){
    prevBeta = InitialAx;
    prevTheta = InitialAy;
    prevZeta = InitialAz;
    isFirstPass = false;
  }
  
  float x_acc = IMU.readFloatAccelX(); //in G-force
  float y_acc = IMU.readFloatAccelY();
  float z_acc = IMU.readFloatAccelZ();

  float x_gyro = IMU.readFloatGyroX() - gyroBiasX; //angular velocity about x axis //in deg/sec
  float y_gyro = IMU.readFloatGyroY() - gyroBiasY; //angular velocity about y axis
  float z_gyro = IMU.readFloatGyroZ() - gyroBiasZ; //angular velocity about z axis

  double Ax_acc = toDegrees(atan2(y_acc, z_acc)); //Beta: YZ plane, rotation about X axis //atan2() returns in radians
  double Ay_acc = toDegrees(atan2(z_acc, x_acc)); //Theta: XZ plane, rotation about Y axis
  double Az_acc = toDegrees(atan2(x_acc, y_acc)); //Zeta: XY plane, rotation about Z axis

  double Beta = WEIGHT_GYRO*(prevBeta + x_gyro*dt) + WEIGHT_ACCEL*(Ax_acc);
  double Theta = WEIGHT_GYRO*(prevTheta + y_gyro*dt) + WEIGHT_ACCEL*(Ay_acc);
  double Zeta = WEIGHT_GYRO*(prevZeta + z_gyro*dt) + WEIGHT_ACCEL*(Az_acc);

  if(i%5 == 0){
    printXYZ(x_acc, y_acc, z_acc);
    printCalculatedAccelAngles(Ax_acc, Ay_acc, Az_acc);
    printXYZ(x_gyro, y_gyro, z_gyro);
    printXYZ(Beta, Theta, Zeta);
    Serial.println();
  }
  i++;
  
  prevBeta = Beta;
  prevTheta = Theta;
  prevZeta = Zeta;
  
  delay(dt * S_TO_MS);
}
