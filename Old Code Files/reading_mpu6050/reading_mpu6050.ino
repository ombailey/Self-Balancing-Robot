
/*
  MPU6050 Raw

  A code for obtaining raw data from the MPU6050 module with the option to
  modify the data output format.

  Find the full MPU6050 library documentation here:
  https://github.com/ElectronicCats/mpu6050/wiki
*/
#include "I2Cdev.h"
#include "MPU6050.h"
#include <PID_v1.h>

/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;
//MPU6050 mpu(0x69);         //Use for AD0 high
//MPU6050 mpu(0x68, &Wire1); //Use for AD0 low, but 2nd Wire (TWI/I2C) object.

/* OUTPUT FORMAT DEFINITION----------------------------------------------------------------------------------
- Use "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated list of the accel 
X/Y/Z and gyro X/Y/Z values in decimal. Easy to read, but not so easy to parse, and slower over UART.

- Use "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit binary, one right after the other. 
As fast as possible without compression or data loss, easy to parse, but impossible to read for a human. 
This output format is used as an output.
--------------------------------------------------------------------------------------------------------------*/ 
#define OUTPUT_READABLE_ACCELGYRO
//#define OUTPUT_BINARY_ACCELGYRO

int16_t ax, ay, az;
int16_t gx, gy, gz;
float LSBg = 16384;
float LSBdps = 131;
bool blinkState;
double input, output;
double setpoint = 0;
double Kp = 1;
double Ki = 0.5;
double Kd = 0.1;
float pitch;

PID myPid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void Forward() {
  // Motor A 
  analogWrite(5, output);
  analogWrite(6, 0);
  // Motor B 
  analogWrite(9, output);
  analogWrite(10, 0);

  Serial.println("Moving Forward");
}

void Reverse() {
  // Motor A 
  analogWrite(5, 0);
  analogWrite(6, output*-1);
  // Motor B 
  analogWrite(9, 0);
  analogWrite(10, output*-1);

  Serial.println("Moving Reverse");
}

void Stop() {
  // Motor A 
  analogWrite(5, 0);
  analogWrite(6, 0);
  // Motor B 
  analogWrite(9, 0);
  analogWrite(10, 0);

  Serial.println("Moving Reverse");
}

void setup() {
  /*--Start I2C interface--*/
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin(); 
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(38400); //Initializate Serial wo work well at 8MHz/16MHz

  /*Initialize device and check connection*/ 
  Serial.println("Initializing MPU...");
  mpu.initialize();
  Serial.println("Testing MPU6050 connection...");
  if(mpu.testConnection() ==  false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }
  else{
    Serial.println("MPU6050 connection successful");
  }

  /* Use the code below to change accel/gyro offset values. Use MPU6050_Zero to obtain the recommended offsets */ 
  Serial.println("Updating internal sensor offsets...\n");
  mpu.setXAccelOffset(-1125); //Set your accelerometer offset for axis X
  mpu.setYAccelOffset(-1525); //Set your accelerometer offset for axis Y
  mpu.setZAccelOffset(645); //Set your accelerometer offset for axis Z
  mpu.setXGyroOffset(124);  //Set your gyro offset for axis X
  mpu.setYGyroOffset(37);  //Set your gyro offset for axis Y
  mpu.setZGyroOffset(21);  //Set your gyro offset for axis Z
  /*Print the defined offsets*/
  Serial.print("\t");
  Serial.print(mpu.getXAccelOffset());
  Serial.print("\t");
  Serial.print(mpu.getYAccelOffset()); 
  Serial.print("\t");
  Serial.print(mpu.getZAccelOffset());
  Serial.print("\t");
  Serial.print(mpu.getXGyroOffset()); 
  Serial.print("\t");
  Serial.print(mpu.getYGyroOffset());
  Serial.print("\t");
  Serial.print(mpu.getZGyroOffset());
  Serial.print("\n");

  /*Configure board LED pin for output*/ 
  pinMode(LED_BUILTIN, OUTPUT);
  
  myPid.SetMode(AUTOMATIC);
  myPid.SetSampleTime(10);
  myPid.SetOutputLimits(-255, 255);

  //  Initializing Motor Pins

  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  // Turning off Motor Pints Initially

  analogWrite(5, LOW);
  analogWrite(6, LOW);
  analogWrite(9, LOW);
  analogWrite(10, LOW);
}

void loop() {
  /* Read raw accel/gyro data from the module. Other methods commented*/
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // mpu.getAcceleration(&ax, &ay, &az);
  // mpu.getRotation(&gx, &gy, &gz);

  /*Print the obtained data on the defined format*/
  #ifdef OUTPUT_READABLE_ACCELGYRO
    // Redefining values in terms of m/s^2 for accelerometer and rad/s for gyrometer
    float g = -9.81;
    float radps = (PI/180);
    float ax_ms2 = (ax/LSBg)*g;
    float ay_ms2 = (ay/LSBg)*g;
    float az_ms2 = (az/LSBg)*g;
    float gx_ms2 = (gx/LSBdps)*radps;
    float gy_ms2 = (gy/LSBdps)*radps;
    float gz_ms2 = (gz/LSBdps)*radps;
    float pitch = atan2(ax_ms2, sqrt(pow(ay_ms2,2) + pow(az_ms2,2)) * 180/PI);
    input = pitch;

  // pid computes new output
    myPid.Compute();
    Serial.print(input); Serial.print("=>"); Serial.print(output); Serial.print("\n");
    if (input > 0 && input < 0.02) {        // If the Bot is falling 
      if (output > 0) {                  // Falling towards front 
        Forward();                     // Rotate the wheels forward 
      } else if (output < 0) {           // Falling towards back
        Reverse();                     // Rotate the wheels backward 
      }
    } else {                               // If Bot not falling
    Stop();                            // Hold the wheels still
    }
    Serial.print("a/g:\t");
    Serial.print("X-axis: \t"); Serial.print(ax_ms2); Serial.print(" m/s^2 \t");
    Serial.print("Y-axis: \t"); Serial.print(ay_ms2); Serial.print(" m/s^2 \t");
    Serial.print("Z-axis: \t"); Serial.print(az_ms2); Serial.print(" m/s^2 \t");
    Serial.print("Angular X-axis: \t"); Serial.print(gx_ms2); Serial.print(" rad/s \t");
    Serial.print("Angular Y-axis: \t");Serial.print(gy_ms2); Serial.print(" rad/s \t");
    Serial.print("Angular Z-axis: \t");Serial.print(gz_ms2); Serial.print(" rad/s \n");
  #endif

  #ifdef OUTPUT_BINARY_ACCELGYRO
    Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
    Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
    Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
    Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
    Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
    Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
  #endif

  /*Blink LED to indicate activity*/
  blinkState = !blinkState;
  digitalWrite(LED_BUILTIN, blinkState);
}
