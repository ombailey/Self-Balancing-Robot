
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050_6Axis_MotionApps612.h" // Uncomment this library to work with DMP 6.12 and comment on the above library.
#include <PID_v1.h>
/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;
//MPU6050 mpu(0x69); //Use for AD0 high
//MPU6050 mpu(0x68, &Wire1); //Use for AD0 low, but 2nd Wire (TWI/I2C) object.

#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL
//#define OUTPUT_TEAPOT

int const INTERRUPT_PIN = 2;  // Define the interruption #0 pin
bool blinkState;

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer
uint16_t FIFOCount;

/*---Orientation/Motion Variables---*/ 
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 gy;         // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector
double input, output;
double setpoint = 180;
double Kp = 1;
double Ki = 10;
double Kd = 0.1;
float pitch;
const int IN1 = 5;
const int IN2 = 6;
const int IN3 = 9;
const int IN4 = 10;
const int ENA = 3;
const int ENB = 11;
PID myPid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}




void setup() {
  // #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  //   Wire.begin();
  //   Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  // #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  //   Fastwire::setup(400, true);
  // #endif
  
  Serial.begin(115200); //115200 is required for Teapot Demo output
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXAccelOffset(-1125); //Set your accelerometer offset for axis X
  mpu.setYAccelOffset(-1525); //Set your accelerometer offset for axis Y
  mpu.setZAccelOffset(645); //Set your accelerometer offset for axis Z
  mpu.setXGyroOffset(124);  //Set your gyro offset for axis X
  mpu.setYGyroOffset(37);  //Set your gyro offset for axis Y
  mpu.setZGyroOffset(21);  //Set your gyro offset for axis Z

  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
    // Setup PID
    myPid.SetMode(AUTOMATIC);
    myPid.SetSampleTime(10);
    myPid.SetOutputLimits(-255, 255);
  } else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    };

  //  Initializing Motor Pins

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Turning off Motor Pins Initially

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void loop() {
    if (!DMPReady) return;
  /* Read a packet from FIFO */
  while (!MPUInterrupt && FIFOCount < packetSize){
    myPid.Compute();
    Serial.print(input); Serial.print("=>"); Serial.print(output); Serial.print("\n");
  
    if (input > 120 && input < 220) {        // If the Bot is falling 
      if (output > 0) {                  // Falling towards front 
        Forward();                     // Rotate the wheels forward 
      } else if (output < 0) {           // Falling towards back
        Reverse();                     // Rotate the wheels backward 
      }
    } else {                               // If Bot not falling
      Stop();                            // Hold the wheels still
    };

    // Reset Interrupt flag and get INT_STATUS byte
    MPUInterrupt = false;
    MPUIntStatus = mpu.getIntStatus();
    FIFOCount = mpu.getFIFOCount();
    // Check for overflow
    if ((MPUIntStatus & 0x10) || FIFOCount == 1024){
      mpu.resetFIFO();
      Serial.print(("FIFO overflow!"));
    }
    else if (MPUIntStatus & 0x02){
      while (FIFOCount < packetSize) FIFOCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(FIFOBuffer, packetSize);
      FIFOCount -= packetSize;
       /* Display Euler angles in degrees */
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      input = ypr[1] * 180/M_PI + 180;
    }
  }
  /* Blink LED to indicate activity */
  blinkState = !blinkState;
  digitalWrite(LED_BUILTIN, blinkState);

  }

  void Forward() {
    // Motor A 
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    // Motor B 
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Forward");
  }

  void Reverse() {
    // Motor A
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    // Motor B
    digitalWrite(IN3,LOW);
    digitalWrite(IN4, HIGH);
    Serial.println("Reverse");
  }

  void Stop(){
      // Motor A
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    // Motor B
    digitalWrite(IN3,LOW);
    digitalWrite(IN4, LOW);
    Serial.println("Stop");
  }


