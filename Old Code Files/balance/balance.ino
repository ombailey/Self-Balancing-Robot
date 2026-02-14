#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;

int const INTERRUPT_PIN = 2;  // Define the interruption #0 pin
bool blinkState;


/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

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
double speed;
double setpoint = 0;
// Original kp 15, ki, 56, kd 0.3
double Kp = 17, Ki = 50, Kd = 0.5;
float pitch;
const int IN1 = 4, IN2 = 5, IN3 = 6, IN4 = 7;
const int ENA = 10, ENB = 11;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}

// Motor Functions
void Forward() {
  // 5.667
  speed = abs(output * 5.667);
  // Motor A 
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
  // Motor B 
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
  Serial.println("Reverse");
}

void Reverse() {
  speed = abs(output * 5.667);
  // Motor A
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);
  // Motor B
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
  Serial.println("Forward");
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

void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  
  Serial.begin(115200); 

  /*Initialize device*/
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  if(mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }
  else {
    Serial.println("MPU6050 connection successful");
  }

  /* Initializate and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(-1125);
  mpu.setYGyroOffset(-1525);
  mpu.setZGyroOffset(645);
  mpu.setXAccelOffset(124);
  mpu.setYAccelOffset(37);
  mpu.setZAccelOffset(21);

  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    /*Enable Arduino interrupt detection*/
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  } 
  else {
    Serial.print(F("DMP Initialization failed (code ")); //Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }
  pinMode(LED_BUILTIN, OUTPUT);

  // Initializing PID parameters
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
  myPID.SetOutputLimits(-255, 255);

  //  Initializing Motor Pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

}

void loop() {
  if (!DMPReady) return; // Stop the program if DMP programming fails.
    
  /* Read a packet from FIFO */
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
      /* Display Euler angles in degrees */
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      input = ypr[1] * 180/M_PI; // Pitch 
  };
  myPID.Compute();
  Serial.print(input); Serial.print("=>"); Serial.print(output); Serial.print("\n");
  
  if (output < 0) {
    Forward();
  } else if (output > 0) {
    Reverse();
  } else {
    Stop();
  }
 
  /* Blink LED to indicate activity */
  blinkState = !blinkState;
  digitalWrite(LED_BUILTIN, blinkState);
  }



