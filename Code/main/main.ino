#include <Wire.h>

const int mpu = 0x68; //mpu I2C address
const float gyroLsbSens = 131; // LSB/degrees/second
const float accLsbSens = 16384; // LSB/g
const float g = 9.81; // m^2/s
int16_t ax;
int16_t ay;
int16_t az;
int16_t gx;
int16_t gy;
int16_t gz;
float accelZ;
float gyDps;
float gyBiasRawAvg;
float gyBias;
unsigned long prevTime;
unsigned long nowTime;
float dt ;
float angleY  = 0;
const float r2d = 180/PI;
// PID variables
float prevError = 0;
float integral = 0;
float derivative;
float setpoint = 0;
float output;
float error;
float kp = 8;
float ki = 0;
float kd = 0;
float pwmMultiplier = (255.0/90.0);
float pwm;

// Determining Angle due to accelerometer
float tiltAngle(float angle, float ax_g, float az_g){
  float alpha = 0.9;
  float accAngle = atan2(ax_g,az_g) * r2d;
  return alpha*(angle) + (1-alpha)*accAngle;
};

// Creating motor functions
void Reverse(float pwm){
  pwm = constrain(abs(pwm), 100, 255);
  // Setting PWM on Pin 5 - PD5
  OCR0B = pwm;

  // Setting PWM on Pin 10 - PB2
  OCR1B = pwm;
};

void Forward(float pwm){
  pwm = constrain(abs(pwm), 100, 255);
  // Setting PWM on Pin 9 - PB1
  OCR1A = pwm;

  // Setting PWM on Pin 6 - PD6
  OCR0A = pwm;
}

float PID(float setpoint, float feedback, float kp, float ki, float kd, float timeInt){
  error = setpoint - feedback;
  integral += (error * timeInt);
  derivative = -gyDps;
  output = ((kp * error) + (ki * integral) + (kd * derivative))* pwmMultiplier;
  output = constrain(output,-255,255);
  prevError = error;
  Serial.println(output);
  return output;
};

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(9600);
  delay(100);
  prevTime = micros();
  // Set ENA and ENB to HIGH on Motor Drivers
  DDRB |= (1 << 3);
  PORTB |= (1 << 3);
  DDRD |= (1 << 3);
  PORTD |= (1 << 3);

  // Turning on MPU 6050
  Wire.beginTransmission(mpu);
  Wire.write(0x6B); // Power Management Register
  Wire.write(0);
  Wire.endTransmission(true);

  // Determine Gyroscope Y bias
  float gyBiasRaw = 0;
  for (int i =0; i < 1000; i++){
    Wire.beginTransmission(mpu);
    Wire.write(0x45);
    Wire.endTransmission(false);
    Wire.requestFrom(mpu,2);
    int16_t gyRaw = Wire.read() << 8 | Wire.read();
    gyBiasRaw += gyRaw;
  }

  gyBiasRawAvg = gyBiasRaw / 1000;
  gyBias = gyBiasRawAvg/131; 
  Serial.print("gyBias: "); Serial.println(gyBias);

   // Setting PWM on Pin 5 - PD5
  DDRD |= (1 << 5);
  TCCR0A = (1 << 5) | (1 << 1) | (1 << 0);
  TCCR0B = (1 << 1);
  OCR0B = pwm;

  // Setting PWM on Pin 10 - PB2
  DDRB |= (1 << 2);
  TCCR1A = (1 << 5) | (1 << 0);
  TCCR1B = (1 << 3) | ( 1 << 1);
  OCR1B = pwm;

    // Setting PWM on Pin 9 - PB1
  DDRB |= (1 << 1);
  TCCR1A = (1 << 7) | (1 << 0);
  TCCR1B = (1 << 3) | (1 << 1);
  OCR1A = pwm;

  // Setting PWM on Pin 6 - PD6
  DDRD |= (1 << 6);
  TCCR0A = (1 << 7) | (1 << 1) | (1 << 0);
  TCCR0B = (1 << 1);
  OCR0A = pwm;
  
}

void loop() {

  // Creating bus with Gyroscope Y-Axis Register
  Wire.beginTransmission(mpu);
  Wire.write(0x3B); // Starting from AccelXOut register
  Wire.endTransmission(false);
  Wire.requestFrom(mpu,14);

  nowTime = micros();
  dt = (nowTime - prevTime) / pow(10,6);
  prevTime = nowTime;

  // Getting data from Accelerometer & Gyroscope

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();

  // Calculating current angle of the robot
  gyDps = (gy/gyroLsbSens) - gyBias; // degrees per second
  float ax_g = ax/accLsbSens;
  float az_g = az/accLsbSens;
  if (abs(gyDps) < 0.05){ // Remove Total RMS Noise
    gyDps = 0;
  };
  angleY += gyDps * dt;
  angleY = tiltAngle(angleY, ax_g, az_g);

  // Serial.print("Y-Angular Velocity: "); Serial.println(gyroY);
  Serial.print("AngleY: "); Serial.println(angleY);
  pwm = PID(setpoint, angleY, kp, ki, kd, dt);
  if (pwm > 0){
    Forward(pwm);
  } else {
    Reverse(pwm);
  };

  };

