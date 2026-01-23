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
float kp = 15;
float ki = 0;
float kd = 0;
float pwmMultiplier = (255.0/90.0);
float pwm;
float deadband = 8;

// Creating motor functions
void Reverse(float pwm){
  PORTD = (1 << PD4) | (1 << PD7);
  pwm = constrain(abs(pwm), 200,255);
  OCR2A = pwm;
  OCR1B = pwm;
};

void Forward(float pwm){
  PORTD = (1 << PD5) | (1 << PD6);
  pwm = constrain(abs(pwm), 200,255);
  OCR2A = pwm;
  OCR1B = pwm;
};

void Stop(){
  PORTD = ( 0 << PD4) | (0 << PD5) | (0 << PD6) | (0 << PD7);
  OCR2A = 0;
  OCR1B = 0;
}

float PID(float setpoint, float feedback, float kp, float ki, float kd, float timeInt){
  error = setpoint - feedback;
  integral += (error * timeInt);
  derivative = -gyDps;
  output = 200 + abs((kp * error) + (ki * integral) + (kd * derivative))/15;
  output = constrain(output,200,255);
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

  // Configuring Motors Pins
  DDRD |= (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7); 
  DDRB |= (1 << PB2) | (1 << PB3);
  TCCR1A = (1 << COM1B1) | (1 << WGM10);
  TCCR1B = (1 << WGM12) | (1 << CS11);
  TCCR2A = (1 << COM2A1) | (1 << WGM20) | (1 << WGM21);
  TCCR2B = (1 << CS21);

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
  float ax_g = ax/accLsbSens;
  float ay_g = ay/accLsbSens ;
  float az_g = az/accLsbSens ;

  float angleBias = 4.00;
  float pitch = (atan2(ax_g,sqrt(pow(az_g,2) + pow(ay_g,2))) * r2d) - angleBias;

  Serial.print("Pitch: "); Serial.println(pitch);
  pwm = PID(setpoint, pitch, kp, ki, kd, dt);
  if (pitch < -deadband){
    Forward(pwm);
  } else if (pitch > deadband) {
    Reverse(pwm);
  } else {
    Stop();
  };

  };

