#include <Wire.h>

const int mpu = 0x68; //mpu I2C address
const float gyroLsbSens = 131; // LSB/degrees/second
const float accLsbSens = 16384; // LSB/g
const float g = 9.81; // m^2/s
const float alpha = 0.985;
const unsigned long loopPeriod = 2000; // 500 Hz
int16_t ax;
int16_t ay;
int16_t az;
int16_t gx;
int16_t gy;
int16_t gz;
float accelZ;
float gyDps;
int32_t gyBiasRaw = 0;
float gyBiasRawAvg;
float gyBias;
unsigned long prevTime = 0;
unsigned long nowTime;
float dt ;
float gyAngle = 0;
float compAngle = 0;
const float r2d = 180/PI;
const float d2r = PI/180;
// PID variables
float prevError = 0;
float integral = 0;
float derivative;
float setpoint = 1.90;
float output;
float error;
float kpScale = 6.5;
float kdScale = 4.15;
float kp = -13.41622596;
float kd = -0.95875022;
float ki = 0;
float pwm;
float prop;

// Creating motor functions
void Reverse(float pwm){
  PORTD = (1 << PD4) | (1 << PD7);
   pwm = abs(pwm);
  OCR2A = pwm;
  OCR1B = pwm;
};

void Forward(float pwm){
  PORTD = (1 << PD5) | (1 << PD6);
  OCR2A = pwm;
  OCR1B = pwm;
};

void Brake(){
  PORTD = ( 1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7);
  OCR2A = 0;
  OCR1B = 0;
}

float PolePlace(float setpoint, float currentAngle, float gyDps, float kp, float ki, float kd){
  error = setpoint - currentAngle;
  derivative = -gyDps;
  integral += error * dt;
  integral = constrain(integral,-50,50);
  output = -(kp*error*kpScale + ki*integral+ kd*derivative*kdScale);
  output = constrain(output,-255,255);
  prevError = error;
  return output;
};

void setup() {

  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  // Turning on MPU 6050
  Wire.beginTransmission(mpu);
  Wire.write(0x6B); // Power Management Register
  Wire.write(0);
  Wire.endTransmission(true);

  // Determine Gyroscope Y bias
  for (int i =0; i < 5000; i++){
    Wire.beginTransmission(mpu);
    Wire.write(0x45);
    Wire.endTransmission(false);
    Wire.requestFrom(mpu,2);
    int16_t gyRaw = Wire.read() << 8 | Wire.read();
    gyBiasRaw += gyRaw;
  };

  gyBiasRawAvg = gyBiasRaw / 5000.0;
  gyBias = gyBiasRawAvg/gyroLsbSens; 

  // Configuring Motors Pins
  DDRD |= (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7); 
  DDRB |= (1 << PB2) | (1 << PB3);
  TCCR1A = (1 << COM1B1) | (1 << WGM10);
  TCCR1B = (1 << WGM12) | (1 << CS11);
  TCCR2A = (1 << COM2A1) | (1 << WGM20) | (1 << WGM21);
  TCCR2B = (1 << CS21);

  prevTime = micros();
};

void loop() {

  nowTime = micros();

  if (nowTime- prevTime < loopPeriod) {
    return;
    }
  
  prevTime += loopPeriod;
  dt = 0.002;

  // Creating bus with Gyroscope Y-Axis Register
  Wire.beginTransmission(mpu);
  Wire.write(0x3B); // Starting from AccelXOut register
  Wire.endTransmission(false);
  Wire.requestFrom(mpu,14);
  
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
  gyDps = -(gy-gyBiasRawAvg)/gyroLsbSens;
  gyAngle += gyDps *dt;
  float angleBias = 4.00;

  // float pitch = (atan2(ax_g,sqrt((az_g*az_g) + (ay_g*ay_g))) * r2d) - angleBias;
  float pitch = (atan2(ax_g,sqrt((az_g*az_g) + (ay_g*ay_g))) * r2d);
  compAngle = alpha*(compAngle + (gyDps *dt)) + (1.0-alpha)*pitch;
  pwm = PolePlace(setpoint, compAngle, gyDps, kp, ki, kd);

  if (pwm > 0){
    Forward(pwm);  
  } else if (pwm < 0) {
    Reverse(pwm);
  } 
};

