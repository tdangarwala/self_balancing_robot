#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h> //I2C library for communication with MPU-6050

int MPU_addr = 0x68; // I2C address of the MPU-6050

const byte encoder0pinA = 2;
const byte encoder0pinB = 4;
volatile byte encoder0LastStateA;
volatile int duration; //number of pulses
volatile boolean direction; // true for forward, false for backward

const float WHEEL_DIAMETER = 0.065;
const int PULSES_PER_REVOLUTION = 2797;
const float METERS_PER_PULSE = (PI * WHEEL_DIAMETER) / PULSES_PER_REVOLUTION;

float x = 0;
float x_dot = 0;
long total_pulses = 0;

// put function declarations here:
void wheelSpeed();
float controller(float x, float x_dot, float theta, float thetadot);
void commandMotor(float u, bool dir);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Initialize MPU-6050
  Wire.begin(); // Initialize I2C communication
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Set to zero (wakes up the MPU-605
  if(Wire.endTransmission() != 0) {
    Serial.println("Failed to initialize MPU-6050");
    while(1); // Halt if initialization fails
  }

  Serial.println("MPU-6050 initialized successfully");

  // Initialize encoder pins
  pinMode(encoder0pinA, INPUT);
  pinMode(encoder0pinB, INPUT);
  direction = true; // Assuming initial direction is forward
  encoder0LastStateA = digitalRead(encoder0pinA); // Read initial state of pin A

  attachInterrupt(digitalPinToInterrupt(encoder0pinA), wheelSpeed, CHANGE); // Interrupt on change of pin A
  Serial.println("Encoder initialized successfully");
}

void loop() {
  // put your main code here, to run repeatedly:

  //little endian better for machine operations, big endian better for human readability

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // Start with Accel X register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // Request 14 registers requested as each measurement is 2 bytes and I need 7 measurements

  // Read Accelerometer
  int16_t ax = Wire.read()<<8 | Wire.read(); //ORing the two bytes together to get full 16 bit value. OR is same as add but simpler
  int16_t ay = Wire.read()<<8 | Wire.read();
  int16_t az = Wire.read()<<8 | Wire.read();
  
  // Skip Temperature (2 bytes)
  Wire.read(); Wire.read();

  // Read Gyroscope
  int16_t gx = Wire.read()<<8 | Wire.read();
  int16_t gy = Wire.read()<<8 | Wire.read();
  int16_t gz = Wire.read()<<8 | Wire.read();

  // Conversion factors for default settings
  float accScale = 16384.0;
  float gyroScale = 131.0;

  float ax_m_s2 = ax / accScale;
  float az_m_s2 = az / accScale;
  float gx_deg_s = gx / gyroScale;

  float theta = atan2(-ax_m_s2, az_m_s2); 
  float thetadot = gx_deg_s; 
  
  noInterrupts();
  int pulses_this_interval = duration;
  duration = 0;
  total_pulses += pulses_this_interval;
  interrupts();

  x = total_pulses * METERS_PER_PULSE;
  x_dot = (pulses_this_interval * METERS_PER_PULSE) / 0.1;

  float u = controller(x, x_dot, theta, thetadot);

  Serial.print("Theta (rad): ");
  Serial.print(theta, 4);
  Serial.print(" | Theta dot (deg/s): ");
  Serial.println(thetadot, 4);
  Serial.print(" | x: "); Serial.print(x, 4);
  Serial.print(" | x_dot: "); Serial.println(x_dot, 4);


  delay(100); 

  

}

// put function definitions here:

void wheelSpeed(){
  int Lstate = digitalRead(encoder0pinA);
  if((encoder0LastStateA == LOW) && (Lstate == HIGH)){
    int val = digitalRead(encoder0pinB);
    if (val == LOW && direction){
      direction = false;
    }
    else if (val == HIGH && !direction){
      direction = true;
    }
  }
  
  encoder0LastStateA = Lstate;

  if(!direction){
    duration++;
  }
  else{
    duration--;
  }
}

float controller(float x, float x_dot, float theta, float thetadot){
  // implement LQR controller 
  float k1 = -1.0;
  float k2 = -3.037;
  float k3 = -69.774;
  float k4 = -13.843;

  float u = k1*x + k2*x_dot + k3*theta + k4*thetadot;

  return u;

}

void commandMotor(float u, bool dir){
  // convert output from LQR to motor command
  // u is torque needed to balance robot, need to convert to pwm
  float max_torque = 1.765; // stall torque of motor in Nm
  int max_pwm = 255;

  //there are 2 motors so each motor needs to provide half the torque
  u = u / 2.0;

  int pwm = (int)(abs(u) / max_torque * max_pwm);
  pwm = constrain(pwm, 0, max_pwm);

  // set motor direction based on sign of u
  if (u > 0){
    // forward
    // digitalWrite(motorPin1, HIGH);
    // digitalWrite(motorPin2, LOW);
    // analogWrite(motorPin1, pwm);
  }
  else {
    // backward
    // digitalWrite(motorPin1, LOW);
    // digitalWrite(motorPin2, HIGH);
    // analogWrite(motorPin2, pwm);
  }
}