//https://www.instructables.com/Arduino-Self-Balancing-Robot-1/


// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
/* Wire Connections
   VCC - 5V
   GND - GND
   SCL - A5
   SDA - A4
   XDA - NC
   XCL - NC
   AD0 - NC
   INT - 2
*/

#include<Wire.h>
#include<math.h>


const byte ledPin = 13;
const byte motor0PWMPin = 3;
const byte motor0In1Pin = 4;
const byte motor0In2Pin = 5;

const byte motor1PWMPin = 6;
const byte motor1In1Pin = 7;
const byte motor1In2Pin = 9;


//based on Ziegler-Nichols
// Kp_cr: Lowest value of Kp (when I, D = 0), where system oscillates
// P_cr: Period of oscillations
//const float Kp = 0.6*Kcr;
//const float Kd = Kcr * 0.125 * P_cr;
//const float Ki = Kcr * 1.0/(0.5*P_cr);

// Kp_cr = 450
// P_cr = 0.35

// Passable constants
//voltage into L298N = 9.0V
//const float Kp = 0.6 * 450.0;
//const float Kd = 0.125 * 0.35;
//const float Ki = 100.0 / (0.5 * 0.35);

const float Kp = 0.6 * 450.0;
const float Kd = 0.125 * 0.35;
const float Ki = 158.0 / (0.5 * 0.35);

const float alpha = 0.0066;
const int maxError = 3000;

//const float reverseCorrection = 1.1; // motors move slower in reverse direction for some reason, this corrects it.

const float sampleTime = 0.001; // seconds
const float targetAngle = 0.4; //degrees

// MPU6050 Setup
const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyY, GyZ;

// calibration values
const float xCal = 16596.0;
const float yCal = 16845.0;
const float zCal = 17742.0;
const int GyXCal = 854;

volatile int16_t gyroRate, GyX;
volatile float AcXg, AcYg, AcZg, accAngle = 0, gyroAngle = 0;
volatile float currentAngle, prevAngle = 0, error, prevError = 0, errorSum = 0, motorSpeed = 0;


volatile byte count = 0;


unsigned long currTime, prevTime = 0, loopTime;


void setup() {
  

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);

  pinMode(ledPin, OUTPUT);
  pinMode(motor0PWMPin, OUTPUT);
  pinMode(motor0In1Pin, OUTPUT);
  pinMode(motor0In2Pin, OUTPUT);
  pinMode(motor1PWMPin, OUTPUT);
  pinMode(motor1In1Pin, OUTPUT);
  pinMode(motor1In2Pin, OUTPUT);

  digitalWrite(motor0In1Pin, LOW);
  digitalWrite(motor0In2Pin, LOW);
  digitalWrite(motor1In1Pin, LOW);
  digitalWrite(motor1In2Pin, LOW);
  analogWrite(motor0PWMPin, 0);
  analogWrite(motor1PWMPin, 0);

  DDRB |= 0b00010000;
  PORTB |= 0b00010000; // turn on pin 12



  for (int i = 0; i < 6; i++) {
    digitalWrite(ledPin, LOW);
    delay(250);
    digitalWrite(ledPin, HIGH);
    delay(250);

  }

  init_PID();
}


void loop() {
  // PORTB |= 0b00010000; // turn on pin 12
  updateSensorValues();
  // PORTB &= 0b11101111; // turn off pin 12
  //  Serial.println(currentAngle);
  //    Serial.print("motor speed: ");
  //    Serial.print(motorSpeed);
  //    Serial.print(", V:");
  //    Serial.print(voltage);
  //    Serial.print(", output: ");
  //    Serial.println(output);
  //
  //  Serial.println(errorSum);
  setSpeed(motorSpeed);
}


void updateSensorValues() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  AcXg = AcX / xCal;
  AcYg = AcY / yCal;
  AcZg = AcZ / zCal;
  GyX = GyX + GyXCal;

  /*
    Serial.print("AcX = "); Serial.print(AcX);
    Serial.print(" | AcY = "); Serial.print(AcY);
    Serial.print(" | AcZ = "); Serial.print(AcZ);
    Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
    Serial.print(" | GyX = "); Serial.print(GyX);
    Serial.print(" | GyY = "); Serial.print(GyY);
    Serial.print(" | GyZ = "); Serial.println(GyZ);
  */
}

ISR(TIMER1_COMPA_vect) {
  //This interrupt activated every time timer comparitor triggers
  PORTB |= 0b00010000; // turn on pin 12
  accAngle = atan2(AcYg, AcZg) * RAD_TO_DEG;
  gyroRate = map(GyX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate * sampleTime;
  currentAngle = (1.0 - alpha) * (prevAngle + gyroAngle) + alpha * (accAngle);

  error = currentAngle - targetAngle;
  errorSum = errorSum + error;
  errorSum = constrain(errorSum, -maxError, maxError);
  motorSpeed = Kp * error + Ki * errorSum * sampleTime - Kd * (currentAngle - prevAngle) / sampleTime;
  //  motorSpeed = constrain(motorSpeed, -255, 255);

  prevAngle = currentAngle;
  //turns the LED on/off at 1 Hz to indicate functioning/
  count++;
  if (count == 100) {
    count = 0;
    digitalWrite(ledPin, !digitalRead(ledPin));
  }
  PORTB &= 0b11101111; // turn off pin 12
}
