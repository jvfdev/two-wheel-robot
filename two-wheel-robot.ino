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

// MPU6050 Setup
const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, gyroRate;
float gyroAngle = 0;
// calibration values
float xCal = 16596.0;
float yCal = 16845.0;
float zCal = 17742.0;
int GyXCal = 854;
float AcXg, AcYg, AcZg;
float currentAngle, previousAngle = 0;

const float alpha = .0066;
byte count = 0;


unsigned long currTime, prevTime = 0, loopTime;


void init_PID() {
  // Sets up timer to run control loop at 200 Hz
  noInterrupts(); // disable interrupts before setting registers
  TCCR1A = 0; //sets register to zero, not yet sure why this is here.
  TCCR1B = 0; // same as above
  OCR1A = 9999; // sets compare match register
  TCCR1B |= (1 << WGM12); // turns on CTC
  TCCR1B |= (1 << CS11); // sets prescaler to 8
  TIMSK1 |= (1 << OCIE1A); // enable the interrupt
  interrupts();
}

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
  init_PID();
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
}
void loop() {
  updateSensorValues();
  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;
  //  Serial.println(GyX+854);
  gyroRate = map(GyX, -32768, 32767, -250, 250);
  gyroAngle = gyroAngle + (float)gyroRate * loopTime / 1000.0;
  //  Serial.print("X: ");
  //  Serial.print(AcXg);
  //  Serial.print(", Y: ");
  //  Serial.println(AcYg);
  float angle = atan2(AcYg, AcZg) * RAD_TO_DEG;
  //  Serial.print("acc ang: ");
  //  Serial.print(angle);
  //  Serial.print(", gyro ang: ");
  //  Serial.println(gyroAngle);
  currentAngle = alpha * (previousAngle + gyroAngle) + (1.0 - alpha) * (angle);
  Serial.println(currentAngle);
  delay(250);
//  digitalWrite(ledPin, !digitalRead(ledPin));
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

ISR(TIMER1_COMPA_vect){
  //This interrupt activated every time timer comparitor triggers
  
  //turns the LED on/off at 1 Hz to indicate functioning
  count++;
  if(count == 100){
    count = 0;
    digitalWrite(ledPin, !digitalRead(ledPin));
  }
  }
