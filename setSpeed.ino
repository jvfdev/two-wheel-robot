
const float m = 507.8; // Hz/V
const float b = -254.7; // Hz

float voltage = 0;
int output = 0;

void setSpeed(int inputSpeed) {
  if (inputSpeed > 0) {
    digitalWrite(motor0In1Pin, HIGH);
    digitalWrite(motor0In2Pin, LOW);
    digitalWrite(motor1In1Pin, HIGH);
    digitalWrite(motor1In2Pin, LOW);
  }
  else {
    digitalWrite(motor0In1Pin, LOW);
    digitalWrite(motor0In2Pin, HIGH);
    digitalWrite(motor1In1Pin, LOW);
    digitalWrite(motor1In2Pin, HIGH);
  }
  voltage = (float(abs(inputSpeed)) - b) / m;
  //  output = map(voltage, 0.0, 6.0, 0.0, 255.0);
  output = floatMap(voltage, 0.0, 6.0, 0.0, 255.0);
  output = constrain(output, 0, 255);
  analogWrite(motor0PWMPin, output);
  analogWrite(motor1PWMPin, output
  );
}

float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
