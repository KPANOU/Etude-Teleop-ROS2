// === motor_bridge_arduino.ino ===
const int pwmPin = 9;  // PWM vers contrôleur ZS-X11H
float accelValue = 0.0;

void setup() {
  Serial.begin(115200);
  pinMode(pwmPin, OUTPUT);
}

void loop() {
  if (Serial.available()) {
    accelValue = Serial.parseFloat();  // lit la valeur 0–1 envoyée
    int pwm = map(accelValue * 1000, 0, 1000, 0, 255);  // convertit en PWM 8 bits
    analogWrite(pwmPin, pwm);
  }
}
