const int PWM_PIN = 18;
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 1000;
const int PWM_RES = 8;

void setup() {
  Serial.begin(115200);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);

  Serial.println("ESP32 ready to receive ACC:XXX");
}

void loop() {
  if (Serial.available()) {
    String rx = Serial.readStringUntil('\n');

    if (rx.startsWith("ACC:")) {
      int pwmValue = rx.substring(4).toInt();
      pwmValue = constrain(pwmValue, 0, 255);

      ledcWrite(PWM_CHANNEL, pwmValue);

      Serial.print("PWM updated to: ");
      Serial.println(pwmValue);
    }
  }
}
