// === motor_bridge_arduino_dir.ino ===

// --- Définition des broches ---
const int pwmPin = 9;        // PWM vers contrôleur ZS-X11H
const int DIR_PIN_BT = A2;   // Bouton poussoir
const int DIR_PIN_OUT = 8;   // Direction vers ZS-X11H

// --- Variables ---
float accelValue = 0.0;
bool dirState = 0;           // 0 = avant, 1 = arrière
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;  // anti-rebond en ms

void setup() {
  Serial.begin(115200);
  pinMode(pwmPin, OUTPUT);
  pinMode(DIR_PIN_OUT, OUTPUT);
  pinMode(DIR_PIN_BT, INPUT_PULLUP);  // bouton avec résistance interne activée

  digitalWrite(DIR_PIN_OUT, dirState);  // direction initiale
  Serial.println("✅ Motor bridge avec changement de direction prêt !");
}

void loop() {
  // --- Lecture de l'accélération (0.0 à 1.0) ---
  if (Serial.available()) {
    accelValue = Serial.parseFloat();
    accelValue = constrain(accelValue, 0.0, 1.0);
    int pwm = map(accelValue * 1000, 0, 1000, 0, 255);
    analogWrite(pwmPin, pwm);
  }

  // --- Lecture du bouton pour inversion de direction ---
  int buttonState = digitalRead(DIR_PIN_BT);
  if (buttonState == LOW && lastButtonState == HIGH && (millis() - lastDebounceTime) > debounceDelay) {
    dirState = !dirState;  // inverse la direction
    digitalWrite(DIR_PIN_OUT, dirState);
    Serial.print("🔄 Changement de direction : ");
    Serial.println(dirState ? "Arrière" : "Avant");
    lastDebounceTime = millis();
  }

  lastButtonState = buttonState;
}
