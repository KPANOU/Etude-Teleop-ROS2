#include <PID_v1.h>

// --- Broches capteurs Hall ---
const int HALL1 = 25;
const int HALL2 = 26;
const int HALL3 = 27;

// --- Broches moteur ---
const int DIR_PIN_OUT = 18;     // Direction vers ZS-X11H
const int PWM_PIN_OUT = 19;     // PWM vers ZS-X11H

// --- Variables de mesure ---
volatile unsigned long pulseCount = 0;
const unsigned long windowMs = 100;   // Fenêtre d’échantillonnage (ms)
unsigned long lastWindowTime = 0;
float feedback = 0.0;
const float alpha = 0.05;             // Filtrage exponentiel
const int pulses_per_rev = 45;        // 3 capteurs Hall × 15 impulsions

// --- PID ---
double Input = 0, Output = 0, Setpoint = 0;
double Kp = 0.6, Ki = 0.4, Kd = 0.05;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// --- Divers ---
bool dir = 0;
unsigned long lastSerialTime = 0;
const unsigned long serialTimeout = 1000;  // Timeout sécurité
const float MAX_RPM = 4000.0;              // RPM max mesuré à pleine pédale

// --- Pour compatibilité PWM ---
bool use_ledc = false;  // activé si LEDC disponible

void setup() {
  Serial.begin(115200);

  // Capteurs Hall
  pinMode(HALL1, INPUT_PULLUP);
  pinMode(HALL2, INPUT_PULLUP);
  pinMode(HALL3, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(HALL1), hallISR, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL2), hallISR, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL3), hallISR, RISING);

  pinMode(DIR_PIN_OUT, OUTPUT);
  pinMode(PWM_PIN_OUT, OUTPUT);

  // --- Configuration PWM (8 bits, 0–255) ---
  #if defined(ledcAttachPin)
    ledcAttachPin(PWM_PIN_OUT, 0);     // Canal 0
    ledcSetup(0, 1000, 8);             // 1 kHz, 8 bits
    use_ledc = true;
  #else
    use_ledc = false;
  #endif

  // --- PID ---
  myPID.SetOutputLimits(0, 255);   // sortie = PWM (0–255)
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(50);         // ms

  lastWindowTime = millis();
  Serial.println("✅ PID Hoverboard ESP32 prêt !");
}

void loop() {
  unsigned long now = millis();

  // --- Lecture du setpoint (valeur d’accélérateur depuis ROS) ---
  if (Serial.available()) {
    float accelValue = Serial.parseFloat();   // attendu 0..255
    Setpoint = (accelValue / 255.0) * MAX_RPM; // converti en RPM
    lastSerialTime = now;
  }

  // --- Sécurité : stop si plus de données depuis ROS ---
  if (now - lastSerialTime > serialTimeout) {
    Setpoint = 0;
  }

  // --- Mesure RPM ---
  if (now - lastWindowTime >= windowMs) {
    noInterrupts();
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    float revs = (float)pulses / (float)pulses_per_rev;
    float rpm_instant = (revs * 60000.0) / (float)windowMs;  // en RPM
    feedback = alpha * rpm_instant + (1.0 - alpha) * feedback; // filtre doux

    Input = feedback;
    lastWindowTime = now;
  }

  // --- PID ---
  myPID.Compute();
  applyPWM((int)Output);

  // --- Monitoring série ---
  Serial.print("Setpoint: ");
  Serial.print(Setpoint, 2);
  Serial.print(" | Feedback: ");
  Serial.print(feedback, 2);
  Serial.print(" | Output: ");
  Serial.println(Output, 2);
}

// --- Applique la consigne PWM selon la méthode disponible ---
void applyPWM(int value) {
  value = constrain(value, 0, 255);
  if (use_ledc) {
    ledcWrite(0, value);
  } else {
    analogWrite(PWM_PIN_OUT, value);
  }
}

// --- ISR : comptage des impulsions Hall ---
void hallISR() {
  pulseCount++;
}
