#include <PID_v1.h>

// === Broches (adaptées ESP32) ===
#define HALL1 27
#define HALL2 26
#define HALL3 25
#define DIR_PIN_BT 34       // entrée (remplace ton bouton)
#define DIR_PIN_OUT 18      // sortie DIR vers ZS-X11H
#define PWM_PIN_OUT 19      // sortie PWM vers ZS-X11H

// === Variables de mesure ===
volatile unsigned long pulseCount = 0;
const unsigned long windowMs = 100;   // Fenêtre d’échantillonnage (ms)
unsigned long lastWindowTime = 0;
float feedback = 0.0;
const float alpha = 0.05;             // Filtrage exponentiel
const int pulses_per_rev = 45;        // 3 capteurs Hall × 15 impulsions = 45

// === PID ===
double Input = 0, Output = 0, Setpoint = 0;
double Kp = 0.6, Ki = 0.4, Kd = 0.05;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// === Divers ===
bool dir = 0;
unsigned long lastSerialTime = 0;
const unsigned long serialTimeout = 1000;  // Timeout sécurité
const float MAX_RPM = 255.0;               // Vitesse max cible

void setup() {
  Serial.begin(115200);

  // Capteurs Hall
  pinMode(HALL1, INPUT_PULLUP);
  pinMode(HALL2, INPUT_PULLUP);
  pinMode(HALL3, INPUT_PULLUP);

  // Interruptions
  attachInterrupt(digitalPinToInterrupt(HALL1), hallISR, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL2), hallISR, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL3), hallISR, RISING);

  // Moteur
  pinMode(DIR_PIN_BT, INPUT_PULLUP);
  pinMode(DIR_PIN_OUT, OUTPUT);
  pinMode(PWM_PIN_OUT, OUTPUT);

  // === PWM ESP32 ===
  ledcAttachPin(PWM_PIN_OUT, 0);   // Canal 0
  ledcSetup(0, 1000, 8);           // 1 kHz, 8 bits de résolution

  // PID setup
  myPID.SetOutputLimits(0, 255);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(50);
  lastWindowTime = millis();

  Serial.println("✅ PID Hoverboard ESP32 prêt !");
}

void loop() {
  unsigned long now = millis();

  // --- Réception série ---
  if (Serial.available()) {
    float accelValue = Serial.parseFloat();  // Valeur 0–1
    Setpoint = accelValue * MAX_RPM;
    lastSerialTime = now;
  }

  // Sécurité : arrêt si pas de signal
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
    float rpm_instant = (revs * 60000.0) / (float)windowMs;

    feedback = alpha * rpm_instant + (1.0 - alpha) * feedback;
    Input = feedback;
    lastWindowTime = now;
  }

  // --- PID ---
  myPID.Compute();
  ledcWrite(0, (int)Output);  // PWM ESP32

  // --- Changement de direction ---
  if (!digitalRead(DIR_PIN_BT)) {
    dir = !dir;
    delay(300);
    ledcWrite(0, 0);  // stop avant inversion
    digitalWrite(DIR_PIN_OUT, dir);
  }

  // --- Monitoring ---
  Serial.print(Setpoint); Serial.print(",");
  Serial.print(feedback, 2); Serial.print(",");
  Serial.println(Output, 2);
}

// === ISR capteurs Hall ===
void hallISR() {
  pulseCount++;
}
