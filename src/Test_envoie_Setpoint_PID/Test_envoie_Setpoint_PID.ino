#include <PID_v1.h>

// --- Broches capteurs Hall ---
const int HALL1 = 2;
const int HALL2 = 5;
const int HALL3 = 6;

// --- Broches moteur ---
const int DIR_PIN_BT = A2;
const int DIR_PIN_OUT = 8;
const int PWM_PIN_OUT = 9;

// --- Variables de mesure ---
volatile unsigned long pulseCount = 0;
const unsigned long windowMs = 100;   // Fenêtre d’échantillonnage (ms)
unsigned long lastWindowTime = 0;
float feedback = 0.0;
const float alpha = 0.05;             // Filtrage exponentiel
const int pulses_per_rev = 45;        // 3 capteurs Hall × 15 impulsions = 45

// --- PID ---
double Input = 0, Output = 0, Setpoint = 0;
double Kp = 0.6, Ki = 0.4, Kd = 0.05;  // Gains initiaux
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// --- Divers ---
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

  // PID setup
  myPID.SetOutputLimits(0, 255);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(50);
  lastWindowTime = millis();

  Serial.println("PID Hoverboard (adapté ROS2 accel) prêt !");
}

void loop() {
  unsigned long now = millis();

  // --- Réception de la valeur d’accélérateur ---
  if (Serial.available()) {
    float accelValue = Serial.parseFloat();  // Valeur 0.0 – 1.0
    Setpoint = accelValue * MAX_RPM;         // Conversion en RPM cible
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
  analogWrite(PWM_PIN_OUT, (int)Output);

  // --- Direction ---
  if (!digitalRead(DIR_PIN_BT)) {
    dir = !dir;
    delay(300);
    analogWrite(PWM_PIN_OUT, 0);
    digitalWrite(DIR_PIN_OUT, dir);
  }

  // --- Monitoring série ---
  Serial.print(Setpoint); Serial.print(",");
  Serial.print(feedback, 2); Serial.print(",");
  Serial.println(Output, 2);
}

// --- ISR ---
void hallISR() {
  pulseCount++;
}
