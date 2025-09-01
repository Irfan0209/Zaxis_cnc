#include "OneButton.h"

#define STEP_PIN 8
#define DIR_PIN 9
#define EN 10

#define DELAY_STEP 800

// Jumlah step per cm (sesuaikan mekanik Anda)
#define STEPS_PER_CM 1000  

// Tombol OneButton
OneButton UP(3, true);
OneButton DOWN(2, true);
OneButton START(4, true);
OneButton RESET(11, true);   // tombol reset emergency

// Pin interrupt
#define EMERGENCY_PIN 5
#define LIMIT_BAWAH 6
#define LIMIT_ATAS 7

long posisi = 0;        
long maxPosisi = 10 * STEPS_PER_CM;
volatile bool emergencyStop = false;
volatile bool limitAtasTriggered = false;
volatile bool limitBawahTriggered = false;

void setup() {
  Serial.begin(9600);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN, OUTPUT);
  digitalWrite(EN, LOW);

  pinMode(LIMIT_BAWAH, INPUT_PULLUP);
  pinMode(LIMIT_ATAS, INPUT_PULLUP);
  pinMode(EMERGENCY_PIN, INPUT_PULLUP);

  // Attach tombol manual
  UP.attachClick(UPRUN);
  DOWN.attachClick(DOWNRUN);
  UP.attachDuringLongPress(UPRUN);
  DOWN.attachDuringLongPress(DOWNRUN);
  START.attachClick(startSequence);
  RESET.attachClick(resetEmergency);

  // Attach interrupt
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_PIN), isrEmergency, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_ATAS), isrLimitAtas, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_BAWAH), isrLimitBawah, FALLING);

  // Homing di awal
  //homing();
}

void loop() {
  UP.tick();
  DOWN.tick();
  START.tick();
  RESET.tick();

  // Jika emergency aktif, disable driver
  if (emergencyStop) {
    digitalWrite(EN, HIGH); // disable driver
    return;
  } else {
    digitalWrite(EN, LOW); // aktifkan driver
  }
}

// ----------------- Gerakan Manual -----------------
void UPRUN() {
  if (emergencyStop || limitAtasTriggered) return;
  moveSteps(1, HIGH);
}

void DOWNRUN() {
  if (emergencyStop || limitBawahTriggered) return;
  moveSteps(1, LOW);
}

// ----------------- START Sequence -----------------
void startSequence() {
  if (emergencyStop) return;

  Serial.println("START ditekan");

  long currentCM = posisi / STEPS_PER_CM;   // simpan posisi awal (cm)
  Serial.print("Posisi awal: "); Serial.print(currentCM); Serial.println(" cm");

  // Naik 3 cm dari posisi awal
  long targetNaik = posisi + (3 * STEPS_PER_CM);
  if (targetNaik > maxPosisi) targetNaik = maxPosisi;
  moveTo(targetNaik);

  // Turun kembali ke posisi awal
  long targetTurun = currentCM * STEPS_PER_CM;
  if (targetTurun < 0) targetTurun = 0;
  moveTo(targetTurun);

  Serial.print("Kembali ke posisi awal: "); Serial.print(currentCM); Serial.println(" cm");
}


// ----------------- Fungsi Gerak -----------------
void moveSteps(long steps, bool dir) {
  digitalWrite(DIR_PIN, dir);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(DELAY_STEP);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(DELAY_STEP);

  if (dir == HIGH) posisi++;
  else posisi--;
}

void moveTo(long targetPos) {
  if (targetPos < 0) targetPos = 0;  
  if (targetPos > maxPosisi) targetPos = maxPosisi;

  while (posisi != targetPos && !emergencyStop) {
    if (posisi < targetPos) {
      if (limitAtasTriggered) break;
      moveSteps(1, HIGH);
    } else {
      if (limitBawahTriggered) break;
      moveSteps(1, LOW);
    }
  }
}

// ----------------- Homing -----------------
void homing() {
  Serial.println("HOMING...");
  // Turun sampai limit bawah
  while (digitalRead(LIMIT_BAWAH) == HIGH && !emergencyStop) {
    moveSteps(1, LOW);
  }
  posisi = 0;
  limitBawahTriggered = false;

  // Naik sampai limit atas
  long counter = 0;
  while (digitalRead(LIMIT_ATAS) == HIGH && !emergencyStop) {
    moveSteps(1, HIGH);
    counter++;
  }
  maxPosisi = counter;
  posisi = maxPosisi;
  limitAtasTriggered = false;

  Serial.print("Travel terdeteksi: ");
  Serial.print(maxPosisi / STEPS_PER_CM);
  Serial.println(" cm");

  // Balik ke tengah
  moveTo(5 * STEPS_PER_CM);
}

// ----------------- ISR -----------------
void isrEmergency() {
  emergencyStop = true;
}

void isrLimitAtas() {
  limitAtasTriggered = true;
}

void isrLimitBawah() {
  limitBawahTriggered = true;
}

// ----------------- RESET Emergency -----------------
void resetEmergency() {
  if (emergencyStop) {
    Serial.println("RESET ditekan, emergency di-clear.");
    emergencyStop = false;
    limitAtasTriggered = false;
    limitBawahTriggered = false;
    digitalWrite(EN, LOW); // aktifkan driver lagi
  }
}
