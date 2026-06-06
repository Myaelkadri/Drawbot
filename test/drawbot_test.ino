// ============================================================
//  DRAWBOT – Code de test complet
//  ECE Paris – Systèmes Bouclés 2026
//  À téléverser via VSCode (extension Arduino / PlatformIO)
//  Carte cible : NodeMCU ESP32
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>

// ---------- Définitions des pins (slide 6 du cours) ----------

// LEDs utilisateur
#define LEDU1 25
#define LEDU2 26

// Enable moteurs
#define EN_D 23
#define EN_G  4

// Commande PWM moteur droit
#define IN_1_D 19
#define IN_2_D 18

// Commande PWM moteur gauche
#define IN_1_G 17
#define IN_2_G 16

// Encodeurs
#define ENC_G_CH_A 32
#define ENC_G_CH_B 33
#define ENC_D_CH_A 27
#define ENC_D_CH_B 14

// I2C
#define SDA_PIN 21
#define SCL_PIN 22

// Adresses I2C des capteurs
#define ADDR_IMU 0x6B   // LSM6DS3
#define ADDR_MAG 0x1E   // LIS3MDL

// ---------- Paramètres PWM (LEDC ESP32) ----------
#define PWM_FREQ      20000   // 20 kHz  (inaudible)
#define PWM_RES       8       // 8 bits → 0‑255
#define CH_IN1_D      0
#define CH_IN2_D      1
#define CH_IN1_G      2
#define CH_IN2_G      3

// ---------- Variables encodeurs (volatile = modifiées par ISR) ----------
volatile long encG = 0;
volatile long encD = 0;

// ---------- Interruptions encodeurs ----------
void IRAM_ATTR isrEncG() {
  if (digitalRead(ENC_G_CH_B)) encG++;
  else                          encG--;
}

void IRAM_ATTR isrEncD() {
  if (digitalRead(ENC_D_CH_B)) encD++;
  else                          encD--;
}

// ---------- Fonctions utilitaires moteurs ----------

void motorStop() {
  ledcWrite(CH_IN1_D, 0);
  ledcWrite(CH_IN2_D, 0);
  ledcWrite(CH_IN1_G, 0);
  ledcWrite(CH_IN2_G, 0);
}

// vitesse : -255 (arrière) … +255 (avant)
void setMotorD(int vitesse) {
  vitesse = constrain(vitesse, -255, 255);
  if (vitesse >= 0) {
    ledcWrite(CH_IN1_D, vitesse);
    ledcWrite(CH_IN2_D, 0);
  } else {
    ledcWrite(CH_IN1_D, 0);
    ledcWrite(CH_IN2_D, -vitesse);
  }
}

void setMotorG(int vitesse) {
  vitesse = constrain(vitesse, -255, 255);
  if (vitesse >= 0) {
    ledcWrite(CH_IN1_G, vitesse);
    ledcWrite(CH_IN2_G, 0);
  } else {
    ledcWrite(CH_IN1_G, 0);
    ledcWrite(CH_IN2_G, -vitesse);
  }
}

// ---------- Scan I2C ----------
void scanI2C() {
  Serial.println("\n=== Scan I2C ===");
  int found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Périphérique trouvé à 0x%02X", addr);
      if (addr == ADDR_IMU) Serial.print("  <-- IMU LSM6DS3  OK");
      if (addr == ADDR_MAG) Serial.print("  <-- MAG LIS3MDL  OK");
      Serial.println();
      found++;
    }
  }
  if (found == 0) Serial.println("  Aucun périphérique I2C detecté !");
  else Serial.printf("  %d périphérique(s) trouvé(s)\n", found);
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n========================================");
  Serial.println("   DRAWBOT – Test de démarrage");
  Serial.println("   ECE Paris – Systèmes Bouclés 2026");
  Serial.println("========================================\n");

  // ----- LEDs -----
  pinMode(LEDU1, OUTPUT);
  pinMode(LEDU2, OUTPUT);
  digitalWrite(LEDU1, LOW);
  digitalWrite(LEDU2, LOW);

  // ----- Enable moteurs -----
  pinMode(EN_D, OUTPUT);
  pinMode(EN_G, OUTPUT);
  digitalWrite(EN_D, HIGH);   // activer les drivers
  digitalWrite(EN_G, HIGH);

  // ----- PWM moteurs (API LEDC ESP32) -----
  ledcSetup(CH_IN1_D, PWM_FREQ, PWM_RES);
  ledcSetup(CH_IN2_D, PWM_FREQ, PWM_RES);
  ledcSetup(CH_IN1_G, PWM_FREQ, PWM_RES);
  ledcSetup(CH_IN2_G, PWM_FREQ, PWM_RES);

  ledcAttachPin(IN_1_D, CH_IN1_D);
  ledcAttachPin(IN_2_D, CH_IN2_D);
  ledcAttachPin(IN_1_G, CH_IN1_G);
  ledcAttachPin(IN_2_G, CH_IN2_G);

  motorStop();

  // ----- Encodeurs -----
  pinMode(ENC_G_CH_A, INPUT_PULLUP);
  pinMode(ENC_G_CH_B, INPUT_PULLUP);
  pinMode(ENC_D_CH_A, INPUT_PULLUP);
  pinMode(ENC_D_CH_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_G_CH_A), isrEncG, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_D_CH_A), isrEncD, RISING);

  // ----- I2C -----
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);   // 400 kHz
  scanI2C();

  Serial.println("\nDémarrage des tests dans 2 s …");
  delay(2000);
}

// ---------- Loop : séquence de tests ----------

// États de la machine de test
enum TestState {
  TEST_LED,
  TEST_MOTOR_D_FWD,
  TEST_MOTOR_D_BWD,
  TEST_MOTOR_G_FWD,
  TEST_MOTOR_G_BWD,
  TEST_BOTH_FWD,
  TEST_ENCODER,
  TEST_I2C,
  TEST_DONE
};

TestState state       = TEST_LED;
unsigned long tStart  = 0;
int           step    = 0;

void loop() {
  unsigned long now = millis();

  switch (state) {

    // ---- Test LEDs (clignotement alterné) ----
    case TEST_LED:
      if (step == 0) {
        Serial.println(">>> TEST 1 : LEDs");
        tStart = now;
        step = 1;
      }
      if (now - tStart < 3000) {
        bool tog = ((now / 300) % 2);
        digitalWrite(LEDU1, tog);
        digitalWrite(LEDU2, !tog);
      } else {
        digitalWrite(LEDU1, LOW);
        digitalWrite(LEDU2, LOW);
        Serial.println("    LEDs OK");
        step = 0;
        state = TEST_MOTOR_D_FWD;
      }
      break;

    // ---- Test moteur droit avant ----
    case TEST_MOTOR_D_FWD:
      if (step == 0) {
        Serial.println(">>> TEST 2 : Moteur DROIT – avant (150/255)");
        encD = 0;
        setMotorD(150);
        tStart = now;
        step = 1;
      }
      if (now - tStart > 1500) {
        motorStop();
        Serial.printf("    Encodeur droit : %ld ticks\n", encD);
        if (encD > 5) Serial.println("    Moteur + encodeur droit OK");
        else           Serial.println("    /!\\ Encodeur droit ne compte pas – vérifier câblage");
        step = 0;
        delay(500);
        state = TEST_MOTOR_D_BWD;
      }
      break;

    // ---- Test moteur droit arrière ----
    case TEST_MOTOR_D_BWD:
      if (step == 0) {
        Serial.println(">>> TEST 3 : Moteur DROIT – arrière");
        encD = 0;
        setMotorD(-150);
        tStart = now;
        step = 1;
      }
      if (now - tStart > 1500) {
        motorStop();
        Serial.printf("    Encodeur droit arrière : %ld ticks\n", encD);
        step = 0;
        delay(500);
        state = TEST_MOTOR_G_FWD;
      }
      break;

    // ---- Test moteur gauche avant ----
    case TEST_MOTOR_G_FWD:
      if (step == 0) {
        Serial.println(">>> TEST 4 : Moteur GAUCHE – avant (150/255)");
        encG = 0;
        setMotorG(150);
        tStart = now;
        step = 1;
      }
      if (now - tStart > 1500) {
        motorStop();
        Serial.printf("    Encodeur gauche : %ld ticks\n", encG);
        if (encG > 5) Serial.println("    Moteur + encodeur gauche OK");
        else           Serial.println("    /!\\ Encodeur gauche ne compte pas – vérifier câblage");
        step = 0;
        delay(500);
        state = TEST_MOTOR_G_BWD;
      }
      break;

    // ---- Test moteur gauche arrière ----
    case TEST_MOTOR_G_BWD:
      if (step == 0) {
        Serial.println(">>> TEST 5 : Moteur GAUCHE – arrière");
        encG = 0;
        setMotorG(-150);
        tStart = now;
        step = 1;
      }
      if (now - tStart > 1500) {
        motorStop();
        Serial.printf("    Encodeur gauche arrière : %ld ticks\n", encG);
        step = 0;
        delay(500);
        state = TEST_BOTH_FWD;
      }
      break;

    // ---- Test les deux moteurs ensemble ----
    case TEST_BOTH_FWD:
      if (step == 0) {
        Serial.println(">>> TEST 6 : Les deux moteurs – avant simultané");
        encG = 0; encD = 0;
        setMotorD(150);
        setMotorG(150);
        tStart = now;
        step = 1;
      }
      if (now - tStart > 2000) {
        motorStop();
        Serial.printf("    Encodeur G=%ld  D=%ld ticks\n", encG, encD);
        long diff = abs(encG - encD);
        if (diff < 20) Serial.println("    Deux moteurs synchrones OK");
        else           Serial.printf("    /!\\ Écart entre roues = %ld ticks (normal si sol glissant)\n", diff);
        step = 0;
        delay(500);
        state = TEST_ENCODER;
      }
      break;

    // ---- Affichage continu encodeurs (3 s) ----
    case TEST_ENCODER:
      if (step == 0) {
        Serial.println(">>> TEST 7 : Lecture encodeurs en temps réel (3 s) – faites tourner les roues à la main");
        encG = 0; encD = 0;
        tStart = now;
        step = 1;
      }
      if (now - tStart < 3000) {
        static unsigned long tPrint = 0;
        if (now - tPrint > 200) {
          tPrint = now;
          Serial.printf("    G=%5ld  D=%5ld\n", encG, encD);
        }
      } else {
        Serial.println("    Test encodeurs terminé");
        step = 0;
        state = TEST_I2C;
      }
      break;

    // ---- Lecture registre WHO_AM_I des deux capteurs I2C ----
    case TEST_I2C:
      if (step == 0) {
        Serial.println(">>> TEST 8 : Capteurs I2C (WHO_AM_I)");

        // LSM6DS3 – registre 0x0F doit retourner 0x69 ou 0x6A
        Wire.beginTransmission(ADDR_IMU);
        Wire.write(0x0F);
        Wire.endTransmission(false);
        Wire.requestFrom(ADDR_IMU, 1);
        if (Wire.available()) {
          uint8_t id = Wire.read();
          Serial.printf("    IMU LSM6DS3 WHO_AM_I = 0x%02X  %s\n",
                        id, (id == 0x69 || id == 0x6A) ? "OK" : "/!\\ inattendu");
        } else {
          Serial.println("    /!\\ IMU non répondu");
        }

        // LIS3MDL – registre 0x0F doit retourner 0x3D
        Wire.beginTransmission(ADDR_MAG);
        Wire.write(0x0F);
        Wire.endTransmission(false);
        Wire.requestFrom(ADDR_MAG, 1);
        if (Wire.available()) {
          uint8_t id = Wire.read();
          Serial.printf("    MAG LIS3MDL WHO_AM_I = 0x%02X  %s\n",
                        id, (id == 0x3D) ? "OK" : "/!\\ inattendu");
        } else {
          Serial.println("    /!\\ MAG non répondu");
        }

        step = 0;
        state = TEST_DONE;
      }
      break;

    // ---- Bilan ----
    case TEST_DONE:
      if (step == 0) {
        Serial.println("\n========================================");
        Serial.println("   Tous les tests terminés !");
        Serial.println("   Consulte le moniteur série ci-dessus.");
        Serial.println("   Les LEDs clignotent en alternance.");
        Serial.println("========================================\n");
        step = 1;
      }
      // Clignotement continu pour confirmer que le code tourne
      digitalWrite(LEDU1, (millis() / 500) % 2);
      digitalWrite(LEDU2, !((millis() / 500) % 2));
      break;
  }
}
