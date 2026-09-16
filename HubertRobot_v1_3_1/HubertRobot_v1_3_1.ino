/*
  Hubert robot main demo program.

  This sketch controls Hubert's servos via two PCA9685 servo drivers.
  On startup, all servos move to their home (neutral) position.
  The program then waits for serial commands from the Host PC.


  Requirements:
    - Arduino Nano ESP32-S3
    - Two PCA9685 servo drivers (0x40 and 0x41)
    - External 6V power supply for servos
    - Adafruit PWM Servo Driver Library

  created 8 Jun 2026
  by Krister Wolff
  modified 11 Sep 2026
  by Krister Wolff

  This example code is in the public domain.

  http://www.arduino.cc/en/
*/




#include "HubertConfig.h"

// PCA9685 objekt
Adafruit_PWMServoDriver pwm50(PCA9685_ADDR_50HZ);
Adafruit_PWMServoDriver pwm330(PCA9685_ADDR_330HZ);

// Position arrays
int currentPositions50[NUM_SERVOS_50HZ];
int currentPositions330[NUM_SERVOS_330HZ];
int moveDelayMs = DEFAULT_MOVE_DELAY_MS;

bool demoRunning = false;
bool menuDisplayed = false;

// ===================================================
// HJÄLPFUNKTIONER
// ===================================================

// Omvandla hemposition (puls) till vinkel (0-180°)
int homeAngle(const ServoConfig &servo) {
  return map(servo.homePosition, servo.minPulse, servo.maxPulse, 0, 180);
}

// Omvandla puls till vinkel
int angleFromPulse(int pulseUs, const ServoConfig &servo) {
  return map(pulseUs, servo.minPulse, servo.maxPulse, 0, 180);
}

// Omvandla vinkel till puls
int pulseFromAngle(int angle, const ServoConfig &servo) {
  return map(angle, 0, 180, servo.minPulse, servo.maxPulse);
}

// ===================================================
// GRUNDLÄGGANDE SERVO-FUNKTIONER
// ===================================================

void setServo50(int index, int angle) {
  if (index >= NUM_SERVOS_50HZ) return;
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, 
                  SERVOS_50HZ[index].minPulse, 
                  SERVOS_50HZ[index].maxPulse);
  int pwmValue = map(pulse, 0, PERIOD_50HZ, 0, 4096);
  pwm50.setPWM(SERVOS_50HZ[index].channel, 0, pwmValue);
}

void setServo330(int index, int angle) {
  if (index >= NUM_SERVOS_330HZ) return;
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, 
                  SERVOS_330HZ[index].minPulse, 
                  SERVOS_330HZ[index].maxPulse);
  int pwmValue = map(pulse, 0, PERIOD_330HZ, 0, 4096);
  pwm330.setPWM(SERVOS_330HZ[index].channel, 0, pwmValue);
}

// ===================================================
// SYNKRON RÖRELSE – ALLA SERVON SAMTIDIGT
// ===================================================

void syncMoveAll(const int targets50[], const int targets330[]) {
  int maxDistance = 0;
  int distances50[NUM_SERVOS_50HZ];
  int distances330[NUM_SERVOS_330HZ];
  
  // Beräkna maximala avståndet från alla servon
  for (int i = 0; i < NUM_SERVOS_50HZ; i++) {
    int target = constrain(targets50[i], 0, 180);
    distances50[i] = abs(target - currentPositions50[i]);
    if (distances50[i] > maxDistance) maxDistance = distances50[i];
  }
  
  for (int i = 0; i < NUM_SERVOS_330HZ; i++) {
    int target = constrain(targets330[i], 0, 180);
    distances330[i] = abs(target - currentPositions330[i]);
    if (distances330[i] > maxDistance) maxDistance = distances330[i];
  }
  
  if (maxDistance == 0) return;
  
  // Utför rörelsen – ALLA servon uppdateras i varje steg
  for (int step = 0; step <= maxDistance; step++) {
    float progress = (float)step / maxDistance;
    
    // 50 Hz servon
    for (int i = 0; i < NUM_SERVOS_50HZ; i++) {
      if (distances50[i] > 0) {
        int newPos = currentPositions50[i] + (targets50[i] - currentPositions50[i]) * progress;
        setServo50(i, newPos);
      }
    }
    
    // 330 Hz servon
    for (int i = 0; i < NUM_SERVOS_330HZ; i++) {
      if (distances330[i] > 0) {
        int newPos = currentPositions330[i] + (targets330[i] - currentPositions330[i]) * progress;
        setServo330(i, newPos);
      }
    }
    
    delay(moveDelayMs);
  }
  
  // Uppdatera positioner
  for (int i = 0; i < NUM_SERVOS_50HZ; i++) {
    currentPositions50[i] = constrain(targets50[i], 0, 180);
  }
  for (int i = 0; i < NUM_SERVOS_330HZ; i++) {
    currentPositions330[i] = constrain(targets330[i], 0, 180);
  }
}

// ===================================================
// HEMPOSITION
// ===================================================

void goHome() {
  int targets50[NUM_SERVOS_50HZ];
  int targets330[NUM_SERVOS_330HZ];
  
  for (int i = 0; i < NUM_SERVOS_50HZ; i++) {
    targets50[i] = homeAngle(SERVOS_50HZ[i]);
  }
  for (int i = 0; i < NUM_SERVOS_330HZ; i++) {
    targets330[i] = homeAngle(SERVOS_330HZ[i]);
  }
  
  syncMoveAll(targets50, targets330);
}

// ===================================================
// SETUP
// ===================================================

void setup() {
  Serial.begin(115200);
  Serial.println("========================================");
  Serial.println("Hubert Robot v1.1.0");
  Serial.println("Individual Servo Limits with Home Position");
  Serial.println("========================================");
  
  // Skriv ut konfiguration för 50 Hz servon
  Serial.println("\n--- 50 Hz Servos (0x40) ---");
  for (int i = 0; i < NUM_SERVOS_50HZ; i++) {
    Serial.print(SERVOS_50HZ[i].name);
    Serial.print(": ch=");
    Serial.print(SERVOS_50HZ[i].channel);
    Serial.print(" min=");
    Serial.print(SERVOS_50HZ[i].minPulse);
    Serial.print(" max=");
    Serial.print(SERVOS_50HZ[i].maxPulse);
    Serial.print(" home=");
    Serial.println(SERVOS_50HZ[i].homePosition);
  }
  
  // Skriv ut konfiguration för 330 Hz servon
  Serial.println("\n--- 330 Hz Servos (0x41) ---");
  for (int i = 0; i < NUM_SERVOS_330HZ; i++) {
    Serial.print(SERVOS_330HZ[i].name);
    Serial.print(": ch=");
    Serial.print(SERVOS_330HZ[i].channel);
    Serial.print(" min=");
    Serial.print(SERVOS_330HZ[i].minPulse);
    Serial.print(" max=");
    Serial.print(SERVOS_330HZ[i].maxPulse);
    Serial.print(" home=");
    Serial.println(SERVOS_330HZ[i].homePosition);
  }
  
  Serial.println("\nInitierar PCA9685 moduler...");
  
  // Initiera 50 Hz modul
  pwm50.begin();
  pwm50.setPWMFreq(50);
  
  // Initiera 330 Hz modul
  pwm330.begin();
  pwm330.setPWMFreq(330);
  
  delay(100);
  
  // Sätt alla servon till hemposition
  Serial.println("Sätter alla servon till hemposition...");
  for (int i = 0; i < NUM_SERVOS_50HZ; i++) {
    currentPositions50[i] = homeAngle(SERVOS_50HZ[i]);
    setServo50(i, currentPositions50[i]);
  }
  
  for (int i = 0; i < NUM_SERVOS_330HZ; i++) {
    currentPositions330[i] = homeAngle(SERVOS_330HZ[i]);
    setServo330(i, currentPositions330[i]);
  }
  
  delay(500);
  Serial.println("\nReady! All servos at home position.");
  Serial.println("========================================\n");
  delay(1000);
  Serial.println("Type '?' for commands");
}

void showMenu() {
  Serial.println("\n--- Commands ---");
  Serial.println("  h  -> HOME position");
  Serial.println("  0  -> All servos to 0°");
  Serial.println("  1  -> All servos to 180°");
  Serial.println("  ?  -> Show this menu");
  Serial.print("> ");
}

// ===================================================
// HUVUDLOOP – VÄNTAR PÅ ANVÄNDARKOMMANDON
// ===================================================

// bool demoRunning = false;  // Flagga för om demo-loopen körs

// Demo-sekvens (ett varv)
void runDemoSequence() {
  int targets50[NUM_SERVOS_50HZ];
  int targets330[NUM_SERVOS_330HZ];
  
  Serial.println("  -> HOME position");
  goHome();
  delay(2000);
  
  Serial.println("  -> All servos to 0°");
  for (int i = 0; i < NUM_SERVOS_50HZ; i++) targets50[i] = 0;
  for (int i = 0; i < NUM_SERVOS_330HZ; i++) targets330[i] = 0;
  syncMoveAll(targets50, targets330);
  delay(2000);
  
  Serial.println("  -> All servos to 180°");
  for (int i = 0; i < NUM_SERVOS_50HZ; i++) targets50[i] = 180;
  for (int i = 0; i < NUM_SERVOS_330HZ; i++) targets330[i] = 180;
  syncMoveAll(targets50, targets330);
  delay(2000);
  
  Serial.println("  -> Back to HOME position");
  goHome();
}

void loop() {
  // Om demo-loopen är aktiv, kör ett varv
  if (demoRunning) {
    runDemoSequence();
  }
  
  // Visa meny (men bara om demo-loopen INTE körs)
  if (!demoRunning && !menuDisplayed) {
    Serial.println("\n========================================");
    Serial.println("Hubert Robot Control");
    Serial.println("========================================");
    Serial.println("Commands:");
    Serial.println("  [h] HOME position");
    Serial.println("  [0] All servos to 0°");
    Serial.println("  [1] All servos to 180°");
    Serial.println("  [d] Run DEMO loop");
    Serial.println("  [s] STOP demo");
    Serial.println("  [?] Show this menu");
    Serial.print("\nEnter command: ");
    menuDisplayed = true;
  }
  
  // Visa kort indikation att demo körs
  if (demoRunning) {
    static unsigned long lastDot = 0;
    if (millis() - lastDot > 1000) {
      Serial.print(".");
      lastDot = millis();
    }
  }
  
  // Kolla efter användarinput
  if (Serial.available()) {
    char cmd = Serial.read();
    
    // Rensa input-bufferten
    while (Serial.available()) {
      Serial.read();
    }
    
    Serial.println(); // Ny rad efter kommando
    menuDisplayed = false;  // Nästa gång demo stoppas, visa menyn igen
    
    int targets50[NUM_SERVOS_50HZ];
    int targets330[NUM_SERVOS_330HZ];
    
    switch (cmd) {
      case 'h':
      case 'H':
        demoRunning = false;
        Serial.println("Moving: All servos to HOME position");
        goHome();
        break;
        
      case '0':
        demoRunning = false;
        Serial.println("Moving: All servos to 0°");
        for (int i = 0; i < NUM_SERVOS_50HZ; i++) targets50[i] = 0;
        for (int i = 0; i < NUM_SERVOS_330HZ; i++) targets330[i] = 0;
        syncMoveAll(targets50, targets330);
        Serial.println("Done.");
        break;
        
      case '1':
        demoRunning = false;
        Serial.println("Moving: All servos to 180°");
        for (int i = 0; i < NUM_SERVOS_50HZ; i++) targets50[i] = 180;
        for (int i = 0; i < NUM_SERVOS_330HZ; i++) targets330[i] = 180;
        syncMoveAll(targets50, targets330);
        Serial.println("Done.");
        break;
        
      case 'd':
      case 'D':
        if (demoRunning) {
          Serial.println("Demo already running.");
        } else {
          Serial.println("Starting DEMO loop...");
          Serial.println("(Type 's' to stop)");
          demoRunning = true;
        }
        break;
        
      case 's':
      case 'S':
        if (demoRunning) {
          Serial.println("Stopping DEMO loop.");
          demoRunning = false;
        } else {
          Serial.println("No demo running. Type 'd' to start demo.");
        }
        break;
        
      case '?':
        // Menyn visas nästa loop (menuDisplayed = false redan)
        break;
        
      default:
        Serial.print("Unknown command: ");
        Serial.println(cmd);
        break;
    }
  }
  
  delay(50);
}
