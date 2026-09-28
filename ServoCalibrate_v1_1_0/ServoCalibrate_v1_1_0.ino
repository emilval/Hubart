/*
  Hubert robot servo calibration program.

  This sketch is used to calibrate individual servos.
  It allows you to find the min/max pulse values for each servo
  by adjusting the pulse width manually and observing the movement.


  created 01 Jul 2026
  by Krister Wolff
  modified 11 Sep 2026
  by Krister Wolff

  This example code is in the public domain.

  http://www.arduino.cc/en/
*/



#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Välj vilken modul du vill testa (ändra här)
//#define TEST_MODULE_50HZ
#define TEST_MODULE_330HZ

// Välj servo-kanal (0-15)
int SERVO_CHANNEL = 1;

// PCA9685 adresser
const int ADDR_50HZ = 0x40;
const int ADDR_330HZ = 0x41;

const int SWEEP_DELAY_MS = 50;  // 50 ms = lagom, öka för långsammare

// Skapa PWM-objekt
#ifdef TEST_MODULE_50HZ
  Adafruit_PWMServoDriver pwm(ADDR_50HZ);
  const int FREQ = 50;
  const int PERIOD_US = 20000;
  const char* MODULE_NAME = "50 Hz (0x40)";
#else
  Adafruit_PWMServoDriver pwm(ADDR_330HZ);
  const int FREQ = 330;
  const int PERIOD_US = 3030;
  const char* MODULE_NAME = "330 Hz (0x41)";
#endif

// Testparametrar
int minPulse = 550;      // Börja försiktigt
int maxPulse = 2400;     // Börja försiktigt
int currentPulse = 1500; // Mittläge

void setup() {
  Serial.begin(115200);
  Serial.println("========================================");
  Serial.print("Servo Kalibrering - ");
  Serial.println(MODULE_NAME);
  Serial.print("Kanal: ");
  Serial.println(SERVO_CHANNEL);
  Serial.println("========================================");
  Serial.println();
  Serial.println("Kommandon:");
  Serial.println("  +     - öka puls med 10 µs");
  Serial.println("  -     - minska puls med 10 µs");
  Serial.println("  ++    - öka puls med 100 µs");
  Serial.println("  --    - minska puls med 100 µs");
  Serial.println("  mXXX  - sätt minPulse (t.ex. m550)");
  Serial.println("  MXXX  - sätt maxPulse (t.ex. M2440)");
  Serial.println("  s     - sweep mellan min/max");
  Serial.println("  c     - centrera (1500 µs)");
  Serial.println("  ?     - visa aktuella värden");
  Serial.println("  <     - föregående kanal");
  Serial.println("  >     - nästa kanal");
  Serial.println();
  
  pwm.begin();
  pwm.setPWMFreq(FREQ);
  delay(100);
  
  // Starta i mittläge
  setPulse(currentPulse);
  Serial.print("Startar med puls: ");
  Serial.print(currentPulse);
  Serial.println(" µs");
  showStatus();
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd == "+") {
      currentPulse += 10;
      setPulse(currentPulse);
      Serial.print("Puls: ");
      Serial.println(currentPulse);
    }
    else if (cmd == "-") {
      currentPulse -= 10;
      setPulse(currentPulse);
      Serial.print("Puls: ");
      Serial.println(currentPulse);
    }
    else if (cmd == "++") {
      currentPulse += 100;
      setPulse(currentPulse);
      Serial.print("Puls: ");
      Serial.println(currentPulse);
    }
    else if (cmd == "--") {
      currentPulse -= 100;
      setPulse(currentPulse);
      Serial.print("Puls: ");
      Serial.println(currentPulse);
    }
    else if (cmd.startsWith("m")) {
      minPulse = cmd.substring(1).toInt();
      Serial.print("MinPulse satt till: ");
      Serial.println(minPulse);
      showStatus();
    }
    else if (cmd.startsWith("M")) {
      maxPulse = cmd.substring(1).toInt();
      Serial.print("MaxPulse satt till: ");
      Serial.println(maxPulse);
      showStatus();
    }
    else if (cmd == "c") {
      Serial.println("Centrerar långsamt till 1500 µs...");
      slowMove(currentPulse, 1500);
      currentPulse = 1500;
      Serial.println("Centrerad till 1500 µs");
    }
    else if (cmd == "<") {
      if (SERVO_CHANNEL > 0) {
        SERVO_CHANNEL--;
      }
      Serial.print("Bytt till kanal: ");
      Serial.println(SERVO_CHANNEL);
      setPulse(currentPulse); // Move the new servo to the current position
    }
    else if (cmd == ">") {
      if (SERVO_CHANNEL < 15) {
        SERVO_CHANNEL++;
      }
      Serial.print("Bytt till kanal: ");
      Serial.println(SERVO_CHANNEL);
      setPulse(currentPulse);
    }
  /*  else if (cmd == "c") {
      currentPulse = 1500;
      setPulse(currentPulse);
      Serial.println("Centrerad till 1500 µs");
    }
  */
    else if (cmd == "s") {
      sweep();
    }
    else if (cmd == "?") {
      showStatus();
    }
  }
}

void setPulse(int pulseUs) {
  pulseUs = constrain(pulseUs, 300, 2700);
  int pwmValue = map(pulseUs, 0, PERIOD_US, 0, 4096);
  pwm.setPWM(SERVO_CHANNEL, 0, pwmValue);
}

void slowMove(int fromPulse, int toPulse) {
  if (fromPulse < toPulse) {
    for (int p = fromPulse; p <= toPulse; p += 20) {
      setPulse(p);
      Serial.print("Puls: ");
      Serial.println(p);
      delay(SWEEP_DELAY_MS);
    }
  } else {
    for (int p = fromPulse; p >= toPulse; p -= 20) {
      setPulse(p);
      Serial.print("Puls: ");
      Serial.println(p);
      delay(SWEEP_DELAY_MS);
    }
  }
}

void sweep() {
  Serial.println("Startar sweep...");
  
  slowMove(currentPulse, minPulse);      // 1. Till min
  Serial.println("Vid minPulse");
  delay(500);
  
  slowMove(minPulse, maxPulse);          // 2. Till max
  Serial.println("Vid maxPulse");
  delay(500);
  
  slowMove(maxPulse, 1500);              // 3. Tillbaka till mitt
  
  currentPulse = 1500;
  Serial.println("Sweep klar, återgick till 1500 µs");
}

void showStatus() {
  Serial.println("----------------------------------------");
  Serial.print("MinPulse: ");
  Serial.print(minPulse);
  Serial.print(" µs (0°)");
  if (currentPulse == minPulse) Serial.print(" ← NU");
  Serial.println();
  
  Serial.print("MaxPulse: ");
  Serial.print(maxPulse);
  Serial.print(" µs (180°)");
  if (currentPulse == maxPulse) Serial.print(" ← NU");
  Serial.println();
  
  Serial.print("Current: ");
  Serial.print(currentPulse);
  Serial.print(" µs → ");
  int angle = map(currentPulse, minPulse, maxPulse, 0, 180);
  Serial.print(angle);
  Serial.println("°");
  Serial.println("----------------------------------------");
}
