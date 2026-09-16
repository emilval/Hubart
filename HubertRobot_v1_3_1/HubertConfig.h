/*
  Hubert robot configuration file.

  This file contains all servo definitions and hardware settings
  for the Hubert robot, including:
    - Servo channel assignments
    - Min/max pulse values
    - Home (neutral) positions
    - PCA9685 I2C addresses

  Two PCA9685 boards are used:
    - 0x40: 50 Hz servos (HeadPan, HeadTilt, Elbow, WristRot, Gripper)
    - 0x41: 330 Hz servos (BaseRotate, Shoulder)

  created 8 Jun 2026
  by Krister Wolff
  modified 11 Sep 2026
  by Krister Wolff

  This example code is in the public domain.

  http://www.arduino.cc/en/
*/



#ifndef HUBERT_CONFIG_H
#define HUBERT_CONFIG_H

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ===================================================
// SERVO KONFIGURATION – INDIVIDUELLA VÄRDEN
// ===================================================

struct ServoConfig {
  int channel;
  int minPulse;
  int maxPulse;
  int homePosition;     // Hemposition (neutral puls, startläge)
  const char* name;
};

// Modul 1: 50 Hz (0x40)
const int NUM_SERVOS_50HZ = 5;
const ServoConfig SERVOS_50HZ[] = {
  {0, 770,  2270, 1630, "HeadPan"},
  {1, 460, 2440, 1820, "HeadTilt"},
  {2, 450,  2480, 2010, "Elbow"},
  {3, 550,  2440, 1500, "WristRot"}, // no wrist rotation
  {4, 780,  1820, 605,  "Gripper"}
};

// Modul 2: 330 Hz (0x41)
const int NUM_SERVOS_330HZ = 2;
const ServoConfig SERVOS_330HZ[] = {
  {0, 610, 2670, 1530, "BaseRotate"},
  {1, 630, 2690, 890,  "Shoulder"}
};

// Gemensamma inställningar
const int PERIOD_50HZ = 20000;      // 1/50 Hz = 20000 µs
const int PERIOD_330HZ = 3030;      // 1/330 Hz ≈ 3030 µs
const int DEFAULT_MOVE_DELAY_MS = 20;  // ms per grad

// PCA9685 adresser
const int PCA9685_ADDR_50HZ = 0x40;
const int PCA9685_ADDR_330HZ = 0x41;

#endif
