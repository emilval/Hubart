#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9685 Driver Objects
Adafruit_PWMServoDriver pwm50(0x40);   // 50 Hz Board
Adafruit_PWMServoDriver pwm330(0x41);  // 330 Hz Board

const int PERIOD_50HZ = 20000;
const int PERIOD_330HZ = 3030;
const int STEP_DELAY_MS = 10;
const int HOME_OFFSET = 90;
const int NUM_JOINTS = 6;
const float STEP_DEG = 0.5;



struct Joint {
  const char* name;
  Adafruit_PWMServoDriver* driver;
  int channel;
  int periodUs;
  int minPulseUs;
  int maxPulseUs;
  float currentAngle; // Physical 0 - 180 degrees
  float targetAngle;  // Physical 0 - 180 degrees
};

Joint joints[NUM_JOINTS] = {
  {"base_joint",       &pwm330, 0, PERIOD_330HZ, 630,  2690, 90, 90},
  {"shoulder_joint",   &pwm330, 1, PERIOD_330HZ, 930,  2910, 90, 90},
  {"elbow_joint",      &pwm50,  2, PERIOD_50HZ,  450,  2480, 90, 90},
  {"neck_joint",       &pwm50,  0, PERIOD_50HZ,  770,  2270, 90, 90},
  {"head_turn_joint",  &pwm50,  1, PERIOD_50HZ,  460,  2440, 90, 90},
  {"gripper_joint",    &pwm50,  4, PERIOD_50HZ,  780,  1820, 90, 90}
};

bool targetInProgress = false;

void setJointAngle(Joint &j, float angleDeg) {
  // Constrain between 0.0 and 180.0
  if (angleDeg < 0.0) angleDeg = 0.0;
  if (angleDeg > 180.0) angleDeg = 180.0;

  float centerPulseUs = (j.minPulseUs + j.maxPulseUs) / 2.0;
  float pulseUs;
  
  // Float equivalent of your piecewise map() logic
  if (angleDeg <= 90.0) {
    pulseUs = j.minPulseUs + (angleDeg / 90.0) * (centerPulseUs - j.minPulseUs);
  } else {
    pulseUs = centerPulseUs + ((angleDeg - 90.0) / 90.0) * (j.maxPulseUs - centerPulseUs);
  }

  // Convert pulse microseconds to the 0-4095 PCA9685 scale
  int pwmValue = (int)((pulseUs / (float)j.periodUs) * 4096.0);
  j.driver->setPWM(j.channel, 0, pwmValue);
}

void printStatus() {
  Serial.println("\n--- Current Joint Positions ---");
  for (int i = 0; i < NUM_JOINTS; i++) {
    int userAngle = joints[i].targetAngle - HOME_OFFSET;
    Serial.print("  [");
    Serial.print(i);
    Serial.print("] ");
    Serial.print(joints[i].name);
    Serial.print(": ");
    Serial.print(userAngle);
    Serial.println(" deg");
  }
  Serial.println("--------------------------------");
}

void processSerialInput(String input) {
  input.trim();
  if (input.length() == 0) return;

  // Option A: Single joint command (e.g. "j0:15" or "j2:-30")
  if (input.startsWith("j") || input.startsWith("J")) {
    int colonIdx = input.indexOf(':');
    if (colonIdx > 1) {
      int jointIdx = input.substring(1, colonIdx).toInt();
      int angleVal = input.substring(colonIdx + 1).toInt();
      if (jointIdx >= 0 && jointIdx < NUM_JOINTS) {
        angleVal = constrain(angleVal, -90, 90);
        joints[jointIdx].targetAngle = angleVal + HOME_OFFSET;
        Serial.print("Updated ");
        Serial.print(joints[jointIdx].name);
        Serial.print(" target to ");
        Serial.print(angleVal);
        Serial.println(" deg");
      }
    }
    return;
  }

  // Option B: Query status command
  if (input == "?" || input == "status") {
    printStatus();
    return;
  }

  // Option C: 6 comma- or space-separated values (e.g., "0, -20, 45, 0, 0, 10")
  int parsedCount = 0;
  int startIdx = 0;

  for (int i = 0; i <= input.length(); i++) {
    if (i == input.length() || input.charAt(i) == ',' || input.charAt(i) == ' ') {
      if (i > startIdx) {
        String token = input.substring(startIdx, i);
        token.trim();
        if (token.length() > 0 && parsedCount < NUM_JOINTS) {
          int userAngle = constrain(token.toInt(), -90, 90);
          joints[parsedCount].targetAngle = userAngle + HOME_OFFSET;
          parsedCount++;
        }
      }
      startIdx = i + 1;
    }
  }

  if (parsedCount == NUM_JOINTS) {
    targetInProgress = true;
    Serial.println("Successfully updated all 6 joint target angles.");
  } else if (parsedCount > 0) {
    Serial.print("Updated ");
    Serial.print(parsedCount);
    Serial.println(" joint(s). Expected 6 values.");
  } else {
    Serial.println("Invalid input format. Use: '0, -30, 45, 0, 10, -10' or 'j0:15'");
  }
}
void moveJointsSimultaneously() {

  for (int i = 0; i < NUM_JOINTS; i++) {
    // Check the distance to the target
    float distance = joints[i].targetAngle - joints[i].currentAngle;
    
    // If we are closer than one step size, just snap perfectly to the target
    if (abs(distance) <= STEP_DEG) {
      if (joints[i].currentAngle != joints[i].targetAngle) {
        joints[i].currentAngle = joints[i].targetAngle;
        setJointAngle(joints[i], joints[i].currentAngle);
      }
    } 
    // Otherwise, step toward the target by the fraction
    else if (distance > 0) {
      joints[i].currentAngle += STEP_DEG;
      setJointAngle(joints[i], joints[i].currentAngle);
    } else {
      joints[i].currentAngle -= STEP_DEG;
      setJointAngle(joints[i], joints[i].currentAngle);
    }
  }

  // Check if all reached
  if (targetInProgress) {
    bool targetReached = true;
    for (int i = 0; i < NUM_JOINTS; i++) {
      // Use a small threshold (0.01) instead of exact != for floats
      if (abs(joints[i].currentAngle - joints[i].targetAngle) > 0.005) {
        targetReached = false;
        break;
      }
    }

    if (targetReached) {
      targetInProgress = false;
      Serial.println("TARGET_REACHED");
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize hardware PWM drivers
  pwm50.begin();
  pwm50.setPWMFreq(50);
  pwm330.begin();
  pwm330.setPWMFreq(330);
  delay(100);

  // Move joints to home position initially
  for (int i = 0; i < NUM_JOINTS; i++) {
    setJointAngle(joints[i], joints[i].currentAngle);
  }

  Serial.println("=========================================");
  Serial.println(" HUBERT 6-DOF Manual Serial Angle Control");
  Serial.println("=========================================");
  Serial.println("Commands:");
  Serial.println("  1. Enter 6 angles (-90 to +90): e.g. '0, -30, 45, 0, 10, -10'");
  Serial.println("  2. Single joint update: e.g. 'j0:15' or 'j2:-45'");
  Serial.println("  3. Type '?' to view current joint angles");
  Serial.println("=========================================");
}

void loop() {
  // Read Serial commands from user
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    processSerialInput(input);
  }

  // Smoothly step servos toward target position
  moveJointsSimultaneously();

  delay(STEP_DELAY_MS);
}
