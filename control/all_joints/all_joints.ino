#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9685 Driver Objects
Adafruit_PWMServoDriver pwm50(0x40);   // 50 Hz Board
Adafruit_PWMServoDriver pwm330(0x41);  // 330 Hz Board

const int PERIOD_50HZ = 20000;
const int PERIOD_330HZ = 3030;
const int STEP_DELAY_MS = 10;
const int HOME_OFFSET = 90;
const int NUM_JOINTS = 5;
const float STEP_PULSE_US = 5.0; // The max microseconds to move per 10ms loop. Adjust for speed!

struct Joint {
  const char* name;
  Adafruit_PWMServoDriver* driver;
  int channel;
  int periodUs;
  int minPulseUs;
  int maxPulseUs;
  float currentAngle;    // Physical 0 - 180 degrees (used for serial status)
  float targetAngle;     // Physical 0 - 180 degrees (used for serial target)
  float currentPulseUs;  // HIGH-RES: Current physical position
  float targetPulseUs;   // HIGH-RES: Target physical position
};

Joint joints[NUM_JOINTS] = {
  {"base_joint",       &pwm330, 0, PERIOD_330HZ, 710,  2670, 90, 90},
  {"shoulder_joint",   &pwm330, 1, PERIOD_330HZ, 930,  2910, 90, 90},
  {"elbow_joint",      &pwm50,  2, PERIOD_50HZ,  450,  2480, 90, 90},
  {"neck_joint",       &pwm50,  0, PERIOD_50HZ,  770,  2270, 90, 90},
  {"head_turn_joint",  &pwm50,  1, PERIOD_50HZ,  460,  2440, 90, 90}
};

bool targetInProgress = false;
float angleToPulse(const Joint &j, float angleDeg);
void moveJointsSimultaneously();

void setTargetAnglesFromArray(const float targetAngles[NUM_JOINTS]) {
  for (int i = 0; i < NUM_JOINTS; i++) {
    float userAngle = constrain(targetAngles[i], -90.0, 90.0);
    joints[i].targetAngle = userAngle + HOME_OFFSET;
    joints[i].targetPulseUs = angleToPulse(joints[i], joints[i].targetAngle);
  }
  targetInProgress = true;
}

void processBufferedPointSet(String payload) {
  int pointStart = 0;
  int processedCount = 0;

  while (pointStart <= payload.length()) {
    int pointEnd = payload.indexOf(';', pointStart);
    if (pointEnd < 0) pointEnd = payload.length();

    String pointToken = payload.substring(pointStart, pointEnd);
    pointToken.trim();

    if (pointToken.length() > 0) {
      float pointAngles[NUM_JOINTS] = {0.0, 0.0, 0.0, 0.0, 0.0};
      int parsedCount = 0;
      int valueStart = 0;

      for (int i = 0; i <= pointToken.length(); i++) {
        if (i == pointToken.length() || pointToken.charAt(i) == ',') {
          if (i > valueStart) {
            String token = pointToken.substring(valueStart, i);
            token.trim();
            if (token.length() > 0 && parsedCount < NUM_JOINTS) {
              pointAngles[parsedCount++] = token.toFloat();
            }
          }
          valueStart = i + 1;
        }
      }

      if (parsedCount == NUM_JOINTS) {
        setTargetAnglesFromArray(pointAngles);
        while (targetInProgress) {
          moveJointsSimultaneously();
          delay(STEP_DELAY_MS);
        }
        processedCount++;
      } else {
        Serial.print("Skipped malformed buffered point: ");
        Serial.println(pointToken);
      }
    }

    if (pointEnd >= payload.length()) break;
    pointStart = pointEnd + 1;
  }

  Serial.print("BUFFER_DONE: ");
  Serial.print(processedCount);
  Serial.println(" points processed");
}

// 1. Convert user degrees to precise microseconds
float angleToPulse(const Joint &j, float angleDeg) {
  float constrainedAngle = constrain(angleDeg, 0.0, 180.0);
  float centerPulseUs = (j.minPulseUs + j.maxPulseUs) / 2.0;
  
  if (constrainedAngle <= 90.0) {
    return j.minPulseUs + (constrainedAngle / 90.0) * (centerPulseUs - j.minPulseUs);
  } else {
    return centerPulseUs + ((constrainedAngle - 90.0) / 90.0) * (j.maxPulseUs - centerPulseUs);
  }
}

// 2. Convert microseconds back to degrees (for the ? status command)
float pulseToAngle(const Joint &j, float pulseUs) {
  float centerPulseUs = (j.minPulseUs + j.maxPulseUs) / 2.0;
  if (pulseUs <= centerPulseUs) {
     return 90.0 * (pulseUs - j.minPulseUs) / (centerPulseUs - j.minPulseUs);
  } else {
     return 90.0 + 90.0 * (pulseUs - centerPulseUs) / (j.maxPulseUs - centerPulseUs);
  }
}

// 3. Send the exact pulse to the PCA9685 driver
void applyPulse(const Joint &j, float pulseUs) {
  int pwmValue = (int)((pulseUs / (float)j.periodUs) * 4096.0);
  j.driver->setPWM(j.channel, 0, pwmValue);
}

void setJointAngle(Joint &j, float angleDeg) {
  // Constrain between 0.0 and 180.0
  if (angleDeg < 0.0) angleDeg = 0.0;
  if (angleDeg > 180.0) angleDeg = 180.0;

  float centerPulseUs = (j.minPulseUs + j.maxPulseUs) / 2.0;
  float pulseUs;
  
  // Piecewise mapping relative to home center
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
    int userAngle = (int)round(joints[i].currentAngle - HOME_OFFSET);
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

  // Option A: Buffered point list command (e.g. "B:0,-30,45,0,10,-10;1,-20,40,0,0,0")
  if (input.startsWith("B:") || input.startsWith("P:")) {
    String payload = input.substring(2);
    payload.trim();
    if (payload.length() > 0) {
      processBufferedPointSet(payload);
    } else {
      Serial.println("Empty point buffer.");
    }
    return;
  }

  // Option B: Single joint command (e.g. "j0:15" or "j2:-30")
  if (input.startsWith("j") || input.startsWith("J")) {
    int colonIdx = input.indexOf(':');
    if (colonIdx > 1) {
      int jointIdx = input.substring(1, colonIdx).toInt();
      int angleVal = input.substring(colonIdx + 1).toInt();
      if (jointIdx >= 0 && jointIdx < NUM_JOINTS) {
        angleVal = constrain(angleVal, -90, 90);
        joints[jointIdx].targetAngle = angleVal + HOME_OFFSET;
        
        // ADDED HERE: Calculate the target pulse for the single joint
        joints[jointIdx].targetPulseUs = angleToPulse(joints[jointIdx], joints[jointIdx].targetAngle); 
        
        targetInProgress = true;
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

  // Option C: 5 comma- or space-separated values (e.g., "0, -20, 45, 0, 10")
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
          
          // ADDED HERE: Calculate the target pulse using parsedCount
          joints[parsedCount].targetPulseUs = angleToPulse(joints[parsedCount], joints[parsedCount].targetAngle); 
          
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
    Serial.println(" joint(s). Expected 5 values.");
  } else {
    Serial.println("Invalid input format. Use: '0, -30, 45, 0, 10' or 'j0:15'");
  }
}

void moveJointsSimultaneously() {
  if (!targetInProgress) return;

  float maxDistanceUs = 0.0;
  float distancesUs[NUM_JOINTS];

  // 1. Calculate required high-res displacement
  for (int i = 0; i < NUM_JOINTS; i++) {
    distancesUs[i] = joints[i].targetPulseUs - joints[i].currentPulseUs;
    float absDist = abs(distancesUs[i]);
    if (absDist > maxDistanceUs) {
      maxDistanceUs = absDist;
    }
  }

  // 2. Check if movement is practically complete (within half a microsecond)
  if (maxDistanceUs <= 0.5) {
    for (int i = 0; i < NUM_JOINTS; i++) {
      joints[i].currentPulseUs = joints[i].targetPulseUs;
      joints[i].currentAngle = joints[i].targetAngle; // sync the display variable
      applyPulse(joints[i], joints[i].currentPulseUs);
    }
    targetInProgress = false;
    Serial.println("TARGET_REACHED");
    return;
  }

  // 3. Step each joint scaled proportionally in microsecond space
  for (int i = 0; i < NUM_JOINTS; i++) {
    if (abs(distancesUs[i]) <= 0.5) continue;

    // Scale step size so every joint takes the same time to finish
    float stepSizeUs = STEP_PULSE_US * (abs(distancesUs[i]) / maxDistanceUs);

    // Apply the micro-step
    if (abs(distancesUs[i]) <= stepSizeUs) {
      joints[i].currentPulseUs = joints[i].targetPulseUs;
    } else if (distancesUs[i] > 0) {
      joints[i].currentPulseUs += stepSizeUs;
    } else {
      joints[i].currentPulseUs -= stepSizeUs;
    }

    applyPulse(joints[i], joints[i].currentPulseUs);
    
    // Sync the readable angle for the status command mid-move
    joints[i].currentAngle = pulseToAngle(joints[i], joints[i].currentPulseUs);
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
    joints[i].currentPulseUs = angleToPulse(joints[i], joints[i].currentAngle);
    joints[i].targetPulseUs = joints[i].currentPulseUs;
    applyPulse(joints[i], joints[i].currentPulseUs);
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

  // Smoothly step servos toward target position synchronously
  moveJointsSimultaneously();

  delay(STEP_DELAY_MS);
}