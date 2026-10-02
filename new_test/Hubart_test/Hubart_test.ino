/*
  Hubert Robot - Serial Drawing Controller
  
  This sketch integrates a non-blocking serial state machine 
  with the official dual PCA9685 driver framework.
  It listens for incoming angle coordinates in the format:
  <theta1,theta2,theta3> and uses linear interpolation 
  (syncMoveAll) to move the drawing joints smoothly.
*/

#include "HubertConfig.h"

// Initialize the dual PCA9685 driver objects
Adafruit_PWMServoDriver pwm50(PCA9685_ADDR_50HZ);
Adafruit_PWMServoDriver pwm330(PCA9685_ADDR_330HZ);

// Arrays to store current positions of all servos
int currentPositions50[NUM_SERVOS_50HZ];
int currentPositions330[NUM_SERVOS_330HZ];
int moveDelayMs = DEFAULT_MOVE_DELAY_MS;

// Static memory allocation for serial buffer (prevents heap fragmentation)
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
boolean newData = false;

// ===================================================
// HELPER FUNCTIONS (From Official Demo)
// ===================================================

int homeAngle(const ServoConfig &servo) {
    return map(servo.homePosition, servo.minPulse, servo.maxPulse, 0, 180);
}

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

// Linear interpolation function for smooth joint coordination
void syncMoveAll(const int targets50[], const int targets330[]) {
    int maxDistance = 0;
    int distances50[NUM_SERVOS_50HZ];
    int distances330[NUM_SERVOS_330HZ];
    
    // Calculate the maximum angular displacement required
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
    
    // Execute synchronous movement
    for (int step = 0; step <= maxDistance; step++) {
        float progress = (float)step / maxDistance;
        
        for (int i = 0; i < NUM_SERVOS_50HZ; i++) {
            if (distances50[i] > 0) {
                int newPos = currentPositions50[i] + (targets50[i] - currentPositions50[i]) * progress;
                setServo50(i, newPos);
            }
        }
        
        for (int i = 0; i < NUM_SERVOS_330HZ; i++) {
            if (distances330[i] > 0) {
                int newPos = currentPositions330[i] + (targets330[i] - currentPositions330[i]) * progress;
                setServo330(i, newPos);
            }
        }
        delay(moveDelayMs);
    }
    
    // Update current positions post-movement
    for (int i = 0; i < NUM_SERVOS_50HZ; i++) currentPositions50[i] = constrain(targets50[i], 0, 180);
    for (int i = 0; i < NUM_SERVOS_330HZ; i++) currentPositions330[i] = constrain(targets330[i], 0, 180);
}

void goHome() {
    int targets50[NUM_SERVOS_50HZ];
    int targets330[NUM_SERVOS_330HZ];
    
    for (int i = 0; i < NUM_SERVOS_50HZ; i++) targets50[i] = homeAngle(SERVOS_50HZ[i]);
    for (int i = 0; i < NUM_SERVOS_330HZ; i++) targets330[i] = homeAngle(SERVOS_330HZ[i]);
    
    syncMoveAll(targets50, targets330);
}

// ===================================================
// SETUP ROUTINE
// ===================================================

void setup() {
    Serial.begin(115200);
    Serial.println("Initializing Hubert Drawing Controller...");
    
    pwm50.begin();
    pwm50.setPWMFreq(50);
    
    pwm330.begin();
    pwm330.setPWMFreq(330);
    
    delay(100);
    
    // Calibrate current positions and move to home safely
    for (int i = 0; i < NUM_SERVOS_50HZ; i++) {
        currentPositions50[i] = homeAngle(SERVOS_50HZ[i]);
        setServo50(i, currentPositions50[i]);
    }
    
    for (int i = 0; i < NUM_SERVOS_330HZ; i++) {
        currentPositions330[i] = homeAngle(SERVOS_330HZ[i]);
        setServo330(i, currentPositions330[i]);
    }
    
    delay(500);
    Serial.println("System Ready. Send format: <theta1,theta2,theta3>");
}

// ===================================================
// MAIN LOOP & STATE MACHINE
// ===================================================

void loop() {
    // Non-blocking serial listener
    recvWithStartEndMarkers();
    
    // Trigger action when a complete data packet arrives
    if (newData == true) {
        // Copy string safely to prevent strtok from altering the original buffer
        strcpy(tempChars, receivedChars);
        parseAndExecute();
        newData = false; 
    }
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1; // Prevent buffer overflow
                }
            } else {
                receivedChars[ndx] = '\0'; // Terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        } else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void parseAndExecute() {
    char * strtokIndx; 
    float t1, t2, t3;
    
    // Parse theta 1
    strtokIndx = strtok(tempChars, ","); 
    if(strtokIndx == NULL) return; 
    t1 = atof(strtokIndx);     
    
    // Parse theta 2
    strtokIndx = strtok(NULL, ","); 
    if(strtokIndx == NULL) return;
    t2 = atof(strtokIndx); 
    
    // Parse theta 3
    strtokIndx = strtok(NULL, ",");
    if(strtokIndx == NULL) return;
    t3 = atof(strtokIndx);
    
    // Map parsed angles to the global target arrays
    int target_50Hz[NUM_SERVOS_50HZ];
    int target_330Hz[NUM_SERVOS_330HZ];
    
    // By default, hold current positions for non-drawing servos
    for (int i = 0; i < NUM_SERVOS_50HZ; i++) target_50Hz[i] = currentPositions50[i];
    for (int i = 0; i < NUM_SERVOS_330HZ; i++) target_330Hz[i] = currentPositions330[i];
    
  
    int base_home     = homeAngle(SERVOS_330HZ[0]);
    int shoulder_home = homeAngle(SERVOS_330HZ[1]);
    int elbow_home    = homeAngle(SERVOS_50HZ[2]);


    // theta1 -> BaseRotate (Index 0)
    target_330Hz[0] = base_home + (int)t1;
    
    // theta2 -> Shoulder (Index 1)
    target_330Hz[1] = shoulder_home + (int)t2;
    
    // theta3 -> Elbow (Index 2)
    target_50Hz[2]  = elbow_home + (int)t3;
    // =======================================================
    
    // Send to hardware via linear interpolation
    syncMoveAll(target_50Hz, target_330Hz);

    Serial.println("READY");
}