#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Initialize PCA9685 driver at default I2C address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ 50 // Analog/Digital servos run at 50Hz
const int NUM_JOINTS = 6;

// =========================================================================
// MACRO / RECORDING STORAGE CONFIGURATION
// =========================================================================
const int MAX_SAVED_STEPS = 50;           // Maximum number of poses you can record
float macroMemory[MAX_SAVED_STEPS][NUM_JOINTS]; 
int savedStepCount = 0;                  // Tracks total recorded positions
bool isPlayingMacro = false;             // Toggle state for playback engine
int currentPlaybackStep = 0;             // Tracks active playback sequence index
unsigned long lastPlaybackTime = 0;      
const int PLAYBACK_DELAY = 1500;         // Time (ms) spent waiting at each position

// Struct to store configuration and state for each joint
struct Joint {
    uint8_t channel;     // PCA9685 Channel (0-15)
    int minMicroseconds; // Pulse width for 0 degrees (Usually ~500 to 1000)
    int maxMicroseconds; // Pulse width for maximum degrees (Usually ~2000 to 2500)
    float maxAngle;      // Maximum rotation capability (e.g., 180 or 270 degrees)
    float currentAngle;  // Tracks the current physical position
    float targetAngle;   // Destination angle
};

// =========================================================================
// SAFE CONFIGURATION ARRAY
// The arm will instantly snap to CurrentAngle on boot, then smoothly 
// glide to TargetAngle over a few seconds.
// =========================================================================
Joint armJoints[NUM_JOINTS] = {
//   Ch, MinUs, MaxUs, MaxAngle, CurrentAngle, TargetAngle
    {0,  500,   2500,  270.0,    135.0,        135.0 },   // Joint 0: Base
    {1,  500,   2500,  180.0,    90.0,         90.0  },   // Joint 1: Shoulder
    {2,  500,   2500,  180.0,    90.0,         90.0  },   // Joint 2: Elbow
    {3,  500,   2500,  180.0,    90.0,         90.0  },   // Joint 3: Wrist Pitch
    {4,  500,   2500,  180.0,    90.0,         90.0  },   // Joint 4: Wrist Roll
    {5,  500,   2500,  270.0,    45.0,         45.0  }    // Joint 5: Gripper
};

// Function declarations
void setJointAngle(int jointIndex, float angle);
void updateArmMovement(float interpolationStep);
void handleNumpadInput(char c);
void recordCurrentPose();
void runPlayback();

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n====================================================");
    Serial.println("  6DOF ROBOTIC ARM CONTROL SYSTEM INITIALIZED        ");
    Serial.println("====================================================");
    Serial.println(" MANUAL CONTROLS:                                   ");
    Serial.println("  [7/4] Base  | [8/5] Shoulder | [9/6] Elbow        ");
    Serial.println("  [*/ /] WPitch | [-/+] WRoll    | [1/2] Gripper      ");
    Serial.println(" AUTOMATION CONTROLS:                               ");
    Serial.println("  [0] - RECORD CURRENT POSE                         ");
    Serial.println("  [.] - TOGGLE PLAYBACK LOOP                        ");
    Serial.println("====================================================\n");

    // Start I2C on default ESP32 pins (SDA: 21, SCL: 22)
    Wire.begin(21, 22);
    
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);

    // Command all joints to go to their startup safe CurrentAngle coordinates instantly
    for (int i = 0; i < NUM_JOINTS; i++) {
        setJointAngle(i, armJoints[i].currentAngle);
    }
    
    delay(1000); // Wait 1 second for the arm to stabilize at home position
    Serial.println(">> Arm calibrated at safe home position. Awaiting input...");
}

void loop() {
    // 1. Process incoming Serial Numpad key strokes
    if (Serial.available() > 0) {
        char inputChar = Serial.read();
        
        // Safety feature: Typing manual keys instantly aborts playback loops
        if (inputChar != '.' && inputChar != '0' && inputChar != '\n' && inputChar != '\r') { 
            if (isPlayingMacro) {
                isPlayingMacro = false; 
                Serial.println(">> [PLAYBACK ABORTED] Manual user override detected.");
            }
        }
        
        handleNumpadInput(inputChar);
    }

    // 2. Automated macro task manager running asynchronously 
    if (isPlayingMacro) {
        runPlayback();
    }

    // 3. Smoothly step the arm towards targets via LERP (3% increments)
    updateArmMovement(0.03); 
    
    delay(20); // Maintain roughly a 50Hz control cycle matching the PWM
}

/**
 * Direct mapping array parser translating characters to positional joint changes
 */
void handleNumpadInput(char c) {
    const float moveStep = 5.0; // Degrees altered per single button press

    switch(c) {
        // Joint 0: Base
        case '7': armJoints[0].targetAngle = constrain(armJoints[0].targetAngle + moveStep, 0.0f, armJoints[0].maxAngle); break;
        case '4': armJoints[0].targetAngle = constrain(armJoints[0].targetAngle - moveStep, 0.0f, armJoints[0].maxAngle); break;
        
        // Joint 1: Shoulder
        case '8': armJoints[1].targetAngle = constrain(armJoints[1].targetAngle + moveStep, 0.0f, armJoints[1].maxAngle); break;
        case '5': armJoints[1].targetAngle = constrain(armJoints[1].targetAngle - moveStep, 0.0f, armJoints[1].maxAngle); break;
        
        // Joint 2: Elbow
        case '9': armJoints[2].targetAngle = constrain(armJoints[2].targetAngle + moveStep, 0.0f, armJoints[2].maxAngle); break;
        case '6': armJoints[2].targetAngle = constrain(armJoints[2].targetAngle - moveStep, 0.0f, armJoints[2].maxAngle); break;
        
        // Joint 3: Wrist Pitch
        case '*': armJoints[3].targetAngle = constrain(armJoints[3].targetAngle + moveStep, 0.0f, armJoints[3].maxAngle); break;
        case '/': armJoints[3].targetAngle = constrain(armJoints[3].targetAngle - moveStep, 0.0f, armJoints[3].maxAngle); break;
        
        // Joint 4: Wrist Roll
        case '-': armJoints[4].targetAngle = constrain(armJoints[4].targetAngle + moveStep, 0.0f, armJoints[4].maxAngle); break;
        case '+': armJoints[4].targetAngle = constrain(armJoints[4].targetAngle - moveStep, 0.0f, armJoints[4].maxAngle); break;
        
        // Joint 5: Gripper
        case '1': armJoints[5].targetAngle = constrain(armJoints[5].targetAngle + moveStep, 0.0f, armJoints[5].maxAngle); break;
        case '2': armJoints[5].targetAngle = constrain(armJoints[5].targetAngle - moveStep, 0.0f, armJoints[5].maxAngle); break;

        // Dedicated Button: Record Position Matrix
        case '0': 
            recordCurrentPose(); 
            break;

        // Dedicated Button: Play/Pause Macro Toggle
        case '.': 
            if (savedStepCount > 0) {
                isPlayingMacro = !isPlayingMacro;
                if (isPlayingMacro) {
                    currentPlaybackStep = 0;
                    lastPlaybackTime = millis() - PLAYBACK_DELAY; // Trigger immediate execution of first step
                    Serial.println(">> [PLAYBACK STATUS: RUNNING]");
                } else {
                    Serial.println(">> [PLAYBACK STATUS: PAUSED]");
                }
            } else {
                Serial.println("!! [ERROR] Memory buffer is empty. Record positions first by hitting '0'.");
            }
            break;

        default: 
            return; // Discard invalid characters like newlines or spaces safely
    }

    // Output telemetry updates during manual configuration adjustments
    if (!isPlayingMacro && c != '0') {
        Serial.printf("Targets: B:%.0f° | S:%.0f° | E:%.0f° | WP:%.0f° | WR:%.0f° | G:%.0f°\n",
                      armJoints[0].targetAngle, armJoints[1].targetAngle, armJoints[2].targetAngle,
                      armJoints[3].targetAngle, armJoints[4].targetAngle, armJoints[5].targetAngle);
    }
}

/**
 * Logs the active multi-joint destination target vector to the global matrix array
 */
void recordCurrentPose() {
    if (savedStepCount < MAX_SAVED_STEPS) {
        for (int i = 0; i < NUM_JOINTS; i++) {
            macroMemory[savedStepCount][i] = armJoints[i].targetAngle;
        }
        Serial.printf(">> [RECORDED] Saved current pose into Step slot [%d].\n", savedStepCount);
        savedStepCount++;
    } else {
        Serial.println("!! [WARNING] Max memory capacity reached! Clear flash or restart microcontroller to reset.");
    }
}

/**
 * Non-blocking timer tracking execution cycles to transition live targets smoothly
 */
void runPlayback() {
    if (millis() - lastPlaybackTime >= PLAYBACK_DELAY) {
        lastPlaybackTime = millis();

        Serial.printf(">> [PLAYING] Progressing to Step [%d/%d]...\n", currentPlaybackStep + 1, savedStepCount);
        
        // Feed the indexed sequence profile configurations down to our main target registers
        for (int i = 0; i < NUM_JOINTS; i++) {
            armJoints[i].targetAngle = macroMemory[currentPlaybackStep][i];
        }

        // Advance to the next sequence item, or loop back to zero if finished
        currentPlaybackStep++;
        if (currentPlaybackStep >= savedStepCount) {
            currentPlaybackStep = 0; 
            Serial.println(">> Sequence cycle reached the end. Looping to Step 1...");
        }
    }
}

/**
 * Maps target degrees into microseconds and sends the pulse command to the PCA9685.
 */
void setJointAngle(int jointIndex, float angle) {
    Joint &j = armJoints[jointIndex];
    
    // Map the degree value to pulse length in microseconds
    float us = j.minMicroseconds + (angle / j.maxAngle) * (j.maxMicroseconds - j.minMicroseconds);
    
    // Convert microseconds to PCA9685 steps (0 to 4095 over a single cycle length)
    // Formula: (us / cycle_period_us) * 4096
    double pulseLength = (us / 20000.0) * 4096.0;
    
    pwm.setPWM(j.channel, 0, (uint16_t)pulseLength);
}

/**
 * Linearly interpolates (LERP) current position variables closer to target steps.
 */
void updateArmMovement(float interpolationStep) {
    for (int i = 0; i < NUM_JOINTS; i++) {
        float diff = armJoints[i].targetAngle - armJoints[i].currentAngle;
        
        if (abs(diff) > 0.1) { // Threshold to prevent minor floating-point jitter
            armJoints[i].currentAngle += diff * interpolationStep;
            setJointAngle(i, armJoints[i].currentAngle);
        }
    }
}