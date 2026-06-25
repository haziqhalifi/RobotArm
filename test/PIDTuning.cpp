#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Initialize PCA9685 driver at default I2C address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ 50 // Servos run at 50Hz (20,000 microseconds per cycle)

// Struct to handle calibration data
struct CalibJoint {
    const char* name;
    uint8_t channel;
    int currentUs;
};

// INITIAL STARTUP VALUES (Safe middle ground: ~90 degrees for standard servos)
// We use 500us as a baseline start for all joints so nothing clips on boot.
CalibJoint joints[] = {
    {"Base",        5, 1500},
    {"Shoulder",    4, 1500},
    {"Elbow",       3, 1500},
    {"Wrist Pitch", 1, 1500},
    {"Wrist Roll",  2, 1500},
    {"Gripper",     0, 1500}
};
const int NUM_JOINTS = 6;
int activeJointIndex = 0; // Starts tracking the Base

void setServoPulseUs(uint8_t channel, int us) {
    // Formula: (us / 20000.0) * 4096
    double pulseLength = (us / 20000.0) * 4096.0;
    pwm.setPWM(channel, 0, (uint16_t)pulseLength);
}

void printStatus() {
    Serial.println("\n----------------------------------------------------");
    Serial.printf("ACTIVE JOINT: [%d] %s (PCA9685 Ch: %d)\n", 
                  activeJointIndex, joints[activeJointIndex].name, joints[activeJointIndex].channel);
    Serial.printf("Current Pulse Width: %d us\n", joints[activeJointIndex].currentUs);
    Serial.println("----------------------------------------------------");
    Serial.println("CONTROLS:");
    Serial.println("  [A] Decrease -100us | [Z] Increase +100us (Coarse)");
    Serial.println("  [S] Decrease -10us  | [X] Increase +10us  (Medium)");
    Serial.println("  [D] Decrease -2us   | [C] Increase +2us   (Fine Tuner)");
    Serial.println("  [0 to 5] - Switch Active Joint Target");
    Serial.println("----------------------------------------------------");
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Wire.begin(21, 22); // ESP32 Pins: SDA=21, SCL=22
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);

    // Send all servos to safe midpoint home immediately
    Serial.println("Initializing all joints to safe 1500us midpoint...");
    for (int i = 0; i < NUM_JOINTS; i++) {
        setServoPulseUs(joints[i].channel, joints[i].currentUs);
    }
    
    delay(1500);
    printStatus();
}

void loop() {
    if (Serial.available() > 0) {
        char c = Serial.read();
        int step = 0;
        
        // Handle joint switching selection
        if (c >= '0' && c <= '5') {
            activeJointIndex = c - '0';
            printStatus();
            return;
        }

        // Determine step adjustments based on input keys
        switch (c) {
            // Coarse adjustments
            case 'a': case 'A': step = -100; break;
            case 'z': case 'Z': step = 100;  break;
            
            // Medium adjustments
            case 's': case 'S': step = -10;  break;
            case 'x': case 'X': step = 10;   break;
            
            // Fine adjustments 
            case 'd': case 'D': step = -2;   break;
            case 'c': case 'C': step = 2;    break;
            
            default: return; // Ignore formatting syntax like line spaces (\n)
        }

        // Apply new values while protecting against out-of-bounds math errors 
        // Absolute absolute safety window: 400us to 2600us
        int targetUs = joints[activeJointIndex].currentUs + step;
        joints[activeJointIndex].currentUs = constrain(targetUs, 400, 2600);

        // Instantly move the single targeted servo
        setServoPulseUs(joints[activeJointIndex].channel, joints[activeJointIndex].currentUs);
        
        // Print live feedback tracking data
        Serial.printf("Joint %s -> %d us\n", joints[activeJointIndex].name, joints[activeJointIndex].currentUs);
    }
}