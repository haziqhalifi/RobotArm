#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include "control_page.h"

// Initialize PCA9685 driver at default I2C address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Preferences preferences;
WebServer server(80);

const char* AP_NAME = "RobotArm-Control";
const char* AP_PASSWORD = "robotarm123";

#define SERVO_FREQ 50 // Servos run at 50Hz (20,000 microseconds per cycle)

// User-adjustable movement speed, hard-limited to 60% for safety.
constexpr int MAX_MOVEMENT_SPEED_PERCENT = 60;
int movementSpeedPercent = 30;

// Timer for active Forward Kinematics serial printing
unsigned long lastFKPrintTime = 0;

// Struct to handle calibration data
struct CalibJoint {
    const char* name;
    uint8_t channel;
    int currentUs;
    int minUs;
    int maxUs;
    int maxDegrees;
};

// INITIAL STARTUP VALUES (Safe middle ground: ~90 degrees for standard servos)
CalibJoint joints[] = {
    {"Base",        5, 1520, 640, 2600, 270},
    {"Shoulder",    4, 1830, 840, 1990, 110},
    {"Elbow",       3, 2380, 750, 2600, 155},
    {"Wrist Pitch", 1, 1460, 400, 1850, 140},
    {"Wrist Roll",  2, 500,  500, 2400, 180},
    {"Gripper",     0, 1470, 830, 1500, 60}
};
const int NUM_JOINTS = 6;
int activeJointIndex = 0; // Starts tracking the Base
bool servoOutputEnabled = true;

constexpr int MAX_ROUTINES = 6;
constexpr int MAX_ROUTINE_STEPS = 30;
struct SavedRoutine {
    char name[21];
    uint8_t stepCount;
    int16_t poses[MAX_ROUTINE_STEPS][NUM_JOINTS];
};
SavedRoutine routines[MAX_ROUTINES] = {};
bool recording = false;
SavedRoutine recordingBuffer = {};
bool routineRunning = false;
int runningRoutine = -1;
int runningStep = 0;
unsigned long routineHoldStarted = 0;
unsigned long routineLastMove = 0;
bool routinePoseReached = false;
SavedRoutine* adhocRoutine = nullptr;

// Pick-and-place demonstration poses
const int HOME_POSE[NUM_JOINTS]  = {1500, 1850, 1480, 1500, 1460, 900};
const int RIGHT_TEST_POSE[NUM_JOINTS] = {860, 1850, 2300, 1520, 2400, 900};
const int FRONT_TEST_POSE[NUM_JOINTS] = {1510, 1850, 2300, 1520, 1460, 900};
const int LEFT_TEST_POSE[NUM_JOINTS] = {2165, 1850, 2300, 1520, 500, 900};
const int PICK_UP[NUM_JOINTS]    = {1540, 1850, 2300, 1520, 1460, 830};
const int PICK_DOWN[NUM_JOINTS]  = {1540, 1850, 2400, 960, 1460, 830};
const int GRAB_POSE[NUM_JOINTS]  = {1540, 1850, 2400, 960, 1460, 1150};
const int LIFT_POSE[NUM_JOINTS]  = {1540, 1850, 2400, 1480, 1460, 1150};
const int PLACE_UP[NUM_JOINTS]   = {640, 1850, 2400, 1200, 1460, 1150};
const int PLACE_DOWN[NUM_JOINTS] = {640, 1850, 2400, 1200, 1460, 1150};
const int RELEASE_POSE[NUM_JOINTS] = {640, 1850, 2400, 1200, 1460, 1110};

const int* const PICK_PLACE_POSES[] = {
    PICK_UP, PICK_DOWN, GRAB_POSE, LIFT_POSE,
    PLACE_UP, PLACE_DOWN, RELEASE_POSE, HOME_POSE
};
const unsigned long POSE_HOLD_MS[] = {
    1000, 1200, 800, 1200, 1200, 1200, 800, 1500
};
const unsigned long SERVO_STEP_INTERVAL_MS = 20;
const int NUM_PICK_PLACE_POSES = sizeof(PICK_PLACE_POSES) / sizeof(PICK_PLACE_POSES[0]);

constexpr double ARM_HOME_Z_MM = 225.0;
constexpr double ARM_MAX_REACH_MM = 225.0;

// Fitted from measured serial-monitor coordinates to physical coordinates:
// (55,-91,128)->(0,0,225), (-24,41,187)->(0,225,0),
// (41,24,167)->(225,0,0), (-41,24,167)->(-225,0,0).
const double SERIAL_MONITOR_TO_REAL_CALIB[3][3] = {
    { 5.487804878048780,  0.549786194257797,  6.118047588539392 },
    { 0.0,               -5.360415394013420, 15.806353084911360 },
    { 0.0,               -2.748930971288940,  2.336591325595593 }
};
const double SERIAL_MONITOR_TO_REAL_OFFSET[3] = {
    -1034.9088159482656,
    -2511.0109957238756,
    -324.2364080635294
};
const int RX_HOME_OFFSET = -91;
const int RY_HOME_OFFSET = -297;
const int RZ_HOME_OFFSET = -28;
const int RX_MIN_DEGREES = -90;
const int RX_MAX_DEGREES = 90;

bool pickPlaceRunning = false;
int pickPlaceStage = 0;
unsigned long poseStartedAt = 0;
unsigned long lastServoStepAt = 0;
bool poseReached = false;

int normalizeSignedDegrees(int degrees) {
    degrees %= 360;
    if (degrees > 180) { degrees -= 360; }
    if (degrees < -180) { degrees += 360; }
    return degrees;
}

void applyRotationHomeOffset(int &rx, int &ry, int &rz) {
    rx = normalizeSignedDegrees(rx + RX_HOME_OFFSET);
    rx = constrain(rx, RX_MIN_DEGREES, RX_MAX_DEGREES);
    ry = normalizeSignedDegrees(ry + RY_HOME_OFFSET);
    rz = normalizeSignedDegrees(rz + RZ_HOME_OFFSET);
}

int roundMm(double value) {
    return (int)(value >= 0.0 ? value + 0.5 : value - 0.5);
}

void calibrateRawFkPose(double &x, double &y, double &z) {
    const double rawX = x;
    const double rawY = y;
    const double rawZ = z;
    x = SERIAL_MONITOR_TO_REAL_CALIB[0][0] * rawX + SERIAL_MONITOR_TO_REAL_CALIB[0][1] * rawY + SERIAL_MONITOR_TO_REAL_CALIB[0][2] * rawZ + SERIAL_MONITOR_TO_REAL_OFFSET[0];
    y = SERIAL_MONITOR_TO_REAL_CALIB[1][0] * rawX + SERIAL_MONITOR_TO_REAL_CALIB[1][1] * rawY + SERIAL_MONITOR_TO_REAL_CALIB[1][2] * rawZ + SERIAL_MONITOR_TO_REAL_OFFSET[1];
    z = SERIAL_MONITOR_TO_REAL_CALIB[2][0] * rawX + SERIAL_MONITOR_TO_REAL_CALIB[2][1] * rawY + SERIAL_MONITOR_TO_REAL_CALIB[2][2] * rawZ + SERIAL_MONITOR_TO_REAL_OFFSET[2];

    const double reach = sqrt(x * x + y * y + z * z);
    if (reach > ARM_MAX_REACH_MM && reach > 0.0) {
        const double scale = ARM_MAX_REACH_MM / reach;
        x *= scale;
        y *= scale;
        z *= scale;
    }
    x = constrain(x, -ARM_MAX_REACH_MM, ARM_MAX_REACH_MM);
    y = constrain(y, -ARM_MAX_REACH_MM, ARM_MAX_REACH_MM);
    z = constrain(z, 0.0, ARM_HOME_Z_MM);
}

struct ArmPose {
    int x;
    int y;
    int z;
    int rx;
    int ry;
    int rz;
};

bool isHomeShape() {
    constexpr int HOME_PULSE_TOLERANCE_US = 20;
    // Base rotation and gripper opening do not change the home XYZ position.
    for (int joint = 1; joint <= 4; joint++) {
        if (abs(joints[joint].currentUs - HOME_POSE[joint]) > HOME_PULSE_TOLERANCE_US) { return false; }
    }
    return true;
}

void printStatus();
void printActiveJoint();
void saveCurrentRecording();
String getJointTheoreticalFKString(int jointIndex);
void printSelectedJointFK();
void moveToPose(const int pose[]);
void setupWebUI();

void setServoPulseUs(uint8_t channel, int us) {
    double pulseLength = (us / 20000.0) * 4096.0;
    if (servoOutputEnabled) {
        pwm.setPWM(channel, 0, (uint16_t)pulseLength);
    }
}

int clampJointPulse(int jointIndex, int us) {
    return constrain(us, joints[jointIndex].minUs, joints[jointIndex].maxUs);
}

int pulseToDegrees(int jointIndex, int us) {
    const CalibJoint& joint = joints[jointIndex];
    const long pulseRange = joint.maxUs - joint.minUs;
    const long offset = clampJointPulse(jointIndex, us) - joint.minUs;
    const int degrees = (offset * joint.maxDegrees + pulseRange / 2) / pulseRange;
    return jointIndex == 2 ? joint.maxDegrees - degrees : degrees;
}

int degreesToPulse(int jointIndex, int degrees) {
    const CalibJoint& joint = joints[jointIndex];
    degrees = constrain(degrees, 0, joint.maxDegrees);
    if (jointIndex == 2) {
        degrees = joint.maxDegrees - degrees;
    }
    const long pulseRange = joint.maxUs - joint.minUs;
    return joint.minUs + (degrees * pulseRange + joint.maxDegrees / 2) / joint.maxDegrees;
}

ArmPose calculateArmPose() {
    if (isHomeShape()) {
        return { 0, 0, (int)ARM_HOME_Z_MM, 0, 0, 90 };
    }

    double q0 = radians(pulseToDegrees(0, joints[0].currentUs));
    double q1 = radians(pulseToDegrees(1, joints[1].currentUs));
    double q2 = radians(pulseToDegrees(2, joints[2].currentUs));
    double q3 = radians(pulseToDegrees(3, joints[3].currentUs));
    double q4 = radians(pulseToDegrees(4, joints[4].currentUs));

    const double L1 = 80.0;
    const double L2 = 120.0;
    const double L3 = 120.0;
    const double L4 = 50.0;

    double r = L2 * cos(q1) + L3 * cos(q1 + q2) + L4 * cos(q1 + q2 + q3);
    double x = r * cos(q0);
    double y = r * sin(q0);
    double z = L1 + L2 * sin(q1) + L3 * sin(q1 + q2) + L4 * sin(q1 + q2 + q3);
    calibrateRawFkPose(x, y, z);

    int rx = (int)degrees(q4) % 360;
    int ry = (int)degrees(q1 + q2 + q3) % 360;
    int rz = (int)degrees(q0) % 360;
    applyRotationHomeOffset(rx, ry, rz);

    return { roundMm(x), roundMm(y), roundMm(z), rx, ry, rz };
}

String currentStateJson() {
    const ArmPose pose = calculateArmPose();
    String json = "{\"degrees\":[";
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (i > 0) { json += ","; }
        json += String(pulseToDegrees(i, joints[i].currentUs));
    }
    json += "],\"pose\":{\"x\":";
    json += String(pose.x);
    json += ",\"y\":";
    json += String(pose.y);
    json += ",\"z\":";
    json += String(pose.z);
    json += ",\"rx\":";
    json += String(pose.rx);
    json += ",\"ry\":";
    json += String(pose.ry);
    json += ",\"rz\":";
    json += String(pose.rz);
    json += "}}";
    return json;
}

// Function to calculate and format Forward Kinematics output to match your image slide
String getTheoreticalFKString() {
    const ArmPose pose = calculateArmPose();

    char buffer[140];
    snprintf(buffer, sizeof(buffer), "X: %d mm, Y: %d mm, Z: %d mm | Rx: %d, Ry: %d, Rz: %d",
             pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz);

    return String(buffer);
}

String getJointTheoreticalFKString(int jointIndex) {
    if (jointIndex < 0 || jointIndex >= NUM_JOINTS) {
        return String("Invalid joint index");
    }

    double q0 = radians(pulseToDegrees(0, joints[0].currentUs));
    double q1 = radians(pulseToDegrees(1, joints[1].currentUs));
    double q2 = radians(pulseToDegrees(2, joints[2].currentUs));
    double q3 = radians(pulseToDegrees(3, joints[3].currentUs));
    double q4 = radians(pulseToDegrees(4, joints[4].currentUs));

    const double L1 = 80.0;
    const double L2 = 120.0;
    const double L3 = 120.0;
    const double L4 = 50.0;

    double x = 0;
    double y = 0;
    double z = 0;
    int rx = 0;
    int ry = 0;
    int rz = 0;

    switch (jointIndex) {
        case 0:
            x = 0;
            y = 0;
            z = 0;
            rx = 0;
            ry = 0;
            rz = (int)degrees(q0) % 360;
            break;
        case 1:
            x = 0;
            y = 0;
            z = L1;
            rx = 0;
            ry = 0;
            rz = (int)degrees(q0) % 360;
            break;
        case 2: {
            double r = L2 * cos(q1);
            x = r * cos(q0);
            y = r * sin(q0);
            z = L1 + L2 * sin(q1);
            rx = 0;
            ry = (int)degrees(q1) % 360;
            rz = (int)degrees(q0) % 360;
            break;
        }
        case 3: {
            double r = L2 * cos(q1) + L3 * cos(q1 + q2);
            x = r * cos(q0);
            y = r * sin(q0);
            z = L1 + L2 * sin(q1) + L3 * sin(q1 + q2);
            rx = 0;
            ry = (int)degrees(q1 + q2) % 360;
            rz = (int)degrees(q0) % 360;
            break;
        }
        case 4: {
            double r = L2 * cos(q1) + L3 * cos(q1 + q2) + L4 * cos(q1 + q2 + q3);
            x = r * cos(q0);
            y = r * sin(q0);
            z = L1 + L2 * sin(q1) + L3 * sin(q1 + q2) + L4 * sin(q1 + q2 + q3);
            rx = (int)degrees(q4) % 360;
            ry = (int)degrees(q1 + q2 + q3) % 360;
            rz = (int)degrees(q0) % 360;
            break;
        }
        case 5: {
            double r = L2 * cos(q1) + L3 * cos(q1 + q2) + L4 * cos(q1 + q2 + q3);
            x = r * cos(q0);
            y = r * sin(q0);
            z = L1 + L2 * sin(q1) + L3 * sin(q1 + q2) + L4 * sin(q1 + q2 + q3);
            rx = (int)degrees(q4) % 360;
            ry = (int)degrees(q1 + q2 + q3) % 360;
            rz = (int)degrees(q0) % 360;
            break;
        }
    }
    applyRotationHomeOffset(rx, ry, rz);

    double calibratedX = x;
    double calibratedY = y;
    double calibratedZ = z;
    calibrateRawFkPose(calibratedX, calibratedY, calibratedZ);

    char buffer[160];
    snprintf(buffer, sizeof(buffer), "Joint %d %s | X: %d mm, Y: %d mm, Z: %d mm | Rx: %d, Ry: %d, Rz: %d",
             jointIndex,
             joints[jointIndex].name,
             roundMm(calibratedX),
             roundMm(calibratedY),
             roundMm(calibratedZ),
             rx,
             ry,
             rz);

    return String(buffer);
}

void printSelectedJointFK() {
    Serial.println("\n========== SELECTED JOINT THEORETICAL POSE ==========");
    Serial.println(getJointTheoreticalFKString(activeJointIndex));
    Serial.println("=================================");
}

void moveJointSlowly(int jointIndex, int targetUs) {
    targetUs = clampJointPulse(jointIndex, targetUs);
    int currentUs = joints[jointIndex].currentUs;
    movementSpeedPercent = constrain(movementSpeedPercent, 1, MAX_MOVEMENT_SPEED_PERCENT);
    int stepSize = map(movementSpeedPercent, 1, 100, 1, 12);
    int stepDelayMs = map(movementSpeedPercent, 1, 100, 25, 1);
    int direction = targetUs >= currentUs ? stepSize : -stepSize;

    while (currentUs != targetUs) {
        currentUs += direction;
        if ((direction > 0 && currentUs > targetUs) || (direction < 0 && currentUs < targetUs)) {
            currentUs = targetUs;
        }
        joints[jointIndex].currentUs = currentUs;
        setServoPulseUs(joints[jointIndex].channel, currentUs);
        delay(stepDelayMs);
        yield();
    }
}

void returnHomeOneJointAtATime() {
    const int homingOrder[NUM_JOINTS] = {5, 3, 4, 1, 2, 0};
    servoOutputEnabled = true;
    for (int i = 0; i < NUM_JOINTS; i++) {
        const int joint = homingOrder[i];
        Serial.printf("Homing %s (%d/%d)\n", joints[joint].name, i + 1, NUM_JOINTS);
        moveJointSlowly(joint, HOME_POSE[joint]);
        delay(250);
    }
}

void performStartupHoming() {
    constexpr unsigned long HOMING_STAGE_SETTLE_MS = 500;
    servoOutputEnabled = true;
    Serial.println("Starting staged software homing...");
    moveJointSlowly(5, 1200);
    moveJointSlowly(3, HOME_POSE[3]);
    moveJointSlowly(4, HOME_POSE[4]);
    delay(HOMING_STAGE_SETTLE_MS);
    moveJointSlowly(1, HOME_POSE[1]);
    moveJointSlowly(2, HOME_POSE[2]);
    delay(HOMING_STAGE_SETTLE_MS);
    moveJointSlowly(0, HOME_POSE[0]);
    delay(HOMING_STAGE_SETTLE_MS);
    Serial.println("Staged software homing complete.");
}

void setAllOutputs(bool enabled) {
    servoOutputEnabled = enabled;
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (enabled) {
            setServoPulseUs(joints[i].channel, joints[i].currentUs);
        } else {
            pwm.setPWM(joints[i].channel, 0, 0);
        }
    }
}

void performInitialCheckMotion() {
    Serial.println("Performing a small safety movement before homing...");
    const int deltaUs = 80;
    int checkPose[NUM_JOINTS];
    for (int i = 0; i < NUM_JOINTS; i++) {
        int offset = (i % 2 == 0) ? deltaUs : -deltaUs;
        checkPose[i] = clampJointPulse(i, joints[i].currentUs + offset);
    }
    moveToPose(checkPose);
    delay(200);
}

bool askRobotConditionOk() {
    while (true) {
        Serial.println("Is the robot condition ok? Type y for yes or n for no.");
        while (Serial.available() == 0) {
            delay(50);
            yield();
        }
        char c = Serial.read();
        while (Serial.available() > 0) { Serial.read(); }
        if (c == 'y' || c == 'Y') {
            Serial.println("Robot condition confirmed. Proceeding to homing.");
            return true;
        }
        if (c == 'n' || c == 'N') {
            Serial.println("Robot condition not ok. Please correct issues and type y when ready.");
            continue;
        }
        Serial.println("Invalid input. Please type y or n.");
    }
}

void saveRoutines() {
    preferences.putBytes("routines", routines, sizeof(routines));
}

void captureRoutinePose() {
    if (!recording || recordingBuffer.stepCount >= MAX_ROUTINE_STEPS) { return; }
    if (recordingBuffer.stepCount > 0) {
        const int lastStep = recordingBuffer.stepCount - 1;
        bool unchanged = true;
        for (int joint = 0; joint < NUM_JOINTS; joint++) {
            if (recordingBuffer.poses[lastStep][joint] != joints[joint].currentUs) {
                unchanged = false;
                break;
            }
        }
        if (unchanged) { return; }
    }
    int step = recordingBuffer.stepCount;
    for (int joint = 0; joint < NUM_JOINTS; joint++) {
        recordingBuffer.poses[step][joint] = joints[joint].currentUs;
    }
    recordingBuffer.stepCount++;
}

void saveCurrentRecording() {
    if (!recording || recordingBuffer.stepCount == 0) {
        Serial.println("No recorded poses to save. Press 5 to capture at least one pose.");
        return;
    }
    int slot = -1;
    for (int i = 0; i < MAX_ROUTINES; i++) {
        if (routines[i].stepCount == 0) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        Serial.println("No free routine slots available. Delete a task first.");
        return;
    }
    snprintf(recordingBuffer.name, sizeof(recordingBuffer.name), "Task %d", slot + 1);
    routines[slot] = recordingBuffer;
    recording = false;
    saveRoutines();
    Serial.printf("Saved recording to slot %d: %s\n", slot + 1, routines[slot].name);
}

void deleteFirstSavedRoutine() {
    for (int i = 0; i < MAX_ROUTINES; i++) {
        if (routines[i].stepCount > 0) {
            memset(&routines[i], 0, sizeof(SavedRoutine));
            saveRoutines();
            Serial.printf("Deleted saved task slot %d and freed the routine slot.\n", i + 1);
            printStatus();
            return;
        }
    }
    Serial.println("No saved tasks to delete.");
}

void updateRoutinePlayback() {
    if (!routineRunning || (runningRoutine < 0 && adhocRoutine == nullptr)) { return; }
    SavedRoutine& routine = (runningRoutine >= 0) ? routines[runningRoutine] : *adhocRoutine;
    unsigned long now = millis();
    if (!routinePoseReached) {
        if (now - routineLastMove < 10) { return; }
        routineLastMove = now;
        bool reached = true;
        const int safeSpeed = constrain(movementSpeedPercent, 1, MAX_MOVEMENT_SPEED_PERCENT);
        int stepSize = map(safeSpeed, 1, 100, 1, 12);
        for (int joint = 0; joint < NUM_JOINTS; joint++) {
            int safeTarget = clampJointPulse(joint, routine.poses[runningStep][joint]);
            int difference = safeTarget - joints[joint].currentUs;
            if (difference != 0) {
                joints[joint].currentUs += constrain(difference, -stepSize, stepSize);
                setServoPulseUs(joints[joint].channel, joints[joint].currentUs);
                reached = false;
            }
        }
        if (reached) {
            routinePoseReached = true;
            routineHoldStarted = now;
        }
        return;
    }
    if (now - routineHoldStarted < 500) { return; }
    runningStep++;
    if (runningStep >= routine.stepCount) {
        routineRunning = false;
        if (runningRoutine >= 0) { runningRoutine = -1; } else { adhocRoutine = nullptr; }
        runningStep = 0;
        Serial.println("Saved task complete; returning home.");
        returnHomeOneJointAtATime();
        return;
    }
    routinePoseReached = false;
}

void setupWebUI() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_NAME, AP_PASSWORD);

    server.on("/", HTTP_GET, []() {
        server.sendHeader("Cache-Control", "no-store, max-age=0");
        server.send_P(200, "text/html", CONTROL_PAGE);
    });

    server.on("/state", HTTP_GET, []() {
        server.send(200, "application/json", currentStateJson());
    });

    server.on("/set", HTTP_GET, []() {
        if (!server.hasArg("joint") || !server.hasArg("degree")) {
            server.send(400, "text/plain", "Missing joint or degree value");
            return;
        }
        const int joint = server.arg("joint").toInt();
        if (joint < 0 || joint >= NUM_JOINTS) {
            server.send(400, "text/plain", "Invalid joint");
            return;
        }
        const int degree = constrain(server.arg("degree").toInt(), 0, joints[joint].maxDegrees);
        activeJointIndex = joint;
        moveJointSlowly(joint, degreesToPulse(joint, degree));
        server.send(200, "text/plain", String(joints[joint].name) + ": " + String(degree) + " deg");
    });

    server.on("/jog", HTTP_GET, []() {
        if (!server.hasArg("j1") || !server.hasArg("d1")) {
            server.send(400, "application/json", "{\"error\":\"Missing joystick movement\"}");
            return;
        }
        const int firstJoint = server.arg("j1").toInt();
        if (firstJoint < 0 || firstJoint >= NUM_JOINTS) {
            server.send(400, "application/json", "{\"error\":\"Invalid joint\"}");
            return;
        }
        const int firstDegree = constrain(
            pulseToDegrees(firstJoint, joints[firstJoint].currentUs) + server.arg("d1").toInt(),
            0,
            joints[firstJoint].maxDegrees);
        activeJointIndex = firstJoint;
        moveJointSlowly(firstJoint, degreesToPulse(firstJoint, firstDegree));

        if (server.hasArg("j2") && server.hasArg("d2")) {
            const int secondJoint = server.arg("j2").toInt();
            if (secondJoint >= 0 && secondJoint < NUM_JOINTS) {
                const int secondDegree = constrain(
                    pulseToDegrees(secondJoint, joints[secondJoint].currentUs) + server.arg("d2").toInt(),
                    0,
                    joints[secondJoint].maxDegrees);
                moveJointSlowly(secondJoint, degreesToPulse(secondJoint, secondDegree));
            }
        }
        server.send(200, "application/json", currentStateJson());
    });

    server.on("/capture", HTTP_GET, []() {
        if (!recording) {
            server.send(409, "text/plain", "Start a new task first");
            return;
        }
        const uint8_t previousCount = recordingBuffer.stepCount;
        captureRoutinePose();
        if (recordingBuffer.stepCount == previousCount) {
            server.send(409, "text/plain",
                        previousCount >= MAX_ROUTINE_STEPS ? "Maximum 30 steps reached" : "This position is already the last step");
            return;
        }
        server.send(200, "text/plain", "Step " + String(recordingBuffer.stepCount) + " added");
    });

    server.on("/speed", HTTP_GET, []() {
        if (!server.hasArg("value")) {
            server.send(400, "text/plain", "Missing speed value");
            return;
        }
        movementSpeedPercent = constrain(server.arg("value").toInt(), 1, MAX_MOVEMENT_SPEED_PERCENT);
        server.send(200, "text/plain", "Speed set to " + String(movementSpeedPercent) + "%");
    });

    server.on("/preset", HTTP_GET, []() {
        if (!server.hasArg("name")) {
            server.send(400, "text/plain", "Missing preset name");
            return;
        }

        const String preset = server.arg("name");
        const int* pose = nullptr;
        if (preset == "home") {
            pose = HOME_POSE;
        } else if (preset == "left") {
            pose = LEFT_TEST_POSE;
        } else if (preset == "right") {
            pose = RIGHT_TEST_POSE;
        } else if (preset == "front") {
            pose = FRONT_TEST_POSE;
        } else {
            server.send(400, "text/plain", "Unknown preset");
            return;
        }

        routineRunning = false;
        runningRoutine = -1;
        runningStep = 0;
        adhocRoutine = nullptr;
        pickPlaceRunning = false;
        servoOutputEnabled = true;
        moveToPose(pose);
        server.send(200, "text/plain", "Moved to " + preset + " preset");
    });

    server.on("/action", HTTP_GET, []() {
        const String action = server.arg("name");
        if (action == "disable") {
            setAllOutputs(false);
            server.send(200, "text/plain", "Servo output disabled");
        } else if (action == "enable") {
            setAllOutputs(true);
            server.send(200, "text/plain", "Servo output enabled");
        } else if (action == "home") {
            routineRunning = false;
            runningRoutine = -1;
            runningStep = 0;
            adhocRoutine = nullptr;
            pickPlaceRunning = false;
            moveToPose(HOME_POSE);
            server.send(200, "text/plain", "Arm returned home");
        } else if (action == "stop") {
            routineRunning = false;
            runningRoutine = -1;
            runningStep = 0;
            adhocRoutine = nullptr;
            pickPlaceRunning = false;
            server.send(200, "text/plain", "Playback stopped");
        } else {
            server.send(400, "text/plain", "Unknown action");
        }
    });

    server.on("/record", HTTP_GET, []() {
        const String action = server.arg("action");
        if (action == "start") {
            routineRunning = false;
            pickPlaceRunning = false;
            memset(&recordingBuffer, 0, sizeof(recordingBuffer));
            recording = true;
            server.send(200, "text/plain", "New task started; set a position and add a step");
            return;
        }
        if (action == "cancel") {
            recording = false;
            memset(&recordingBuffer, 0, sizeof(recordingBuffer));
            server.send(200, "text/plain", "Recording cancelled");
            return;
        }
        if (action == "save") {
            if (!recording || recordingBuffer.stepCount < 1) {
                server.send(400, "text/plain", "Add at least one step before saving");
                return;
            }
            int slot = -1;
            for (int i = 0; i < MAX_ROUTINES; i++) {
                if (routines[i].stepCount == 0) {
                    slot = i;
                    break;
                }
            }
            if (slot < 0) {
                server.send(409, "text/plain", "Delete a task first (maximum 6)");
                return;
            }
            String name = server.arg("name");
            name.trim();
            if (name.length() == 0) { name = "Saved task"; }
            name.substring(0, 20).toCharArray(recordingBuffer.name, 21);
            routines[slot] = recordingBuffer;
            recording = false;
            saveRoutines();
            server.send(200, "text/plain", "Task saved");
            return;
        }
        server.send(400, "text/plain", "Unknown recording action");
    });

    server.on("/routines", HTTP_GET, []() {
        String json = "[";
        bool first = true;
        for (int i = 0; i < MAX_ROUTINES; i++) {
            if (routines[i].stepCount == 0) { continue; }
            if (!first) { json += ","; }
            first = false;
            String safeName = routines[i].name;
            safeName.replace("\\", "\\\\");
            safeName.replace("\"", "\\\"");
            json += "{\"id\":" + String(i) + ",\"name\":\"" + safeName + "\",\"steps\":" + String(routines[i].stepCount) + "}";
        }
        json += "]";
        server.send(200, "application/json", json);
    });

    server.on("/play", HTTP_GET, []() {
        const int id = server.arg("id").toInt();
        if (id < 0 || id >= MAX_ROUTINES || routines[id].stepCount == 0) {
            server.send(404, "text/plain", "Task not found");
            return;
        }
        recording = false;
        pickPlaceRunning = false;
        servoOutputEnabled = true;
        runningRoutine = id;
        adhocRoutine = nullptr;
        runningStep = 0;
        routinePoseReached = false;
        routineLastMove = millis();
        routineRunning = true;
        server.send(200, "text/plain", "Playing " + String(routines[id].name) + " once");
    });

    server.on("/delete", HTTP_GET, []() {
        if (!server.hasArg("id")) {
            server.send(400, "text/plain", "Missing task id");
            return;
        }
        const int id = server.arg("id").toInt();
        if (id < 0 || id >= MAX_ROUTINES || routines[id].stepCount == 0) {
            server.send(400, "text/plain", "Invalid task");
            return;
        }
        if (runningRoutine == id) {
            routineRunning = false;
            runningRoutine = -1;
        }
        memset(&routines[id], 0, sizeof(SavedRoutine));
        saveRoutines();
        server.send(200, "text/plain", "Task deleted");
    });

    server.onNotFound([]() {
        server.send(404, "text/plain", "Not found");
    });

    server.begin();
    Serial.printf("\nWi-Fi UI: connect to \"%s\" using password \"%s\"\n", AP_NAME, AP_PASSWORD);
    Serial.print("Open http://");
    Serial.println(WiFi.softAPIP());
}

void moveToPose(const int pose[]) {
    if (pose == HOME_POSE) {
        returnHomeOneJointAtATime();
        return;
    }
    for (int i = 0; i < NUM_JOINTS; i++) {
        moveJointSlowly(i, pose[i]);
    }
}

void startPickAndPlace() {
    pickPlaceRunning = true;
    pickPlaceStage = 0;
    poseReached = false;
    lastServoStepAt = millis();
    Serial.println("\nPick-and-place demonstration started. ====State Busy====");
}

bool startFirstSavedRoutine() {
    for (int id = 0; id < MAX_ROUTINES; id++) {
        if (routines[id].stepCount == 0) { continue; }
        recording = false;
        pickPlaceRunning = false;
        servoOutputEnabled = true;
        runningRoutine = id;
        runningStep = 0;
        routinePoseReached = false;
        routineLastMove = millis();
        routineRunning = true;
        Serial.printf("Automatically playing saved task: %s\n", routines[id].name);
        return true;
    }
    Serial.println("No saved task available for automatic playback.");
    return false;
}

void updatePickAndPlace() {
    if (!pickPlaceRunning) { return; }
    const unsigned long now = millis();
    if (!poseReached) {
        if (now - lastServoStepAt < SERVO_STEP_INTERVAL_MS) { return; }
        lastServoStepAt = now;
        bool allAtTarget = true;
        const int safeSpeed = constrain(movementSpeedPercent, 1, MAX_MOVEMENT_SPEED_PERCENT);
        const int servoStepUs = map(safeSpeed, 1, 100, 1, 12);
        const int* target = PICK_PLACE_POSES[pickPlaceStage];
        for (int i = 0; i < NUM_JOINTS; i++) {
            const int safeTarget = clampJointPulse(i, target[i]);
            const int difference = safeTarget - joints[i].currentUs;
            if (difference != 0) {
                const int step = constrain(difference, -servoStepUs, servoStepUs);
                joints[i].currentUs += step;
                setServoPulseUs(joints[i].channel, joints[i].currentUs);
                allAtTarget = false;
            }
        }
        if (allAtTarget) {
            poseReached = true;
            poseStartedAt = now;
        }
        return;
    }
    if (now - poseStartedAt < POSE_HOLD_MS[pickPlaceStage]) { return; }
    pickPlaceStage++;
    if (pickPlaceStage >= NUM_PICK_PLACE_POSES) {
        pickPlaceRunning = false;
        Serial.println("Pick-and-place demonstration complete. Returning to homing mode... ====State Idle====");
        returnHomeOneJointAtATime();
        printStatus();
        return;
    }
    poseReached = false;
    lastServoStepAt = now;
}

void printStatus() {
    int savedCount = 0;
    for (int i = 0; i < MAX_ROUTINES; i++) { if (routines[i].stepCount > 0) { savedCount++; } }
    Serial.println("\n----------------------------------------------------");
    Serial.printf("ACTIVE JOINT: [%d] %s (PCA9685 Ch: %d)\n", activeJointIndex, joints[activeJointIndex].name, joints[activeJointIndex].channel);
    Serial.printf("Current Pulse Width: %d us\n", joints[activeJointIndex].currentUs);
    Serial.printf("Recording: %s | Steps: %d\n", recording ? "ON" : "OFF", recordingBuffer.stepCount);
    Serial.printf("Saved Tasks: %d / %d\n", savedCount, MAX_ROUTINES);
    Serial.println("----------------------------------------------------");
}

void printActiveJoint() { Serial.printf("current joint : %s (%d)\n", joints[activeJointIndex].name, activeJointIndex); }

void printAllJointAngles() {
    Serial.println("\n========== CURRENT POSE ==========");
    for (int i = 0; i < NUM_JOINTS; i++) { Serial.printf("  [%d] %-15s : %5d us\n", i, joints[i].name, joints[i].currentUs); }
    Serial.println("=================================\n");
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    preferences.begin("robot-arm", false);
    if (preferences.getBytesLength("routines") == sizeof(routines)) {
        preferences.getBytes("routines", routines, sizeof(routines));
        for (int i = 0; i < MAX_ROUTINES; i++) {
            if (routines[i].stepCount > MAX_ROUTINE_STEPS) { memset(&routines[i], 0, sizeof(SavedRoutine)); }
            else { routines[i].name[20] = '\0'; }
        }
    }

    Wire.begin(21, 22);
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);

    performInitialCheckMotion();
    if (askRobotConditionOk()) { performStartupHoming(); }
    setupWebUI();
    printStatus();
}

void loop() {
    server.handleClient();

    // Actively print calibrated FK tracking string every 500ms (Non-blocking)
    if (millis() - lastFKPrintTime >= 500) {
        lastFKPrintTime = millis();
        Serial.println(getTheoreticalFKString());
    }

    updatePickAndPlace();
    updateRoutinePlayback();

    if (Serial.available() > 0) {
        char c = Serial.read();
        int step = 0;
        if (c == '\r' || c == '\n') { return; }

        if (c == 'p' || c == 'P') {
            if (recordingBuffer.stepCount > 0) {
                recording = false; pickPlaceRunning = false; servoOutputEnabled = true;
                adhocRoutine = &recordingBuffer; runningRoutine = -2; runningStep = 0;
                routinePoseReached = false; routineLastMove = millis(); routineRunning = true;
                Serial.println("Playing ad-hoc recorded movement.");
                return;
            }
            startPickAndPlace();
            return;
        }

        if (c == 'q' || c == 'Q') {
            pickPlaceRunning = false; moveToPose(HOME_POSE);
            Serial.println("Demonstration stopped; arm returned home.");
            return;
        }

        if (pickPlaceRunning) {
            if (c == 'r' || c == 'R') { printAllJointAngles(); return; }
            pickPlaceRunning = false;
            Serial.println("Pick-and-place demo aborted by manual input.");
        }

        switch (c) {
            case '8': step = 20; break;
            case '2': step = -20; break;
            case '4': activeJointIndex = (activeJointIndex + NUM_JOINTS - 1) % NUM_JOINTS; printStatus(); printActiveJoint(); return;
            case '6': activeJointIndex = (activeJointIndex + 1) % NUM_JOINTS; printStatus(); printActiveJoint(); return;
            case '5':
                if (!recording) { recording = true; memset(&recordingBuffer, 0, sizeof(recordingBuffer)); Serial.println("Recording started."); }
                {
                    uint8_t previousCount = recordingBuffer.stepCount;
                    captureRoutinePose();
                    if (recordingBuffer.stepCount == previousCount) { Serial.println("Pose unchanged or duplicate; not recorded."); }
                    else { Serial.printf("Pose recorded as step %d.\n", recordingBuffer.stepCount); }
                }
                return;
            case '3': saveCurrentRecording(); return;
            case 'd': case 'D': deleteFirstSavedRoutine(); return;
            case '1': routineRunning = false; pickPlaceRunning = false; moveToPose(HOME_POSE); Serial.println("Arm returned home."); return;
            case '7': if (!startFirstSavedRoutine()) { Serial.println("No saved task available."); } return;
            case '0': setAllOutputs(!servoOutputEnabled); Serial.printf("Servo output %s\n", servoOutputEnabled ? "enabled" : "disabled"); return;
            case 'j': case 'J': printSelectedJointFK(); return;
            case 'r': case 'R': printAllJointAngles(); return;
            default: return;
        }

        int targetUs = joints[activeJointIndex].currentUs + step;
        targetUs = clampJointPulse(activeJointIndex, targetUs);
        moveJointSlowly(activeJointIndex, targetUs);
        Serial.printf("Joint %s -> %d us\n", joints[activeJointIndex].name, joints[activeJointIndex].currentUs);
    }
}
