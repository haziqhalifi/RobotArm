#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>

// Initialize PCA9685 driver at default I2C address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Preferences preferences;

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

// Affine calibration from measured Serial Monitor coordinates to real arm
// coordinates. The fitted extreme points are:
// (55,-91,128)->(0,0,225), (-24,41,187)->(0,225,0),
// (41,24,167)->(225,0,0), (-33,-25,193)->(-225,0,0).
const double SERIAL_TO_REAL_CALIB[3][4] = {
    {  3.864515910972908,   2.785898974194313,  -1.058337417403212,  176.4356209757836 },
    { -6.211870278674415,   3.196559241545992, -11.655653761005919, 2124.4634377165357 },
    { -1.802342284487869,  -0.266168691801997,  -5.631369036553861, 1020.7227113717453 }
};

const double CALIB_EXTREME_RAW_POINTS[4][3] = {
    {  55.0, -91.0, 128.0 },
    { -24.0,  41.0, 187.0 },
    {  41.0,  24.0, 167.0 },
    { -33.0, -25.0, 193.0 }
};

// Raw FK of HOME_POSE with the current link lengths and pulse-to-degree
// mapping. The physical home reference is X=0, Y=0, Z=225 mm.
const double HOME_RAW_FK_POINT[3] = { 51.51044720871343, -96.87706130725242, 131.6581325423549 };
const double HOME_REAL_CORRECTION[3] = { 33.72986990268817, 39.74765168982428, 12.74663606195498 };

bool pickPlaceRunning = false;
int pickPlaceStage = 0;
unsigned long poseStartedAt = 0;
unsigned long lastServoStepAt = 0;
bool poseReached = false;

double squaredRawDistance(double ax, double ay, double az, const double point[3]) {
    const double dx = ax - point[0];
    const double dy = ay - point[1];
    const double dz = az - point[2];
    return dx * dx + dy * dy + dz * dz;
}

double homeCorrectionWeight(double rawX, double rawY, double rawZ) {
    double weight = 1.0;
    for (int i = 0; i < 4; i++) {
        const double sampleDistance = squaredRawDistance(rawX, rawY, rawZ, CALIB_EXTREME_RAW_POINTS[i]);
        const double homeDistance = squaredRawDistance(HOME_RAW_FK_POINT[0], HOME_RAW_FK_POINT[1], HOME_RAW_FK_POINT[2], CALIB_EXTREME_RAW_POINTS[i]);
        if (homeDistance > 0.000001) {
            weight *= sampleDistance / homeDistance;
        }
    }
    if (weight < 0.0) { return 0.0; }
    if (weight > 1.0) { return 1.0; }
    return weight;
}

void transformSerialToReal(double &x, double &y, double &z) {
    double rawX = x;
    double rawY = y;
    double rawZ = z;
    x = SERIAL_TO_REAL_CALIB[0][0] * rawX + SERIAL_TO_REAL_CALIB[0][1] * rawY + SERIAL_TO_REAL_CALIB[0][2] * rawZ + SERIAL_TO_REAL_CALIB[0][3];
    y = SERIAL_TO_REAL_CALIB[1][0] * rawX + SERIAL_TO_REAL_CALIB[1][1] * rawY + SERIAL_TO_REAL_CALIB[1][2] * rawZ + SERIAL_TO_REAL_CALIB[1][3];
    z = SERIAL_TO_REAL_CALIB[2][0] * rawX + SERIAL_TO_REAL_CALIB[2][1] * rawY + SERIAL_TO_REAL_CALIB[2][2] * rawZ + SERIAL_TO_REAL_CALIB[2][3];

    const double homeWeight = homeCorrectionWeight(rawX, rawY, rawZ);
    x += HOME_REAL_CORRECTION[0] * homeWeight;
    y += HOME_REAL_CORRECTION[1] * homeWeight;
    z += HOME_REAL_CORRECTION[2] * homeWeight;
}

void printStatus();
void printActiveJoint();
void saveCurrentRecording();
String getJointTheoreticalFKString(int jointIndex);
void printSelectedJointFK();
void moveToPose(const int pose[]);

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

// Function to calculate and format Forward Kinematics output to match your image slide
String getTheoreticalFKString() {
    double q0 = radians(pulseToDegrees(0, joints[0].currentUs));
    double q1 = radians(pulseToDegrees(1, joints[1].currentUs));
    double q2 = radians(pulseToDegrees(2, joints[2].currentUs));
    double q3 = radians(pulseToDegrees(3, joints[3].currentUs));
    double q4 = radians(pulseToDegrees(4, joints[4].currentUs));

    // Arbitrary base/link measurements in mm for standard uncalibrated FK
    const double L1 = 80.0;
    const double L2 = 120.0;
    const double L3 = 120.0;
    const double L4 = 50.0;

    double R = L2 * cos(q1) + L3 * cos(q1 + q2) + L4 * cos(q1 + q2 + q3);
    double x = R * cos(q0);
    double y = R * sin(q0);
    double z = L1 + L2 * sin(q1) + L3 * sin(q1 + q2) + L4 * sin(q1 + q2 + q3);

    int rx = (int)degrees(q4) % 360;
    int ry = (int)degrees(q1 + q2 + q3) % 360;
    int rz = (int)degrees(q0) % 360;

    double calibratedX = x;
    double calibratedY = y;
    double calibratedZ = z;
    transformSerialToReal(calibratedX, calibratedY, calibratedZ);

    char buffer[180];
    snprintf(buffer, sizeof(buffer), "Raw X: %d mm, Y: %d mm, Z: %d mm | Cal X: %d mm, Y: %d mm, Z: %d mm | Rx: %d, Ry: %d, Rz: %d",
             (int)x, (int)y, (int)z,
             (int)round(calibratedX), (int)round(calibratedY), (int)round(calibratedZ),
             rx, ry, rz);

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

    double calibratedX = x;
    double calibratedY = y;
    double calibratedZ = z;
    transformSerialToReal(calibratedX, calibratedY, calibratedZ);

    char buffer[200];
    snprintf(buffer, sizeof(buffer), "Joint %d %s | Raw X: %d mm, Y: %d mm, Z: %d mm | Cal X: %d mm, Y: %d mm, Z: %d mm | Rx: %d, Ry: %d, Rz: %d",
             jointIndex,
             joints[jointIndex].name,
             (int)x,
             (int)y,
             (int)z,
             (int)round(calibratedX),
             (int)round(calibratedY),
             (int)round(calibratedZ),
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
    printStatus();
}

void loop() {
    // Actively print uncalibrated FK tracking string every 500ms (Non-blocking)
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
