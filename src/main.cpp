#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "control_page.h"

// Initialize PCA9685 driver at default I2C address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
WebServer server(80);
Preferences preferences;

const char* AP_NAME = "RobotArm-Control";
const char* AP_PASSWORD = "robotarm";

#define SERVO_FREQ 50 // Servos run at 50Hz (20,000 microseconds per cycle)

// User-adjustable movement speed, hard-limited to 60% for safety.
constexpr int MAX_MOVEMENT_SPEED_PERCENT = 60;
constexpr int STARTUP_ROUTINE_SPEED_PERCENT = 30;
int movementSpeedPercent = 30;

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
// We use 500us as a baseline start for all joints so nothing clips on boot.
CalibJoint joints[] = {
    {"Base",        5, 1500, 640, 2600, 270},
    {"Shoulder",    4, 1850, 840, 1990, 110},
    {"Elbow",       3, 1480, 750, 2600, 155},
    {"Wrist Pitch", 1, 1500, 400, 1850, 140},
    {"Wrist Roll",  2, 1460, 500, 2400, 180},
    {"Gripper",     0, 900,  830, 1500, 60}
};
const int NUM_JOINTS = 6;
int activeJointIndex = 0; // Starts tracking the Base
bool servoOutputEnabled = true;

enum StartupState {
    STATE_INIT,
    STATE_INITIALIZE,
    STATE_READY,
    STATE_HOMING,
    STATE_BUSY,
    STATE_IDLE,
    STATE_FAULT
};
StartupState startupState = STATE_INIT;

void printStartupState(StartupState state) {
    Serial.println();
    Serial.println("-------------------------------------------------------");
    switch (state) {
        case STATE_INIT:
            Serial.println("--------------------- STATE_INIT ----------------------");
            break;
        case STATE_INITIALIZE:
            Serial.println("----------------- STATE_INITIALIZE --------------------");
            break;
        case STATE_READY:
            Serial.println("-------------------- STATE_READY ----------------------");
            break;
        case STATE_HOMING:
            Serial.println("------------------- STATE_HOMING ----------------------");
            break;
        case STATE_BUSY:
            Serial.println("--------------------- STATE_BUSY ----------------------");
            break;
        case STATE_IDLE:
            Serial.println("--------------------- STATE_IDLE ----------------------");
            break;
        case STATE_FAULT:
            Serial.println("-------------------- STATE_FAULT ----------------------");
            break;
    }
    Serial.println("-------------------------------------------------------");
}

void setRobotState(StartupState state) {
    if (startupState == state) {
        return;
    }
    startupState = state;
    printStartupState(startupState);
}

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
int lastPlayedRoutine = -1;
int runningStep = 0;
unsigned long routineHoldStarted = 0;
unsigned long routineLastMove = 0;
bool routinePoseReached = false;
int routineSpeedLimitPercent = MAX_MOVEMENT_SPEED_PERCENT;

#if 0
const char CONTROL_PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Robot Arm Control</title>
  <style>
    :root{color-scheme:dark;--bg:#0b1220;--card:#172033;--accent:#38bdf8;--danger:#fb7185}
    *{box-sizing:border-box} body{margin:0;background:var(--bg);color:#e5eefb;font-family:Arial,sans-serif}
    main{width:min(760px,94%);margin:auto;padding:22px 0 36px}
    h1{margin:0 0 6px;font-size:clamp(1.7rem,6vw,2.5rem)} .sub{color:#9fb0c8;margin:0 0 20px}
    .status,.joint,.speed,.recorder{background:var(--card);border:1px solid #283750;border-radius:16px;padding:16px;margin:12px 0}
    .status{display:flex;justify-content:space-between;align-items:center;gap:12px}
    .dot{width:11px;height:11px;border-radius:50%;display:inline-block;background:#4ade80;margin-right:7px}
    .joint-head{display:flex;justify-content:space-between;font-weight:700;margin-bottom:12px}
    output{color:var(--accent);font-variant-numeric:tabular-nums}
    input[type=range]{width:100%;height:36px;accent-color:var(--accent)}
    .buttons{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin-top:18px}
    button{min-height:52px;border:0;border-radius:13px;background:#263650;color:white;font-weight:700;font-size:1rem}
    button:active{transform:scale(.98)} .primary{background:#0284c7}.danger{background:#be123c}
    .recording{background:#dc2626}.routine{display:grid;grid-template-columns:1fr auto;gap:8px;margin-top:8px}
    input[type=number],input[type=text]{width:100%;padding:12px;border-radius:10px;border:1px solid #3b4b65;background:#0f172a;color:white;font-size:1rem}
    small{display:block;text-align:center;color:#8293aa;margin-top:18px}
    @media(max-width:520px){.buttons{grid-template-columns:1fr}}
  </style>
</head>
<body><main>
  <h1>Robot Arm</h1>
  <p class="sub">Move one joint at a time. Changes are sent when you release a slider.</p>
  <div class="status"><span><i class="dot" id="dot"></i><b id="state">Arm enabled</b></span><span id="message">Ready</span></div>
  <div class="speed">
    <div class="joint-head"><label for="speed">Movement speed</label><output id="speedValue">30%</output></div>
    <input id="speed" type="range" min="1" max="60" step="1" value="30">
  </div>
  <section id="joints"></section>
  <div class="recorder">
    <div class="joint-head"><span>Recorded tasks</span><span id="recordState">Not recording</span></div>
    <p class="sub">Start recording, move the sliders through each task position, then save it.</p>
    <div class="buttons">
      <button id="recordButton" class="primary" onclick="startRecording()">Start recording</button>
      <button onclick="saveRecording()">Stop & save</button>
      <button class="danger" onclick="action('stop')">Stop playback</button>
    </div>
    <label>Playback repeats</label>
    <input id="repeats" type="number" min="1" max="50" value="1">
    <div id="routines"></div>
  </div>
  <div class="buttons">
    <button class="primary" onclick="action('home')">Home position</button>
    <button onclick="action('enable')">Enable output</button>
    <button class="danger" onclick="action('disable')">Disable output</button>
  </div>
  <small>Connect to RobotArm-Control · Open 192.168.4.1</small>
</main>
<script>
const names=['Base','Shoulder','Elbow','Wrist Pitch','Wrist Roll','Gripper'];
const starts=[1520,1830,2380,1460,500,1470];
const mins=[640,840,750,400,500,830];
const maxs=[2600,1990,2600,1850,2400,1500];
const degrees=[270,110,183,140,180,60];
const box=document.getElementById('joints');
const speed=document.getElementById('speed'),speedValue=document.getElementById('speedValue');
speed.oninput=()=>speedValue.textContent=`${speed.value}%`;
speed.onchange=()=>send(`/speed?value=${speed.value}`);
names.forEach((name,i)=>{
  box.insertAdjacentHTML('beforeend',`<div class="joint"><div class="joint-head"><label for="j${i}">${name} (0-${degrees[i]}°)</label><output id="v${i}">${starts[i]} µs</output></div><input id="j${i}" type="range" min="${mins[i]}" max="${maxs[i]}" step="2" value="${starts[i]}"></div>`);
  const slider=document.getElementById(`j${i}`), value=document.getElementById(`v${i}`);
  slider.oninput=()=>value.textContent=`${slider.value} µs`;
  slider.onchange=()=>send(`/set?joint=${i}&us=${slider.value}`);
});
async function send(url){
  const m=document.getElementById('message'); m.textContent='Sending...';
  try{const r=await fetch(url);m.textContent=await r.text()}catch(e){m.textContent='Connection lost'}
}
async function action(name){
  await send(`/action?name=${name}`);
  const enabled=name!=='disable';
  if(name==='disable'||name==='enable'){
    document.getElementById('state').textContent=enabled?'Arm enabled':'Output disabled';
    document.getElementById('dot').style.background=enabled?'#4ade80':'#fb7185';
  }
  if(name==='home') location.reload();
}
async function startRecording(){
  await send('/record?action=start');
  document.getElementById('recordState').textContent='Recording movements';
  document.getElementById('recordButton').classList.add('recording');
}
async function saveRecording(){
  const name=prompt('Name this task (example: Pick and place)');
  if(!name)return;
  await send(`/record?action=save&name=${encodeURIComponent(name)}`);
  document.getElementById('recordState').textContent='Saved';
  document.getElementById('recordButton').classList.remove('recording');
  loadRoutines();
}
async function playRoutine(id){
  const repeats=Math.max(1,Math.min(50,Number(document.getElementById('repeats').value)||1));
  await send(`/play?id=${id}&repeat=${repeats}`);
}
async function deleteRoutine(id){
  if(!confirm('Delete this saved task?'))return;
  await send(`/delete?id=${id}`); loadRoutines();
}
async function loadRoutines(){
  const data=await (await fetch('/routines')).json(), list=document.getElementById('routines');
  list.innerHTML=data.length?'':'<p class="sub">No saved tasks yet.</p>';
  data.forEach(r=>list.insertAdjacentHTML('beforeend',`<div class="routine"><button onclick="playRoutine(${r.id})">${r.name} (${r.steps} steps)</button><button class="danger" onclick="deleteRoutine(${r.id})">Delete</button></div>`));
}
loadRoutines();
</script></body></html>
)HTML";
#endif

// Pick-and-place demonstration poses, ordered:
// Base, Shoulder, Elbow, Wrist Pitch, Wrist Roll, Gripper.
// Adjust these pulse widths with the calibration controls before using a load.
// Power-on standing/home position:
// Base 1500, Shoulder 1850, Elbow 1480, Wrist Pitch 1500,
// Wrist Roll 1460, Gripper 900 microseconds.
const int HOME_POSE[NUM_JOINTS]  = {1500, 1850, 1480, 1500, 1460, 900};
const int PICK_UP[NUM_JOINTS]    = {1100, 1650, 1700, 1500, 500, 1200};
const int PICK_DOWN[NUM_JOINTS]  = {1100, 2050, 2150, 1750, 500, 1200};
const int GRAB_POSE[NUM_JOINTS]  = {1100, 2050, 2150, 1750, 500, 1850};
const int LIFT_POSE[NUM_JOINTS]  = {1100, 1650, 1700, 1500, 500, 1850};
const int PLACE_UP[NUM_JOINTS]   = {1950, 1650, 1700, 1500, 500, 1850};
const int PLACE_DOWN[NUM_JOINTS] = {1950, 2050, 2150, 1750, 500, 1850};
const int RELEASE_POSE[NUM_JOINTS] =
                                  {1950, 2050, 2150, 1750, 500, 1200};

const int* const PICK_PLACE_POSES[] = {
    PICK_UP, PICK_DOWN, GRAB_POSE, LIFT_POSE,
    PLACE_UP, PLACE_DOWN, RELEASE_POSE, HOME_POSE
};
const unsigned long POSE_HOLD_MS[] = {
    1000, 1200, 800, 1200, 1200, 1200, 800, 1500
};
const unsigned long SERVO_STEP_INTERVAL_MS = 20;
const int NUM_PICK_PLACE_POSES =
    sizeof(PICK_PLACE_POSES) / sizeof(PICK_PLACE_POSES[0]);

bool pickPlaceRunning = false;
int pickPlaceStage = 0;
unsigned long poseStartedAt = 0;
unsigned long lastServoStepAt = 0;
bool poseReached = false;

void printStatus();
int clampJointPulse(int jointIndex, int us);
int pulseToDegrees(int jointIndex, int us);

void printTaskStep(const char* taskName, int stepIndex, int totalSteps) {
    Serial.println();
    printStartupState(STATE_BUSY);
    Serial.println("---------------------- TASK STEP ----------------------");
    Serial.printf("Task: %s\n", taskName);
    Serial.printf("Running step %d of %d\n", stepIndex + 1, totalSteps);
    Serial.println("-------------------------------------------------------");
}

template <typename PulseType>
void printTaskStepTargets(const PulseType pose[]) {
    for (int joint = 0; joint < NUM_JOINTS; joint++) {
        const int targetPulse = clampJointPulse(joint, pose[joint]);
        if (joints[joint].currentUs == targetPulse) {
            continue;
        }
        Serial.printf("Moving %s to %d degrees.\n",
                      joints[joint].name,
                      pulseToDegrees(joint, targetPulse));
    }
}

void setServoPulseUs(uint8_t channel, int us) {
    // Formula: (us / 20000.0) * 4096
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
    const int degrees =
        (offset * joint.maxDegrees + pulseRange / 2) / pulseRange;
    // The elbow is installed in the opposite direction to the other joints.
    return jointIndex == 2 ? joint.maxDegrees - degrees : degrees;
}

int degreesToPulse(int jointIndex, int degrees) {
    const CalibJoint& joint = joints[jointIndex];
    degrees = constrain(degrees, 0, joint.maxDegrees);
    if (jointIndex == 2) {
        degrees = joint.maxDegrees - degrees;
    }
    const long pulseRange = joint.maxUs - joint.minUs;
    return joint.minUs +
           (degrees * pulseRange + joint.maxDegrees / 2) / joint.maxDegrees;
}

String currentDegreesJson() {
    String json = "{\"degrees\":[";
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (i > 0) {
            json += ",";
        }
        json += String(pulseToDegrees(i, joints[i].currentUs));
    }
    json += "]}";
    return json;
}

void moveJointSlowly(int jointIndex, int targetUs) {
    targetUs = clampJointPulse(jointIndex, targetUs);
    int currentUs = joints[jointIndex].currentUs;
    movementSpeedPercent =
        constrain(movementSpeedPercent, 1, MAX_MOVEMENT_SPEED_PERCENT);
    // Speed controls both step size and delay while preserving smooth motion.
    int stepSize = map(movementSpeedPercent, 1, 100, 1, 12);
    int stepDelayMs = map(movementSpeedPercent, 1, 100, 25, 1);
    int direction = targetUs >= currentUs ? stepSize : -stepSize;

    while (currentUs != targetUs) {
        currentUs += direction;

        if ((direction > 0 && currentUs > targetUs) ||
            (direction < 0 && currentUs < targetUs)) {
            currentUs = targetUs;
        }

        joints[jointIndex].currentUs = currentUs;
        setServoPulseUs(joints[jointIndex].channel, currentUs);
        delay(stepDelayMs);
        yield();
    }
}

void moveJointSmoothly(int jointIndex, int targetUs, int speedLimitPercent) {
    targetUs = clampJointPulse(jointIndex, targetUs);
    const int startUs = joints[jointIndex].currentUs;
    const int distanceUs = targetUs - startUs;
    const int absoluteDistanceUs = abs(distanceUs);
    if (absoluteDistanceUs == 0) {
        return;
    }

    // Cubic easing starts and finishes gently. The duration calculation keeps
    // the fastest part of the eased movement within the requested speed cap.
    const int safeSpeed =
        constrain(speedLimitPercent, 1, MAX_MOVEMENT_SPEED_PERCENT);
    const int stepSize = map(safeSpeed, 1, 100, 1, 12);
    const int stepDelayMs = map(safeSpeed, 1, 100, 25, 1);
    const float maximumRateUsPerSecond =
        (stepSize * 1000.0f) / stepDelayMs;
    unsigned long durationMs = (unsigned long)(
        (absoluteDistanceUs * 1500.0f) / maximumRateUsPerSecond);
    durationMs = max(durationMs, 500UL);

    const unsigned long startedAt = millis();
    unsigned long elapsedMs = 0;
    while (elapsedMs < durationMs) {
        const float progress = (float)elapsedMs / durationMs;
        const float eased = progress * progress * (3.0f - 2.0f * progress);
        joints[jointIndex].currentUs =
            startUs + (int)(distanceUs * eased);
        setServoPulseUs(joints[jointIndex].channel,
                        joints[jointIndex].currentUs);
        delay(SERVO_STEP_INTERVAL_MS);
        yield();
        elapsedMs = millis() - startedAt;
    }

    joints[jointIndex].currentUs = targetUs;
    setServoPulseUs(joints[jointIndex].channel, targetUs);
}

bool isAtHomePosition() {
    for (int joint = 0; joint < NUM_JOINTS; joint++) {
        if (joints[joint].currentUs != HOME_POSE[joint]) {
            return false;
        }
    }
    return true;
}

bool isI2cDevicePresent(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

bool performStartupSelfAudit() {
    Serial.println("Running startup self-audit...");

    // The PCA9685 ACK confirms that its logic supply and I2C connection are
    // available. Verifying servo-rail voltage requires a divided voltage
    // signal connected to an ESP32 ADC pin; no such input is configured here.
    if (!isI2cDevicePresent(0x40)) {
        Serial.println(
            "FAULT: PCA9685 not detected. Check controller power and I2C wiring.");
        return false;
    }

    Serial.println("PASS: PCA9685 motor/servo controller detected.");
    Serial.println(
        "NOTE: Sensor/servo supply voltage requires dedicated ADC feedback.");
    return true;
}

bool testAllServosAndConfirm() {
    constexpr int TEST_OFFSET_US = 100;
    constexpr int TEST_SPEED_PERCENT = 20;

    setRobotState(STATE_INITIALIZE);
    Serial.println("Testing every servo with a small movement...");
    servoOutputEnabled = true;

    for (int joint = 0; joint < NUM_JOINTS; joint++) {
        const int startPulse = joints[joint].currentUs;
        const int firstTestPulse =
            clampJointPulse(joint, startPulse + TEST_OFFSET_US);
        const int secondTestPulse =
            clampJointPulse(joint, startPulse - TEST_OFFSET_US);

        Serial.printf("Testing %s.\n", joints[joint].name);
        Serial.printf("Moving %s to %d degrees.\n",
                      joints[joint].name,
                      pulseToDegrees(joint, firstTestPulse));
        moveJointSmoothly(joint, firstTestPulse, TEST_SPEED_PERCENT);
        delay(350);

        Serial.printf("Moving %s to %d degrees.\n",
                      joints[joint].name,
                      pulseToDegrees(joint, secondTestPulse));
        moveJointSmoothly(joint, secondTestPulse, TEST_SPEED_PERCENT);
        delay(350);
    }

    while (Serial.available() > 0) {
        Serial.read();
    }

    Serial.println();
    Serial.println("Servo movement test complete.");
    Serial.println("Did all servos move correctly?");
    Serial.println("Enter Y to continue to Home.");

    while (true) {
        if (Serial.available() == 0) {
            delay(20);
            yield();
            continue;
        }

        const char response = Serial.read();
        if (response == 'y' || response == 'Y') {
            Serial.println("Servo test confirmed. Starting homing.");
            return true;
        }

        if (response == '\r' || response == '\n') {
            continue;
        }

        Serial.println("Startup paused. Enter Y after confirming all servos.");
    }
}

void returnHomeOneJointAtATime() {
    if (isAtHomePosition()) {
        Serial.println("Arm is already at home; no movement required.");
        return;
    }

    // Move one complete joint at a time in a clearance-first order.
    const int homingOrder[NUM_JOINTS] = {5, 3, 4, 1, 2, 0};
    servoOutputEnabled = true;

    for (int i = 0; i < NUM_JOINTS; i++) {
        const int joint = homingOrder[i];
        if (joints[joint].currentUs == HOME_POSE[joint]) {
            Serial.printf("%s is already home; skipping.\n",
                          joints[joint].name);
            continue;
        }
        Serial.printf("Homing %s (%d/%d)\n",
                      joints[joint].name, i + 1, NUM_JOINTS);
        Serial.printf("Moving %s to %d degrees.\n",
                      joints[joint].name,
                      pulseToDegrees(joint, HOME_POSE[joint]));
        moveJointSlowly(joint, HOME_POSE[joint]);
        delay(250);
    }
}

void homeRobot() {
    // Ready-pose sequence required at every power-up:
    // Wrist -> Elbow -> Shoulder -> Base. Both wrist axes are moved before
    // the arm's larger load-bearing joints.
    const int homingOrder[] = {3, 4, 2, 1, 0};
    const int homingJointCount =
        sizeof(homingOrder) / sizeof(homingOrder[0]);

    servoOutputEnabled = true;
    Serial.println("Moving to Ready Pose...");
    for (int i = 0; i < homingJointCount; i++) {
        const int joint = homingOrder[i];
        Serial.printf("Homing %s (%d/%d)\n",
                      joints[joint].name, i + 1, homingJointCount);
        Serial.printf("Moving %s to %d degrees.\n",
                      joints[joint].name,
                      pulseToDegrees(joint, HOME_POSE[joint]));
        // Always issue the command at startup. The controller cannot infer
        // physical servo position from the last pulse stored in RAM.
        moveJointSmoothly(
            joint, HOME_POSE[joint], STARTUP_ROUTINE_SPEED_PERCENT);
        delay(250);
    }

    // The gripper is not part of the articulated homing order, but it must be
    // commanded to its defined Ready value before startup is complete.
    Serial.printf("Moving %s to %d degrees.\n",
                  joints[5].name,
                  pulseToDegrees(5, HOME_POSE[5]));
    joints[5].currentUs = HOME_POSE[5];
    setServoPulseUs(joints[5].channel, HOME_POSE[5]);
    delay(500);
    Serial.println("Autonomous homing complete; Ready Pose reached.");
}

void performStartupHoming() {
    // Homing is deliberately blocking. The web server and serial command
    // handling are not started until the arm has reached its safe coordinate.
    Serial.println("Starting one-joint-at-a-time software homing...");
    returnHomeOneJointAtATime();

    // Explicitly command and record every exact Home value before allowing
    // automatic task playback, including joints that were already at Home.
    for (int joint = 0; joint < NUM_JOINTS; joint++) {
        joints[joint].currentUs = HOME_POSE[joint];
        setServoPulseUs(joints[joint].channel, HOME_POSE[joint]);
    }
    delay(1000);
    Serial.println("Software homing complete.");
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

void saveRoutines() {
    preferences.putBytes("routines", routines, sizeof(routines));
}

void captureRoutinePose() {
    if (!recording || recordingBuffer.stepCount >= MAX_ROUTINE_STEPS) {
        return;
    }

    if (recordingBuffer.stepCount > 0) {
        const int lastStep = recordingBuffer.stepCount - 1;
        bool unchanged = true;
        for (int joint = 0; joint < NUM_JOINTS; joint++) {
            if (recordingBuffer.poses[lastStep][joint] !=
                joints[joint].currentUs) {
                unchanged = false;
                break;
            }
        }
        if (unchanged) {
            return;
        }
    }

    int step = recordingBuffer.stepCount;
    for (int joint = 0; joint < NUM_JOINTS; joint++) {
        recordingBuffer.poses[step][joint] = joints[joint].currentUs;
    }
    recordingBuffer.stepCount++;
}

void updateRoutinePlayback() {
    if (!routineRunning || runningRoutine < 0) {
        return;
    }

    SavedRoutine& routine = routines[runningRoutine];
    unsigned long now = millis();
    if (!routinePoseReached) {
        const int safeSpeed = constrain(
            min(movementSpeedPercent, routineSpeedLimitPercent),
            1, MAX_MOVEMENT_SPEED_PERCENT);
        const unsigned long moveIntervalMs =
            map(safeSpeed, 1, 100, 25, 1);
        if (now - routineLastMove < moveIntervalMs) {
            return;
        }
        routineLastMove = now;

        bool reached = true;
        int stepSize = map(safeSpeed, 1, 100, 1, 12);
        for (int joint = 0; joint < NUM_JOINTS; joint++) {
            int safeTarget =
                clampJointPulse(joint, routine.poses[runningStep][joint]);
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

    if (now - routineHoldStarted < 500) {
        return;
    }

    runningStep++;
    if (runningStep >= routine.stepCount) {
        routineRunning = false;
        runningRoutine = -1;
        runningStep = 0;
        routineSpeedLimitPercent = MAX_MOVEMENT_SPEED_PERCENT;
        Serial.println("Saved task complete; returning home.");
        setRobotState(STATE_HOMING);
        returnHomeOneJointAtATime();
        Serial.println("Arm returned home; all playback stopped.");
        setRobotState(STATE_IDLE);
        return;
    }
    printTaskStep(routine.name, runningStep, routine.stepCount);
    printTaskStepTargets(routine.poses[runningStep]);
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
        server.send(200, "application/json", currentDegreesJson());
    });

    server.on("/set", HTTP_GET, []() {
        if (!server.hasArg("joint") || !server.hasArg("degree")) {
            server.send(400, "text/plain", "Missing joint or degree value");
            return;
        }

        int joint = server.arg("joint").toInt();
        if (joint < 0 || joint >= NUM_JOINTS) {
            server.send(400, "text/plain", "Invalid joint");
            return;
        }
        int degrees =
            constrain(server.arg("degree").toInt(), 0, joints[joint].maxDegrees);
        int pulse = degreesToPulse(joint, degrees);

        activeJointIndex = joint;
        setRobotState(STATE_BUSY);
        Serial.printf("UI command: moving %s to %d degrees.\n",
                      joints[joint].name, degrees);
        moveJointSlowly(joint, pulse);
        setRobotState(STATE_IDLE);
        server.send(200, "text/plain",
                    String(joints[joint].name) + ": " + degrees + " deg");
    });

    server.on("/jog", HTTP_GET, []() {
        if (!server.hasArg("j1") || !server.hasArg("d1")) {
            server.send(400, "application/json",
                        "{\"error\":\"Missing joystick movement\"}");
            return;
        }

        const int firstJoint = server.arg("j1").toInt();
        if (firstJoint < 0 || firstJoint >= NUM_JOINTS) {
            server.send(400, "application/json",
                        "{\"error\":\"Invalid joint\"}");
            return;
        }

        const int firstDegrees = constrain(
            pulseToDegrees(firstJoint, joints[firstJoint].currentUs) +
                server.arg("d1").toInt(),
            0, joints[firstJoint].maxDegrees);
        activeJointIndex = firstJoint;
        setRobotState(STATE_BUSY);
        Serial.println("UI joystick command received.");
        moveJointSlowly(firstJoint,
                        degreesToPulse(firstJoint, firstDegrees));

        if (server.hasArg("j2") && server.hasArg("d2")) {
            const int secondJoint = server.arg("j2").toInt();
            if (secondJoint >= 0 && secondJoint < NUM_JOINTS) {
                const int secondDegrees = constrain(
                    pulseToDegrees(secondJoint,
                                   joints[secondJoint].currentUs) +
                        server.arg("d2").toInt(),
                    0, joints[secondJoint].maxDegrees);
                moveJointSlowly(secondJoint,
                                degreesToPulse(secondJoint, secondDegrees));
            }
        }

        setRobotState(STATE_IDLE);
        server.send(200, "application/json", currentDegreesJson());
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
                        previousCount >= MAX_ROUTINE_STEPS
                            ? "Maximum 30 steps reached"
                            : "This position is already the last step");
            return;
        }
        server.send(200, "text/plain",
                    "Step " + String(recordingBuffer.stepCount) + " added");
    });

    server.on("/speed", HTTP_GET, []() {
        if (!server.hasArg("value")) {
            server.send(400, "text/plain", "Missing speed value");
            return;
        }

        movementSpeedPercent = constrain(
            server.arg("value").toInt(), 1, MAX_MOVEMENT_SPEED_PERCENT);
        server.send(200, "text/plain",
                    "Speed set to " + String(movementSpeedPercent) + "%");
    });

    server.on("/action", HTTP_GET, []() {
        String action = server.arg("name");
        if (action == "disable") {
            setAllOutputs(false);
            server.send(200, "text/plain", "Servo output disabled");
        } else if (action == "enable") {
            setAllOutputs(true);
            server.send(200, "text/plain", "Servo output enabled");
        } else if (action == "home") {
            // Home is a terminal command: cancel all automatic movement first
            // so no routine or demonstration resumes after reaching home.
            routineRunning = false;
            runningRoutine = -1;
            runningStep = 0;
            routineSpeedLimitPercent = MAX_MOVEMENT_SPEED_PERCENT;
            pickPlaceRunning = false;
            setRobotState(STATE_HOMING);
            returnHomeOneJointAtATime();
            setRobotState(STATE_IDLE);
            server.send(200, "text/plain",
                        "Arm returned home; all playback stopped");
        } else if (action == "stop") {
            routineRunning = false;
            routineSpeedLimitPercent = MAX_MOVEMENT_SPEED_PERCENT;
            pickPlaceRunning = false;
            setRobotState(STATE_IDLE);
            server.send(200, "text/plain", "Playback stopped");
        } else {
            server.send(400, "text/plain", "Unknown action");
        }
    });

    server.on("/record", HTTP_GET, []() {
        String action = server.arg("action");
        if (action == "start") {
            routineRunning = false;
            routineSpeedLimitPercent = MAX_MOVEMENT_SPEED_PERCENT;
            pickPlaceRunning = false;
            memset(&recordingBuffer, 0, sizeof(recordingBuffer));
            recording = true;
            server.send(200, "text/plain",
                        "New task started; set a position and add a step");
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
                server.send(400, "text/plain",
                            "Add at least one step before saving");
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
            if (name.length() == 0) name = "Saved task";
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
            if (routines[i].stepCount == 0) continue;
            if (!first) json += ",";
            first = false;
            String safeName = routines[i].name;
            safeName.replace("\\", "\\\\");
            safeName.replace("\"", "\\\"");
            json += "{\"id\":" + String(i) + ",\"name\":\"" + safeName +
                    "\",\"steps\":" + String(routines[i].stepCount) + "}";
        }
        json += "]";
        server.send(200, "application/json", json);
    });

    server.on("/play", HTTP_GET, []() {
        int id = server.arg("id").toInt();
        if (id < 0 || id >= MAX_ROUTINES || routines[id].stepCount == 0) {
            server.send(404, "text/plain", "Task not found");
            return;
        }
        recording = false;
        pickPlaceRunning = false;
        servoOutputEnabled = true;
        runningRoutine = id;
        lastPlayedRoutine = id;
        runningStep = 0;
        routineSpeedLimitPercent = MAX_MOVEMENT_SPEED_PERCENT;
        routinePoseReached = false;
        routineLastMove = millis();
        routineRunning = true;
        setRobotState(STATE_BUSY);
        printTaskStep(routines[id].name, runningStep,
                      routines[id].stepCount);
        printTaskStepTargets(routines[id].poses[runningStep]);
        server.send(200, "text/plain",
                    "Playing " + String(routines[id].name) + " once");
    });

    server.on("/delete", HTTP_GET, []() {
        if (!server.hasArg("id")) {
            server.send(400, "text/plain", "Missing task id");
            return;
        }
        int id = server.arg("id").toInt();
        if (id < 0 || id >= MAX_ROUTINES || routines[id].stepCount == 0) {
            server.send(400, "text/plain", "Invalid task");
            return;
        }
        if (runningRoutine == id) {
            routineRunning = false;
            runningRoutine = -1;
        }
        if (lastPlayedRoutine == id) {
            lastPlayedRoutine = -1;
        }
        memset(&routines[id], 0, sizeof(SavedRoutine));
        saveRoutines();
        server.send(200, "text/plain", "Task deleted");
    });

    server.onNotFound([]() {
        server.send(404, "text/plain", "Not found");
    });
    server.begin();
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
    setRobotState(STATE_BUSY);
    Serial.println("\nPick-and-place demonstration started.");
    printTaskStep("Pick-and-place demonstration", pickPlaceStage,
                  NUM_PICK_PLACE_POSES);
    printTaskStepTargets(PICK_PLACE_POSES[pickPlaceStage]);
}

bool startFirstSavedRoutine() {
    for (int id = 0; id < MAX_ROUTINES; id++) {
        if (routines[id].stepCount == 0) {
            continue;
        }

        recording = false;
        pickPlaceRunning = false;
        servoOutputEnabled = true;
        runningRoutine = id;
        lastPlayedRoutine = id;
        runningStep = 0;
        Serial.printf("Automatically playing saved task: %s\n",
                      routines[id].name);
        setRobotState(STATE_BUSY);
        printTaskStep(routines[id].name, runningStep,
                      routines[id].stepCount);
        printTaskStepTargets(routines[id].poses[runningStep]);

        // Let the arm settle at Home, then approach the first pose one joint
        // at a time with eased acceleration and deceleration.
        delay(1000);
        for (int joint = 0; joint < NUM_JOINTS; joint++) {
            moveJointSmoothly(
                joint,
                routines[id].poses[0][joint],
                STARTUP_ROUTINE_SPEED_PERCENT);
            delay(250);
        }

        // Keep every remaining step of the automatically started task at the
        // same low speed, not only the initial Home-to-first-pose transition.
        routineSpeedLimitPercent = STARTUP_ROUTINE_SPEED_PERCENT;
        routinePoseReached = true;
        routineHoldStarted = millis();
        routineLastMove = millis();
        routineRunning = true;
        return true;
    }

    Serial.println("No saved task available for automatic playback.");
    return false;
}

void updatePickAndPlace() {
    if (!pickPlaceRunning) {
        return;
    }

    const unsigned long now = millis();

    if (!poseReached) {
        if (now - lastServoStepAt < SERVO_STEP_INTERVAL_MS) {
            return;
        }
        lastServoStepAt = now;

        bool allAtTarget = true;
        const int safeSpeed =
            constrain(movementSpeedPercent, 1, MAX_MOVEMENT_SPEED_PERCENT);
        const int servoStepUs = map(safeSpeed, 1, 100, 1, 12);
        const int* target = PICK_PLACE_POSES[pickPlaceStage];
        for (int i = 0; i < NUM_JOINTS; i++) {
            const int safeTarget = clampJointPulse(i, target[i]);
            const int difference = safeTarget - joints[i].currentUs;
            if (difference != 0) {
                const int step = constrain(
                    difference, -servoStepUs, servoStepUs);
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

    if (now - poseStartedAt < POSE_HOLD_MS[pickPlaceStage]) {
        return;
    }

    pickPlaceStage++;
    if (pickPlaceStage >= NUM_PICK_PLACE_POSES) {
        pickPlaceRunning = false;
        Serial.println("Pick-and-place demonstration complete.");
        setRobotState(STATE_IDLE);
        printStatus();
        return;
    }

    printTaskStep("Pick-and-place demonstration", pickPlaceStage,
                  NUM_PICK_PLACE_POSES);
    printTaskStepTargets(PICK_PLACE_POSES[pickPlaceStage]);
    poseReached = false;
    lastServoStepAt = now;
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
    Serial.println("  [P] Run pick-and-place demonstration");
    Serial.println("  [T] Repeat the last saved task");
    Serial.println("  [Q] Stop demonstration and return home");
    Serial.println("----------------------------------------------------");
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    preferences.begin("robot-arm", false);
    if (preferences.getBytesLength("routines") == sizeof(routines)) {
        preferences.getBytes("routines", routines, sizeof(routines));
        bool repairedRoutines = false;
        for (int i = 0; i < MAX_ROUTINES; i++) {
            if (routines[i].stepCount > MAX_ROUTINE_STEPS) {
                memset(&routines[i], 0, sizeof(SavedRoutine));
                repairedRoutines = true;
            } else {
                routines[i].name[20] = '\0';
            }
        }
        if (repairedRoutines) {
            saveRoutines();
        }
    }
    
    Wire.begin(21, 22); // ESP32 Pins: SDA=21, SCL=22
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);

    startupState = STATE_INIT;
    printStartupState(startupState);
    if (!performStartupSelfAudit()) {
        setRobotState(STATE_FAULT);
        setAllOutputs(false);
        Serial.println("Startup stopped; servo outputs are disabled.");
        return;
    }

    if (!testAllServosAndConfirm()) {
        return;
    }

    setRobotState(STATE_HOMING);
    homeRobot();
    setRobotState(STATE_READY);
    Serial.println("Startup protocol complete.");

    setupWebUI();
    Serial.printf("\nWi-Fi UI: connect to \"%s\" using password \"%s\"\n", AP_NAME, AP_PASSWORD);
    Serial.print("Then open http://");
    Serial.println(WiFi.softAPIP());
    printStatus();
    if (!startFirstSavedRoutine()) {
        setRobotState(STATE_IDLE);
    }
}

void loop() {
    if (startupState == STATE_FAULT) {
        delay(100);
        return;
    }

    server.handleClient();

    updatePickAndPlace();
    updateRoutinePlayback();

    if (Serial.available() > 0) {
        char c = Serial.read();
        int step = 0;

        if (c == 'p' || c == 'P') {
            startPickAndPlace();
            return;
        }

        if (c == 't' || c == 'T') {
            if (routineRunning || pickPlaceRunning) {
                Serial.println("Cannot repeat task: robot is currently busy.");
                return;
            }
            if (lastPlayedRoutine < 0 ||
                lastPlayedRoutine >= MAX_ROUTINES ||
                routines[lastPlayedRoutine].stepCount == 0) {
                Serial.println("Cannot repeat task: no previous saved task.");
                return;
            }

            const int id = lastPlayedRoutine;
            recording = false;
            servoOutputEnabled = true;
            runningRoutine = id;
            runningStep = 0;
            routineSpeedLimitPercent = MAX_MOVEMENT_SPEED_PERCENT;
            routinePoseReached = false;
            routineLastMove = millis();
            routineRunning = true;
            setRobotState(STATE_HOMING);
            Serial.printf("Repeating saved task: %s\n", routines[id].name);
            printTaskStep(routines[id].name, runningStep,
                          routines[id].stepCount);
            printTaskStepTargets(routines[id].poses[runningStep]);
            return;
        }

        if (c == 'q' || c == 'Q') {
            pickPlaceRunning = false;
            routineRunning = false;
            setRobotState(STATE_BUSY);
            moveToPose(HOME_POSE);
            setRobotState(STATE_IDLE);
            Serial.println("Demonstration stopped; arm returned home.");
            return;
        }

        // Ignore manual calibration commands while the sequence is moving.
        if (pickPlaceRunning) {
            return;
        }
        
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
        targetUs = clampJointPulse(activeJointIndex, targetUs);
        moveJointSlowly(activeJointIndex, targetUs);
        
        // Print live feedback tracking data
        Serial.printf("Joint %s -> %d us\n", joints[activeJointIndex].name, joints[activeJointIndex].currentUs);
    }
}
