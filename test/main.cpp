// Arduino core, I2C communication, PCA9685 servo driver, and ESP32 flash storage.
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <LittleFS.h>

// PCA9685 board I2C address. The default address is normally 0x40.
Adafruit_PWMServoDriver pwm(0x40);

// Servo signal configuration.
constexpr uint8_t NUM_SERVOS = 6;
constexpr uint16_t SERVO_FREQ = 50;
constexpr int USMIN = 500;   // Pulse width for the minimum servo angle.
constexpr int USMAX = 2500;  // Pulse width for the maximum servo angle.

// Each name below corresponds to a PCA9685 output channel.
enum ServoIndex : uint8_t {
  SERVO_BASE = 0,  // Channel 0: rotates the complete arm (0-270 degrees).
  SERVO_SHOULDER,  // Channel 1: moves the upper arm.
  SERVO_ELBOW,     // Channel 2: bends the elbow.
  SERVO_WRIST_P,   // Channel 3: tilts the wrist up/down.
  SERVO_WRIST_R,   // Channel 4: rotates the wrist.
  SERVO_GRIPPER    // Channel 5: opens/closes the gripper.
};

// One recorded movement contains six target angles and the time allowed
// for the servos to reach those angles.
struct Pose {
  // Angle order: base, shoulder, elbow, wrist pitch, wrist roll, gripper.
  int angle[NUM_SERVOS];
  uint32_t durationMs;
};

// Starting/home position used when the ESP32 starts.
Pose currentPose = {{135, 90, 90, 90, 90, 0}, 1000};
File recordingFile;       // File currently receiving recorded movements.
bool isRecording = false; // True between RECORD and STOP commands.
String inputLine;         // Command being received from Serial Monitor.

bool savePose(const Pose &pose);

// Convert a user name such as "pen transfer" into "/seq_pen_transfer.csv".
String sequencePath(String name) {
  name.trim();
  name.replace(" ", "_");
  name.replace("/", "_");
  name.replace("\\", "_");
  return "/seq_" + name + ".csv";
}

void moveServo(uint8_t channel, int angle) {
  // The base servo supports 270 degrees; all other servos support 180.
  const int maxAngle = (channel == SERVO_BASE) ? 270 : 180;
  angle = constrain(angle, 0, maxAngle);

  // Convert the requested angle into a pulse width, then convert that pulse
  // width into the PCA9685 driver's 0-4095 tick range.
  const uint16_t pulseUs = map(angle, 0, maxAngle, USMIN, USMAX);
  const uint16_t ticks =
      static_cast<uint16_t>((pulseUs / 1000000.0) * SERVO_FREQ * 4096.0);
  pwm.setPWM(channel, 0, ticks);
}

void moveToPose(const Pose &pose) {
  // Send all six target angles. The servos begin moving at nearly the same time.
  for (uint8_t channel = 0; channel < NUM_SERVOS; ++channel) {
    moveServo(channel, pose.angle[channel]);
    currentPose.angle[channel] = pose.angle[channel];
  }
  // Wait before allowing the next pose to start.
  currentPose.durationMs = pose.durationMs;
  delay(pose.durationMs);
}

// Use this function for a normal movement. It records the pose when recording
// is active, and then physically moves the arm.
void executePose(const Pose &pose) {
  savePose(pose);
  moveToPose(pose);
}

bool savePose(const Pose &pose) {
  // Do nothing unless a RECORD command has opened a file.
  if (!isRecording || !recordingFile) return false;

  // File line format:
  // base,shoulder,elbow,wristPitch,wristRoll,gripper,durationMs
  // Example: 190,105,120,105,90,0,1200
  for (uint8_t i = 0; i < NUM_SERVOS; ++i) {
    recordingFile.print(pose.angle[i]);
    recordingFile.print(',');
  }
  recordingFile.println(pose.durationMs);
  // Immediately write buffered data to flash to reduce data loss after reset.
  recordingFile.flush();
  return true;
}

bool parsePose(String line, Pose &pose) {
  // Read six comma-separated angles from one saved CSV line.
  line.trim();
  for (uint8_t i = 0; i < NUM_SERVOS; ++i) {
    const int comma = line.indexOf(',');
    if (comma < 0) return false;
    pose.angle[i] = line.substring(0, comma).toInt();
    line.remove(0, comma + 1);
  }
  // The final value after the sixth comma is the movement duration.
  pose.durationMs = line.toInt();
  return pose.durationMs > 0;
}

bool playSequence(const String &path) {
  // Open one sequence file and execute each line in order.
  // Check first because opening a missing file in read mode also prints a
  // confusing LittleFS "no permits for creation" system error.
  if (!LittleFS.exists(path)) {
    Serial.println("Sequence does not exist: " + path);
    Serial.println("Create it with RECORD name, run movements, then STOP.");
    return false;
  }

  File file = LittleFS.open(path, FILE_READ);
  if (!file || file.isDirectory()) {
    Serial.println("Could not open sequence: " + path);
    return false;
  }

  Serial.println("Playing " + path);
  while (file.available()) {
    Pose pose;
    // Invalid lines are ignored instead of moving the arm with bad data.
    if (parsePose(file.readStringUntil('\n'), pose)) moveToPose(pose);
  }
  file.close();
  Serial.println("Finished " + path);
  return true;
}

void playAllSequences() {
  // Search the LittleFS root folder and play every seq_*.csv file.
  File root = LittleFS.open("/");
  File file = root.openNextFile();
  bool found = false;

  while (file) {
    const String path = String("/") + file.name();
    const bool isSequence = !file.isDirectory() &&
                            path.startsWith("/seq_") &&
                            path.endsWith(".csv");
    file.close();
    if (isSequence) {
      found = true;
      playSequence(path);
    }
    file = root.openNextFile();
  }
  root.close();
  if (!found) Serial.println("No recorded sequences.");
}

void listSequences() {
  // Show every saved sequence and its file size in Serial Monitor.
  File root = LittleFS.open("/");
  File file = root.openNextFile();
  Serial.println("Recorded sequences:");
  bool found = false;

  while (file) {
    String path = String("/") + file.name();
    if (!file.isDirectory() && path.startsWith("/seq_") && path.endsWith(".csv")) {
      found = true;
      Serial.printf("  %s (%u bytes)\n", path.c_str(),
                    static_cast<unsigned>(file.size()));
    }
    file.close();
    file = root.openNextFile();
  }
  root.close();
  if (!found) Serial.println("  none");
}

void printHelp() {
  // Available commands are printed at startup and after HELP.
  Serial.println("\nCommands (send with newline):");
  Serial.println("  RECORD name");
  Serial.println("  MOVE base shoulder elbow wristPitch wristRoll gripper durationMs");
  Serial.println("  STOP");
  Serial.println("  PLAY name");
  Serial.println("  PLAYALL");
  Serial.println("  LIST");
  Serial.println("  DELETE name");
  Serial.println("  HOME");
  Serial.println("  PEN             (pick pen from floor and move it to the other side)");
  Serial.println("  HELP");
}

void transferPen() {
  // IMPORTANT: These angles are example positions. Adjust them for the exact
  // arm height, floor level, pen location, and servo mounting direction.
  //
  // Each row has this format:
  // {{base, shoulder, elbow, wristPitch, wristRoll, gripper}, durationMs}
  //
  // Gripper value used here: 0 = open and 100 = closed.
  // This arm's wrist-pitch servo uses lower angles to point the gripper down.
  // The arm always lifts the pen before rotating to avoid dragging it.
  const Pose sequence[] = {
      {{135, 90, 90, 90, 90, 0}, 1200},     // 1. Start at a safe home position.
      {{190, 105, 120, 75, 90, 0}, 1200},   // 2. Move above the pen, gripper facing down.
      {{190, 130, 155, 55, 90, 0}, 1200},   // 3. Lower the open gripper to floor level.
      {{190, 130, 155, 55, 90, 100}, 700},  // 4. Close the gripper around the pen.
      {{190, 100, 115, 80, 90, 100}, 1200}, // 5. Lift the pen clear of the floor.
      {{80, 100, 115, 80, 90, 100}, 1500},  // 6. Rotate the arm to the other side.
      {{80, 130, 155, 55, 90, 100}, 1200},  // 7. Lower the pen to the floor.
      {{80, 130, 155, 55, 90, 0}, 700},     // 8. Open the gripper to release it.
      {{80, 100, 115, 80, 90, 0}, 1000},    // 9. Lift the empty gripper away.
      {{135, 90, 90, 90, 90, 0}, 1500}      // 10. Return to the home position.
  };

  Serial.println("Starting floor pen transfer...");
  // Execute every pose from the array in order.
  for (const Pose &pose : sequence) executePose(pose);
  Serial.println("Pen transfer complete.");
}

void handleCommand(String command) {
  // Remove newline/space characters and keep an uppercase copy so commands
  // such as "pen", "PEN", and "Pen" are treated the same.
  command.trim();
  if (command.isEmpty()) return;

  String upper = command;
  upper.toUpperCase();

  if (upper.startsWith("RECORD ")) {
    // RECORD demo creates/replaces /seq_demo.csv.
    if (isRecording) {
      Serial.println("STOP the current recording first.");
      return;
    }
    const String path = sequencePath(command.substring(7));
    recordingFile = LittleFS.open(path, FILE_WRITE);
    if (!recordingFile) {
      Serial.println("Could not create " + path);
      return;
    }
    isRecording = true;
    Serial.println("Recording to " + path);
  } else if (upper.startsWith("MOVE ")) {
    // Read the seven numbers after MOVE into a Pose structure.
    Pose pose;
    int count = sscanf(command.c_str(), "%*s %d %d %d %d %d %d %lu",
                       &pose.angle[0], &pose.angle[1], &pose.angle[2],
                       &pose.angle[3], &pose.angle[4], &pose.angle[5],
                       &pose.durationMs);
    if (count != 7) {
      Serial.println("Invalid MOVE. Expected 6 angles and durationMs.");
      return;
    }
    // Keep every requested angle inside its servo's safe software range.
    pose.angle[SERVO_BASE] = constrain(pose.angle[SERVO_BASE], 0, 270);
    for (uint8_t i = 1; i < NUM_SERVOS; ++i)
      pose.angle[i] = constrain(pose.angle[i], 0, 180);

    executePose(pose);
    Serial.println(isRecording ? "Moved and recorded." : "Moved.");
  } else if (upper == "STOP") {
    // Close the file so the recorded sequence is safely stored.
    if (!isRecording) {
      Serial.println("No recording is active.");
      return;
    }
    recordingFile.close();
    isRecording = false;
    Serial.println("Recording saved.");
  } else if (upper.startsWith("PLAY ")) {
    // PLAY demo executes /seq_demo.csv once.
    if (isRecording) {
      Serial.println("STOP recording before playback.");
      return;
    }
    playSequence(sequencePath(command.substring(5)));
  } else if (upper == "PLAYALL") {
    // Execute all recorded sequence files once, one after another.
    if (isRecording) {
      Serial.println("STOP recording before playback.");
      return;
    }
    playAllSequences();
  } else if (upper == "LIST") {
    listSequences();
  } else if (upper.startsWith("DELETE ")) {
    // Remove the named sequence from flash storage.
    const String path = sequencePath(command.substring(7));
    Serial.println(LittleFS.remove(path) ? "Deleted " + path
                                        : "Could not delete " + path);
  } else if (upper == "HOME") {
    const Pose home = {{135, 90, 90, 90, 90, 0}, 1500};
    executePose(home);
  } else if (upper == "PEN") {
    // Run the built-in floor pen transfer.
    transferPen();
  } else if (upper == "HELP") {
    printHelp();
  } else {
    Serial.println("Unknown command. Type HELP.");
  }
}

void setup() {
  // Serial Monitor is used to send commands and read status messages.
  Serial.begin(115200);

  // ESP32 I2C pins: SDA = GPIO 21 and SCL = GPIO 22.
  Wire.begin(21, 22);

  // Start the PCA9685 servo controller.
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  // Mount the ESP32 internal flash filesystem. The true argument formats the
  // filesystem automatically if it has not been initialized yet.
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed.");
    while (true) delay(1000);
  }

  // Move to the initial home pose after all hardware is ready.
  delay(10);
  moveToPose(currentPose);
  Serial.println("Robot arm ready.");
  printHelp();
}

void loop() {
  // Build one command from incoming serial characters. Set Serial Monitor's
  // line ending to "Newline" or "Both NL & CR".
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      // A newline marks the end of one complete command.
      handleCommand(inputLine);
      inputLine = "";
    } else if (c != '\r') {
      inputLine += c;
    }
  }
}
