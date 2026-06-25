# Robot Arm

ESP32 PlatformIO firmware for a six-axis robot arm controlled through a PCA9685 PWM servo driver. The firmware creates a Wi-Fi access point with a browser-based control panel, supports calibrated joint movement, and can record and replay task routines.

## Hardware

- ESP32 development board
- PCA9685 16-channel PWM servo driver at I2C address `0x40`
- Six servos:
  - Base
  - Shoulder
  - Elbow
  - Wrist Pitch
  - Wrist Roll
  - Gripper
- I2C wiring on ESP32 pins `SDA=21` and `SCL=22`
- External servo power supply sized for the arm load

## Project Layout

- `src/RobotArm.cpp` - main firmware, servo control logic, startup audit, Wi-Fi server, and routine handling
- `src/control_page.h` - embedded browser control UI
- `platformio.ini` - PlatformIO board, framework, and library configuration
- `test/` - prototype and test sketches

## Build and Upload

Install PlatformIO, connect the ESP32, then run:

```sh
pio run
pio run --target upload
pio device monitor --baud 115200
```

The project targets the `esp32dev` board with the Arduino framework.

## Using the Web Controller

After upload, the ESP32 starts an access point:

- SSID: `RobotArm-Control`
- Password: `robotarm`
- URL: `http://192.168.4.1`

The web controller lets you:

- Move the arm and wrist with joystick controls
- Adjust movement speed, capped at 60 percent
- Home the arm
- Enable or disable servo output
- Create, save, replay, and delete recorded task routines

## Startup Behavior

On boot, the firmware:

1. Starts serial logging at `115200` baud.
2. Checks for the PCA9685 controller on I2C address `0x40`.
3. Runs a small movement test on each servo.
4. Waits for `Y` in Serial Monitor before homing the arm.

This confirmation step is intentional so the arm does not continue if a servo, linkage, or power connection is unsafe.

## Notes

- Calibrate pulse limits in `src/RobotArm.cpp` before running the arm under load.
- Use a dedicated servo power supply. Do not power multiple servos from the ESP32 regulator.
- The built-in pick-and-place poses are examples and should be adjusted for the actual arm geometry and workspace.
