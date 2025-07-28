# Quadcopter Deep Dive

## Overview

This project implements an open‑source four‑rotor UAV platform. The repository contains the complete firmware for both flight controller and remote as well as hardware designs for the custom PCB. The microcontroller at the heart of both devices is the Atmel ATmega128RFA1 which integrates a 2.4&nbsp;GHz radio allowing wireless control and telemetry.

The codebase is organized under `firmware/` for all sketches and reusable libraries while `hardware/` holds EAGLE board/schematic files. The root [README](README.md) provides a high‑level summary and diagrams of the system architecture. Key highlights include support for PID tuning via the remote and telemetry reporting from the quadcopter back to the handheld controller.

This document dives into the design and implementation details of the project, covering hardware features, data structures, algorithms, and how the pieces interact.

## Hardware Design

### Board Metadata

The quadcopter PCB is a four‑layer board with dimensions of 119&nbsp;×&nbsp;119&nbsp;mm using red soldermask and white silkscreen as recorded in `hardware/metadata.txt`:

```text
Color: Red
Silkscreen colors: White
Dimensions: 119mm x 119mm
```
【F:hardware/metadata.txt†L1-L4】

### Major Components

* **ATmega128RFA1** – integrates an AVR CPU with a low‑power 2.4&nbsp;GHz transceiver.
* **LSM6DSOX IMU** – provides 3‑axis gyroscope and accelerometer data.
* FET motor drivers for the four ESC channels.
* Battery monitor on analog pin `BAT_SENSE_PIN`.
* Status LEDs and programming connectors.

The board and schematic files (`quadcopter.sch`, `quadcopter.brd`) can be opened in EAGLE for detailed component placement and routing inspection.

## Firmware Structure

The `firmware/` directory contains a number of Arduino sketches:

```
RFChat/        – radio terminal test
RFCount/       – simple counting transmitter
RFEcho/        – echo received bytes
RFPing/        – radio link ping utility
quad_flight/   – flight controller running on the quadcopter
remote_firmware/ – code for the handheld remote
libraries/     – shared driver libraries
```

The two primary sketches used in normal operation are `quad_flight/quad_flight.ino` and `remote_firmware/remote_firmware.ino`. Several custom libraries are included under `firmware/libraries` to abstract radio access, sensor fusion and user input.

### Wireless Packet Protocol

Both sides communicate using a packed C++ struct defined in `Wireless_Packet.h`. Every field of the struct is transmitted over the air, preceded by a magic number and followed by a checksum. The struct layout is reproduced in the README and in the header file:

```cpp
struct Packet {
    uint8_t magicNumber;
    uint8_t roll;
    uint8_t pitch;
    uint8_t yaw;
    uint8_t throttle;
    double Kp_pitch;
    double Ki_pitch;
    double Kd_pitch;
    double Kp_roll;
    double Ki_roll;
    double Kd_roll;
    double Kp_yaw;
    double Ki_yaw;
    double Kd_yaw;
    uint8_t batteryVoltage;
    int trims[3];
    uint8_t state;
    uint8_t checksum;
};
```
【F:firmware/libraries/Wireless_Packet/Wireless_Packet.h†L17-L36】

Two constants distinguish packet origin: `MAGIC_NUMBER_REMOTE` (`223`) and `MAGIC_NUMBER_QUAD` (`154`). The library provides a simple API consisting of `sendPacket`, `receivePacket`, and `calculateChecksum`. `receivePacket` reads bytes from the radio and verifies both checksum and magic number before returning:

```cpp
void Wireless_Packet::sendPacket(const Packet &packet) {
    rfWrite((uint8_t *)&packet, sizeof(Packet));
}

bool Wireless_Packet::receivePacket(Packet &packet, bool isRemote) {
    bool verify = false;
    if (rfAvailable() >= sizeof(Packet)) {
        rfRead((uint8_t *)&packet, sizeof(Packet));
        uint8_t checksum = calculateChecksum(packet);
        uint8_t magicNumber = packet.magicNumber;
        if (isRemote)
            verify =  (checksum == packet.checksum) && (magicNumber == MAGIC_NUMBER_QUAD);
        else
            verify =  (checksum == packet.checksum) && (magicNumber == MAGIC_NUMBER_REMOTE);
    }
    if (!verify) {
        rfFlush();
    }
    return verify;
}
```
【F:firmware/libraries/Wireless_Packet/Wireless_Packet.cpp†L1-L26】

Checksum generation is a simple XOR of all preceding bytes.

### Radio Driver

The `Radio` library (based on SparkFun code for the ATmega128RFA1) initializes the transceiver, sends and receives frames and implements ring‑buffer management. Functions such as `rfWrite` and `rfRead` handle arbitrary byte sequences while hardware interrupts fill the receive buffer. See `radio.cpp` for the full implementation.

### Remote Firmware

The handheld remote houses two analog gimbals (four axes), a rotary encoder, a directional button pad and a serial LCD. On boot, `remote_firmware.ino` calls `quad_remote_setup()` which configures pins and interrupts as defined in `quad_remote.h`. Calibration data for joystick min/max values is stored in EEPROM so that analog readings can be mapped to 0–255. During startup the sketch loads this data:

```cpp
CalibrationData calibrationData;
EEPROM.get(0, calibrationData);
if (calibrationData.maxYaw != 0 && calibrationData.maxThrottle != 0 &&
    calibrationData.maxRoll != 0 && calibrationData.maxPitch != 0 &&
    calibrationData.minYaw != 1023 && calibrationData.minThrottle != 1023 &&
    calibrationData.minRoll != 1023 && calibrationData.minPitch != 1023) {
  maxYaw = calibrationData.maxYaw;
  maxThrottle = calibrationData.maxThrottle;
  maxRoll = calibrationData.maxRoll;
  maxPitch = calibrationData.maxPitch;
  minYaw = calibrationData.minYaw;
  minThrottle = calibrationData.minThrottle;
  minRoll = calibrationData.minRoll;
  minPitch = calibrationData.minPitch;
}
```
【F:firmware/remote_firmware/remote_firmware.ino†L104-L116】

During operation the remote repeatedly reads the gimbals, maps them to 8‑bit values and constructs an outgoing packet containing pilot commands, PID gains (which can be set over the serial port) and trim adjustments. The packet is then checksummed and sent via the radio:

```cpp
Packet packetToSend = {MAGIC_NUMBER_REMOTE, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0,
                       {pitchTrim, rollTrim, yawTrim}, state, 0};
packetToSend.pitch = gimballVals[3];
packetToSend.roll = gimballVals[2];
packetToSend.yaw = gimballVals[0];
packetToSend.throttle = gimballVals[1];
packetToSend.Kp_pitch = Kp_pitch;
packetToSend.Ki_pitch = Ki_pitch;
packetToSend.Kd_pitch = Kd_pitch;
packetToSend.Kp_roll = Kp_roll;
packetToSend.Ki_roll = Ki_roll;
packetToSend.Kd_roll = Kd_roll;
packetToSend.Kp_yaw = Kp_yaw;
packetToSend.Ki_yaw = Ki_yaw;
packetToSend.Kd_yaw = Kd_yaw;
packetToSend.checksum = wirelessPacket.calculateChecksum(packetToSend);
wirelessPacket.sendPacket(packetToSend);
```
【F:firmware/remote_firmware/remote_firmware.ino†L230-L241】

When in calibration mode, raw ADC values are shown on the LCD and the max/min values are updated until the user presses the center button. The resulting ranges are written back to EEPROM:

```cpp
if(is_pressed(BUTTON_CENTER_PIN) && (millis() - centerButtonStartTime > 250)) {
  centerButtonStartTime = millis();
  state = DISARMED;
  CalibrationData calibrationData = {maxYaw, maxThrottle, maxRoll, maxPitch,
                                     minYaw, minThrottle, minRoll, minPitch};
  EEPROM.put(0, calibrationData);
  lcd.clear();
  lcd.home();
  lcd.print("Calib Done!");
  ...
}
```
【F:firmware/remote_firmware/remote_firmware.ino†L300-L308】

Gimbal input mapping is handled by the helper `readGimballs()` which constrains raw readings according to stored min/max and maps them to 0–255:

```cpp
int yaw = map(constrain(analogRead(PIN_YAW), minYaw+15, maxYaw-15), minYaw+15, maxYaw-15, 0, 255);
int throttle = map(constrain(analogRead(PIN_THROTTLE), minThrottle+15, maxThrottle-15), minThrottle+15, maxThrottle-15, 0, 255);
int roll = map(constrain(analogRead(PIN_ROLL), minRoll+15, maxRoll-15), minRoll+15, maxRoll-15, 0, 255);
int pitch = map(constrain(analogRead(PIN_PITCH), minPitch+15, maxPitch-15), minPitch+15, maxPitch-15, 0, 255);
gimballArr[0] = yaw;
gimballArr[1] = throttle;
gimballArr[2] = roll;
gimballArr[3] = pitch;
```
【F:firmware/remote_firmware/remote_firmware.ino†L449-L457】

### Flight Controller Firmware

The flight controller sketch, `quad_flight.ino`, initializes the radio and the LSM6DSOX IMU, then calibrates the sensor offsets. Using the Adafruit AHRS library, it obtains roll and pitch angles plus gyro rates. A complementary filter mixes gyro rates with measured angles to produce smoothed estimates. The controller maintains PID variables for all three axes.

During `loop()`, received packets provide desired pitch/roll/yaw inputs as well as updated PID coefficients. The incoming packet processing ensures state transitions are stable by requiring several consistent packets before changing modes.

IMU readings are converted to rates and angles, offsets subtracted, and then the complementary filter updates the current orientation. The PID controller then computes corrections and sends commands to the four motors via `spinMotors()`.

The `spinMotors` function prevents output saturation by scaling throttle if any motor would exceed the allowable PWM range. The implementation is:

```cpp
void spinMotors(int throttle, int control_output_roll, int control_output_pitch, int control_output_yaw) {
    if (throttle < 5) {
        analogWrite(MOTOR1, 0);
        analogWrite(MOTOR2, 0);
        analogWrite(MOTOR3, 0);
        analogWrite(MOTOR4, 0);
    } else {
        int pid1 = control_output_roll - control_output_pitch - control_output_yaw;
        int pid2 = control_output_roll + control_output_pitch + control_output_yaw;
        int pid3 = -control_output_roll + control_output_pitch - control_output_yaw;
        int pid4 = -control_output_roll - control_output_pitch + control_output_yaw;

        int max_pid = max(max(pid1, pid2), max(pid3, pid4));
        int adjusted_throttle = throttle;
        if (max_pid + throttle > 255) {
            adjusted_throttle = 255 - max_pid;
        }

        int motor1_val = constrain(adjusted_throttle + pid1, 0, 255);
        int motor2_val = constrain(adjusted_throttle + pid2, 0, 255);
        int motor3_val = constrain(adjusted_throttle + pid3, 0, 255);
        int motor4_val = constrain(adjusted_throttle + pid4, 0, 255);

        analogWrite(MOTOR1, motor1_val);
        analogWrite(MOTOR2, motor2_val);
        analogWrite(MOTOR3, motor3_val);
        analogWrite(MOTOR4, motor4_val);
    }
}
```
【F:firmware/quad_flight/quad_flight.ino†L497-L531】

Battery voltage monitoring is performed with `readBatteryPercentage` which maps the analog reading between calibrated limits to 0‑100%:

```cpp
int readBatteryPercentage() {
  int raw = analogRead(BAT_SENSE_PIN);
  return map(constrain(raw, lowerBatteryLimit, upperBatteryLimit), lowerBatteryLimit, upperBatteryLimit, 0, 100);
}
```
【F:firmware/quad_flight/quad_flight.ino†L568-L571】

### Sensor Fusion Library

The included `QuadClass_AHRS` library is derived from Adafruit’s AHRS algorithms. The `Adafruit_Simple_AHRS` class converts accelerometer and gyroscope readings into Euler angles. `getQuadOrientation` returns `quad_data_t` which includes roll/pitch angles and rotational rates:

```cpp
typedef struct {
     float roll;
     float pitch;
     float roll_rate;
     float pitch_rate;
     float yaw_rate;
} quad_data_t;
```
【F:firmware/libraries/QuadClass_AHRS/src/Adafruit_Simple_AHRS.h†L10-L18】

The library computes pitch and roll from accelerometer data and outputs degrees along with gyro rates, allowing the main firmware to implement the complementary filter and PID loops.

## Remote Input Handling Library

`quad_remote.cpp` sets up the rotary encoder and button interrupts. It defines arrays of pin numbers and labels for all 14 input channels. Interrupt service routines call customizable callbacks when buttons change state so the main firmware can react immediately. The remote uses the `SoftwareSerial2` library to communicate with the serial LCD.

## Safety and Reliability Considerations

* **Checksum Verification** – All radio packets are XOR‑checksummed (see `calculateChecksum`) to detect corruption. Non‑matching packets are discarded and the radio buffer is flushed to prevent stale data being used.
* **Magic Numbers** – `MAGIC_NUMBER_REMOTE` and `MAGIC_NUMBER_QUAD` prevent cross‑talk with other devices and catch mis‑aligned packets.
* **Ring Buffer** – The radio driver uses a 127‑byte ring buffer and interrupt handlers to capture incoming frames without blocking the main loop. Functions such as `rfAvailable` and `rfRead` manage head/tail pointers safely.
* **Throttle Clamping** – `spinMotors` scales down throttle if any motor would exceed 255 ensuring PWM outputs stay within range and maintain control authority.
* **State Machine** – Both remote and quad track states (`DISARMED`, `ARMED`, `CALIBRATION`, `QUAD_RESET`) to avoid spurious motor activation. The flight controller requires multiple identical state packets before changing state to avoid glitching due to packet loss.
* **PID Wind‑up Protection** – Integral terms are cleared when throttle is low or the respective Ki gain is nearly zero, preventing runaway accumulation while the quad is idle.

## Building and Uploading

To compile either firmware sketch you must install the Arduino IDE, copy the board support package from `hardware/QuadClass_Atmega128RFA_Arduino_Addon` into your Arduino hardware directory, and copy the libraries inside `firmware/libraries` to your Arduino `libraries` folder. Building `quad_flight/quad_flight.ino` targets the quadcopter; `remote_firmware/remote_firmware.ino` builds the remote. Uploads are performed via the standard Arduino bootloader over USB.

## Conclusion

The repository implements a full small‑scale UAV stack. The combination of custom hardware, wireless packet protocol and PID‑based stabilization provides a solid foundation for experimentation. The remote’s ability to tune PID parameters in flight and store joystick calibration data in EEPROM makes the platform versatile and easy to adjust.

Telemetry such as battery level and state feedback ensures the operator stays informed of system status. All code is written in C++ using the Arduino framework, making it straightforward to extend or modify.

This deep dive should provide a thorough understanding of how the project components fit together, how data flows from the user inputs to the motors, and how safety is maintained throughout the codebase.
