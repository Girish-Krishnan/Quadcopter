// Imports
#include "radio.h"
#include "quad_remote.h"      // Header file with pin definitions and setup
#include <SerLCD.h>
#include <Wireless_Packet.h>
#include <EEPROM.h>

// Constants
#define MIN_BATTERY_VALUE 45
#define MAX_BATTERY_VALUE 57
#define BAT_SENSE_PIN A7

// Global variables
Wireless_Packet wirelessPacket;

// Data values
uint8_t gimballVals[4] = {0, 0, 0, 0}; // yaw, throttle, roll, pitch
int gimballValsRaw[4] = {0, 0, 0, 0}; // yaw, throttle, roll, pitch

// Max Vals
int maxYaw = 0;
int maxThrottle = 0;
int maxRoll = 0;
int maxPitch = 0;

int minYaw = 1023;
int minThrottle = 1023;
int minRoll = 1023;
int minPitch = 1023;

// State vars
State state = DISARMED;
State quadState = DISARMED;
int quadBattery = 0;

// Calibration data
struct CalibrationData {
  int maxYaw;
  int maxThrottle;
  int maxRoll;
  int maxPitch;
  int minYaw;
  int minThrottle;
  int minRoll;
  int minPitch;
};

// PID coefficients
double Kp_pitch = 0.0;
double Ki_pitch = 0.0;
double Kd_pitch = 0.0;
double Kp_roll = 0.0;
double Ki_roll = 0.0;
double Kd_roll = 0.0;
double Kp_yaw = 0.0;
double Ki_yaw = 0.0;
double Kd_yaw = 0.0;

// other constants
const int timeDelay = 50; // milliseconds

// other vars
int startTime = millis();

// Button debounce variables
unsigned long centerButtonStartTime = millis();
unsigned long button1StartTime = millis();
unsigned long button2StartTime = millis();
unsigned long buttonUpStartTime = millis();
unsigned long buttonDownStartTime = millis();
unsigned long buttonLeftStartTime = millis();
unsigned long buttonRightStartTime = millis();
unsigned long armTime = millis();
bool armFlag = false;

// Calibration variables
int currentYaw = 0;
int currentThrottle = 0;
int currentRoll = 0;
int currentPitch = 0;

int pitchTrim = 0;
int rollTrim = 0; 
int yawTrim = 0;
int tuneMode = 0;

void setup() {
  // Serial
  Serial.begin(115200);

  // Setup
  quad_remote_setup();
  lcd.home();
  lcd.clear();
  lcd.print("DISARM");

  // battery
  pinMode(BATTERY_SENSE_PIN, INPUT);
  analogReference(INTERNAL);

  // transmission variables
  rfBegin(RADIO_CHANNEL);

  // Read calibration data from EEPROM, and first check if there is some valid calibration data
  CalibrationData calibrationData;
  EEPROM.get(0, calibrationData);
  if (calibrationData.maxYaw != 0 && calibrationData.maxThrottle != 0 && calibrationData.maxRoll != 0 && calibrationData.maxPitch != 0 && calibrationData.minYaw != 1023 && calibrationData.minThrottle != 1023 && calibrationData.minRoll != 1023 && calibrationData.minPitch != 1023) {
    maxYaw = calibrationData.maxYaw;
    maxThrottle = calibrationData.maxThrottle;
    maxRoll = calibrationData.maxRoll;
    maxPitch = calibrationData.maxPitch;
    minYaw = calibrationData.minYaw;
    minThrottle = calibrationData.minThrottle;
    minRoll = calibrationData.minRoll;
    minPitch = calibrationData.minPitch;
  }

}

void loop() {

  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');

    // If the line starts with pitch
    if (data.startsWith("pitch")) {
      data.remove(0, 5);
      
      // Find the part of the string starting with P=, and then take the substring after that ending with a space
      String P = data.substring(data.indexOf("P=") + 2, data.indexOf(" ", data.indexOf("P=") + 2));
      String I = data.substring(data.indexOf("I=") + 2, data.indexOf(" ", data.indexOf("I=") + 2));
      String D = data.substring(data.indexOf("D=") + 2, data.indexOf(" ", data.indexOf("D=") + 2));

      // Convert the strings to doubles
      Kp_pitch = P.toDouble();
      Ki_pitch = I.toDouble();
      Kd_pitch = D.toDouble();

      // Print the values to the console 
      Serial.print("[PITCH] ");
      Serial.print("P: ");
      Serial.print(Kp_pitch);
      Serial.print(" I: ");
      Serial.print(Ki_pitch);
      Serial.print(" D: ");
      Serial.println(Kd_pitch);
    }

    // If the line starts with roll
    if (data.startsWith("roll")) {
      data.remove(0, 4);
      
      // Find the part of the string starting with P=, and then take the substring after that ending with a space
      String P = data.substring(data.indexOf("P=") + 2, data.indexOf(" ", data.indexOf("P=") + 2));
      String I = data.substring(data.indexOf("I=") + 2, data.indexOf(" ", data.indexOf("I=") + 2));
      String D = data.substring(data.indexOf("D=") + 2, data.indexOf(" ", data.indexOf("D=") + 2));

      // Convert the strings to doubles
      Kp_roll = P.toDouble();
      Ki_roll = I.toDouble();
      Kd_roll = D.toDouble();

      // Print the values to the console 
      Serial.print("[ROLL] ");
      Serial.print("P: ");
      Serial.print(Kp_roll);
      Serial.print(" I: ");
      Serial.print(Ki_roll);
      Serial.print(" D: ");
      Serial.println(Kd_roll);
    }

    // If the line starts with yaw
    if (data.startsWith("yaw")) {
      data.remove(0, 3);
      
      // Find the part of the string starting with P=, and then take the substring after that ending with a space
      String P = data.substring(data.indexOf("P=") + 2, data.indexOf(" ", data.indexOf("P=") + 2));
      String I = data.substring(data.indexOf("I=") + 2, data.indexOf(" ", data.indexOf("I=") + 2));
      String D = data.substring(data.indexOf("D=") + 2, data.indexOf(" ", data.indexOf("D=") + 2));

      // Convert the strings to doubles
      Kp_yaw = P.toDouble();
      Ki_yaw = I.toDouble();
      Kd_yaw = D.toDouble();

      // Print the values to the console 
      Serial.print("[YAW] ");
      Serial.print("P: ");
      Serial.print(Kp_yaw);
      Serial.print(" I: ");
      Serial.print(Ki_yaw);
      Serial.print(" D: ");
      Serial.println(Kd_yaw);
    }

  }

  // CODE TO RECEIVE PACKET

  Packet receivedPacket;
  if (wirelessPacket.receivePacket(receivedPacket, true)) {
    quadState = (State) receivedPacket.state;
    quadBattery = receivedPacket.batteryVoltage;

    if(quadState == QUAD_RESET) {
      state = DISARMED;
      lcd.clear();
      lcd.home();
      lcd.print("DISARM ");
      lcd.print("Batt ");
      lcd.print(readBatteryPercentage());
      lcd.print("%");
      lcd.setCursor(0, 1);
      lcd.print("FCB Batt: ");
      lcd.print(quadBattery);
      lcd.print("%");
    }

  }

  // CODE TO SEND PACKET
  if (millis() - startTime > timeDelay) {
    startTime = millis();
    readGimballs(gimballVals);
    Packet packetToSend = {MAGIC_NUMBER_REMOTE, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, {pitchTrim, rollTrim, yawTrim}, state, 0};
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
  }

  switch (state) {
    case CALIBRATION:   

      readGimballsRaw(gimballValsRaw);

      currentPitch = gimballValsRaw[3];
      currentRoll = gimballValsRaw[2];
      currentYaw = gimballValsRaw[0];
      currentThrottle = gimballValsRaw[1];  

      lcd.clear();
      lcd.home();
      lcd.print("Y: ");
      lcd.print(currentYaw);
      lcd.print("  Th: ");
      lcd.print(currentThrottle);
      lcd.setCursor(0, 1);
      lcd.print("R: ");
      lcd.print(currentRoll);
      lcd.print("  P: ");
      lcd.print(currentPitch);
      delay(50);

      if (currentPitch > maxPitch) {
        maxPitch = currentPitch;
      }

      if (currentPitch < minPitch) {
        minPitch = currentPitch;
      }

      if (currentRoll > maxRoll) {
        maxRoll = currentRoll;
      }

      if (currentRoll < minRoll) {
        minRoll = currentRoll;
      }

      if (currentYaw > maxYaw) {
        maxYaw = currentYaw;
      }

      if (currentYaw < minYaw) {
        minYaw = currentYaw;
      }

      if (currentThrottle > maxThrottle) {
        maxThrottle = currentThrottle;
      }

      if (currentThrottle < minThrottle) {
        minThrottle = currentThrottle;
      }


      if(is_pressed(BUTTON_CENTER_PIN) && (millis() - centerButtonStartTime > 250)) {
        centerButtonStartTime = millis();
        state = DISARMED;
        CalibrationData calibrationData = {maxYaw, maxThrottle, maxRoll, maxPitch, minYaw, minThrottle, minRoll, minPitch};
        EEPROM.put(0, calibrationData);
        lcd.clear();
        lcd.home();
        lcd.print("Calib Done!");
        delay(1000);
        lcd.clear();
        lcd.home();
        lcd.print("DISARM ");
        lcd.print("Batt ");
        lcd.print(readBatteryPercentage());
        lcd.print("%");
        lcd.setCursor(0, 1);
        lcd.print("FCB Batt: ");
        lcd.print(quadBattery);
        lcd.print("%");
      }

      if(is_pressed(BUTTON1_PIN) && (millis() - button1StartTime > 250)) {
        button1StartTime = millis();
        maxPitch = 0;
        maxRoll = 0;
        maxYaw = 0;
        maxThrottle = 0;
        minPitch = 1023;
        minRoll = 1023;
        minYaw = 1023;
        minThrottle = 1023;
      }

      break;

    case DISARMED:
      
      if(is_pressed(BUTTON_CENTER_PIN) && (millis() - centerButtonStartTime > 250)) {
        centerButtonStartTime = millis();
        state = CALIBRATION;
      }

      readGimballs(gimballVals);
      currentPitch = gimballVals[3];
      currentRoll = gimballVals[2];
      currentYaw = gimballVals[0];
      currentThrottle = gimballVals[1];

      if ((currentThrottle == 0) && (currentPitch == 255) && (currentRoll == 255) && (currentYaw == 255)) {
        if((!armFlag)) {
          armFlag = true;
          armTime = millis();
        }
        else {
          if(millis() - armTime >= 3000) {
            armFlag = false;
            state = ARMED;
            lcd.clear();
            lcd.home();
          }
        }
      }
    else {
      armFlag = false;
    }

      break;
    case ARMED:
      int dialTrim = readKnob();
      lcd.clear();
      lcd.home();
      lcd.print("ARMED TR: ");
      lcd.print(dialTrim);

      if(is_pressed(BUTTON_CENTER_PIN) && (millis() - centerButtonStartTime > 250)) {
        centerButtonStartTime = millis();
        state = DISARMED;
        lcd.clear();
        lcd.home();
        lcd.print("DISARM ");
        lcd.print("Batt ");
        lcd.print(readBatteryPercentage());
        lcd.print("%");
        lcd.setCursor(0, 1);
        lcd.print("FCB Batt: ");
        lcd.print(quadBattery);
        lcd.print("%");
      }

      if(is_pressed(BUTTON_LEFT_PIN) && (millis() - buttonLeftStartTime > 250)) {
        buttonLeftStartTime = millis();
        lcd.clear();
        lcd.home();
        lcd.print("ARMED TR: ");
        lcd.print(dialTrim);
        lcd.setCursor(0, 1);
        lcd.print("YAW TRIMMED");
        yawTrim = dialTrim;
      }

      if(is_pressed(BUTTON_RIGHT_PIN) && (millis() - buttonRightStartTime > 250)) {
        buttonRightStartTime = millis();
        lcd.clear();
        lcd.home();
        lcd.print("ARMED TR: ");
        lcd.print(dialTrim);
        lcd.setCursor(0, 1);
        lcd.print("PITCH TRIMMED");
        pitchTrim = dialTrim;
      }

      if(is_pressed(BUTTON_UP_PIN) && (millis() - buttonUpStartTime > 250)) {
        buttonUpStartTime = millis();
        lcd.clear();
        lcd.home();
        lcd.print("ARMED TR: ");
        lcd.print(dialTrim);
        lcd.setCursor(0, 1);
        lcd.print("ROLL TRIMMED");
        rollTrim = dialTrim;
      }

      if(is_pressed(BUTTON_DOWN_PIN) && (millis() - buttonDownStartTime > 250)) {
        buttonDownStartTime = millis();
        lcd.clear();
        lcd.home();
        lcd.print("ARMED TR: ");
        lcd.print(dialTrim);
        lcd.setCursor(0, 1);
        lcd.print("CLEAR ALL");
        rollTrim = 0;
        pitchTrim = 0;
        yawTrim = 0;
      }

      delay(20);

      break;
  }
  
}

void readGimballsRaw(int * gimballArr) {
  gimballArr[0] = analogRead(PIN_YAW);
  gimballArr[1] = analogRead(PIN_THROTTLE);
  gimballArr[2] = analogRead(PIN_ROLL);
  gimballArr[3] = analogRead(PIN_PITCH);
}

void readGimballs(uint8_t * gimballArr) {
  int yaw = map(constrain(analogRead(PIN_YAW), minYaw+15, maxYaw-15), minYaw+15, maxYaw-15, 0, 255);
  int throttle = map(constrain(analogRead(PIN_THROTTLE), minThrottle+15, maxThrottle-15), minThrottle+15, maxThrottle-15, 0, 255);
  int roll = map(constrain(analogRead(PIN_ROLL), minRoll+15, maxRoll-15), minRoll+15, maxRoll-15, 0, 255);
  int pitch = map(constrain(analogRead(PIN_PITCH), minPitch+15, maxPitch-15), minPitch+15, maxPitch-15, 0, 255);
  gimballArr[0] = yaw;
  gimballArr[1] = throttle;
  gimballArr[2] = roll;
  gimballArr[3] = pitch;
}

int readBatteryPercentage() {
  int val = analogRead(BAT_SENSE_PIN);
  int battery_percentage = map(constrain(val, MIN_BATTERY_VALUE, MAX_BATTERY_VALUE), MIN_BATTERY_VALUE, MAX_BATTERY_VALUE, 0, 100);
  return battery_percentage;
}

int readKnob() {
  return knob1.getCurrentPos();
}