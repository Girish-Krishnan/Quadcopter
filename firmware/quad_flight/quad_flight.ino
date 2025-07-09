#include "radio.h"
#include <Wireless_Packet.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <QuadClass_LSM6DSOX.h>
#include <Adafruit_Simple_AHRS.h>
Wireless_Packet wirelessPacket;
#define RAD_TO_DEG 57.295779513082320876798154814105
#define CALIBRATION_SAMPLES 50

// REF DESIGN
// const int MOTOR1 = 8; // BLUE
// const int MOTOR2 = 3; // RED
// const int MOTOR3 = 4; // GREEN
// const int MOTOR4 = 5; // UN

// OUR QUADCOPTER
// when the quad is upside down...
// spinning 1 and 3 -> clockwise rotation
// spinning 2 and 4 -> counter clockwise rotation
const int MOTOR1 = 8; // BLUE
const int MOTOR2 = 4; // RED
const int MOTOR3 = 3; // GREEN
const int MOTOR4 = 5; // UN

// LEDs
const int LED_1 = 16; 
const int LED_2 = 17; 
const int LED_3 = 18; 
const int LED_4 = 36;
const int PRETTY_LEDS = 34;

// Battery Voltage
const int BAT_SENSE_PIN = A7;
const int lowerBatteryLimit = 490;
const int upperBatteryLimit = 887;

// State
State state = DISARMED;
State prevState = DISARMED;
bool resetMode = true;

// other variables
int remoteThrottle = 0;
int remoteRoll = 0;
int remotePitch = 0;
int remoteYaw = 0;
int pitchTrim = 0;
int rollTrim = 0;
int yawTrim = 0;
int countStateChange = 0;

// Create LSM9DS0 board instance.
QuadClass_LSM6DSOX lsm = QuadClass_LSM6DSOX();
Adafruit_Simple_AHRS *ahrs = NULL;
Adafruit_Sensor *_accel = NULL;
Adafruit_Sensor *_gyro = NULL;
Adafruit_Sensor *_mag = NULL;  // No Yaw orientation | Always NULL
bool is_calib_IMU = false;

// store initial angle for complementary filter
double cf_pitch = 0.0;
double cf_roll = 0.0;
double cf_yaw = 0.0;

double pitch_angle = 0.0;
double roll_angle = 0.0;
double yaw_angle = 0.0;

double IMU_pitch_offset = 0.0;
double IMU_roll_offset = 0.0;
double IMU_yaw_offset = 0.0;

double IMU_pitch_rate_offset = 0.0;
double IMU_roll_rate_offset = 0.0;
double IMU_yaw_rate_offset = 0.0;

double incoming_pitch = 0.0;
double incoming_roll = 0.0;
double incoming_yaw = 0.0;

double incoming_pitch_rate = 0.0;
double incoming_roll_rate = 0.0;
double incoming_yaw_rate = 0.0;

double gain = 0.92;

// PID
double Kp_roll = 0.0;
double Ki_roll = 0.0;
double Kd_roll = 0.0;

double Kp_pitch = 0.0;
double Ki_pitch = 0.0;
double Kd_pitch = 0.0;

double Kp_yaw = 0.0;
double Ki_yaw = 0.0;
double Kd_yaw = 0.0;

double desired_yaw_rate = 0.0;
double desired_roll = 0.0;
double desired_pitch = 0.0;

double error_roll = 0.0;
double last_error_roll = 0.0;
double sum_error_roll = 0.0;
double derivative_error_roll = 0.0;

double error_pitch = 0.0;
double last_error_pitch = 0.0;
double sum_error_pitch = 0.0;
double derivative_error_pitch = 0.0;

double error_yaw = 0.0;
double last_error_yaw = 0.0;
double sum_error_yaw = 0.0;
double derivative_error_yaw = 0.0;

// Timings
unsigned long timeLastMessage = millis();
unsigned long timeLastSent = millis();
unsigned long lastPIDTime = millis();
unsigned long lastIMU = millis();

void setup() {
  // Serial
  Serial.begin(115200);

  // Battery
  analogReference(INTERNAL);
  pinMode(BAT_SENSE_PIN, INPUT);

  // Motors
  pinMode(MOTOR1, OUTPUT);
  pinMode(MOTOR2, OUTPUT);
  pinMode(MOTOR3, OUTPUT);
  pinMode(MOTOR4, OUTPUT);

  // LEDs
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(LED_4, OUTPUT);
  pinMode(PRETTY_LEDS, OUTPUT);

  // Radio
  rfBegin(RADIO_CHANNEL);

  // Setup the sensor gain and integration time.
  setupSensor();

  // Set Offset Values
  quad_data_t orientation;
  turnOnLEDs();
  delay(1000);
  turnOffLEDs();
  delay(3000);
  // while(1) {
  //   if (ahrs->getQuadOrientation(&orientation)) {
  //     Serial.println("CALIBRATING...");
  //     IMU_pitch_rate_offset = orientation.pitch_rate;
  //     IMU_roll_rate_offset = orientation.roll_rate;
  //     IMU_yaw_rate_offset = orientation.yaw_rate;

  //     IMU_pitch_offset = orientation.pitch;
  //     IMU_roll_offset = orientation.roll;

  //     Serial.println("Calibrated Values:");
  //     Serial.print("Pitch Rate Offset: ");
  //     Serial.println(IMU_pitch_rate_offset);
  //     Serial.print("Roll Rate Offset: ");
  //     Serial.println(IMU_roll_rate_offset);
  //     Serial.print("Yaw Rate Offset: ");
  //     Serial.println(IMU_yaw_rate_offset);
  //     Serial.print("Pitch Offset: ");
  //     Serial.println(IMU_pitch_offset);
  //     Serial.print("Roll Offset: ");
  //     Serial.println(IMU_roll_offset);
  //     turnOnLEDs();
  //     delay(1000);
  //     turnOffLEDs();
  //     break;
  //   }
  // }

  double pitch_samples[CALIBRATION_SAMPLES];
  double roll_samples[CALIBRATION_SAMPLES];
  double pitch_rate_samples[CALIBRATION_SAMPLES];
  double roll_rate_samples[CALIBRATION_SAMPLES];
  double yaw_rate_samples[CALIBRATION_SAMPLES];

  // Collect samples
  for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
        quad_data_t orientation;
        while(1) {
        if (ahrs->getQuadOrientation(&orientation)) {
            pitch_samples[i] = orientation.pitch;
            roll_samples[i] = orientation.roll;
            pitch_rate_samples[i] = orientation.pitch_rate;
            roll_rate_samples[i] = orientation.roll_rate;
            yaw_rate_samples[i] = orientation.yaw_rate;
            delay(20); // Delay to allow time between readings
        }
        break;
        }
  }

  findMedian(pitch_samples, CALIBRATION_SAMPLES);
  findMedian(roll_samples, CALIBRATION_SAMPLES);
  findMedian(pitch_rate_samples, CALIBRATION_SAMPLES);
  findMedian(roll_rate_samples, CALIBRATION_SAMPLES);
  findMedian(yaw_rate_samples, CALIBRATION_SAMPLES);

  IMU_pitch_offset = pitch_samples[CALIBRATION_SAMPLES / 2];
  IMU_roll_offset = roll_samples[CALIBRATION_SAMPLES / 2];
  IMU_pitch_rate_offset = pitch_rate_samples[CALIBRATION_SAMPLES / 2];
  IMU_roll_rate_offset = roll_rate_samples[CALIBRATION_SAMPLES / 2];
  IMU_yaw_rate_offset = yaw_rate_samples[CALIBRATION_SAMPLES / 2];

  Serial.println("Calibrated Values:");
  Serial.print("Pitch Rate Offset: ");
  Serial.println(IMU_pitch_rate_offset);
  Serial.print("Roll Rate Offset: ");
  Serial.println(IMU_roll_rate_offset);
  Serial.print("Yaw Rate Offset: ");
  Serial.println(IMU_yaw_rate_offset);
  Serial.print("Pitch Offset: ");
  Serial.println(IMU_pitch_offset);
  Serial.print("Roll Offset: ");
  Serial.println(IMU_roll_offset);

  turnOnLEDs();
  delay(1000);
  turnOffLEDs();

}

void findMedian(double* array, int n) {
    for (int i = 0; i < n - 1; i++) {     
        for (int j = 0; j < n - i - 1; j++) {
            if (array[j] > array[j + 1]) {
                double temp = array[j];
                array[j] = array[j + 1];
                array[j + 1] = temp;
            }
        }
    }
}

void loop() {
  // RECEIVE PACKETS
  if(millis() - timeLastMessage > 100) {
    Packet receivedPacket;
    if (wirelessPacket.receivePacket(receivedPacket, false)) {
      resetMode = false;
      timeLastMessage = millis();
      if(prevState != (State) receivedPacket.state) {
        countStateChange++;
        if(countStateChange >= 8) {
          state = (State) receivedPacket.state;
          prevState = (State) receivedPacket.state;
          countStateChange = 0;
        }
      }
      else {
        countStateChange = 0;
      }

      remoteThrottle = receivedPacket.throttle;
      remoteRoll = receivedPacket.roll;
      remotePitch = receivedPacket.pitch;
      remoteYaw = receivedPacket.yaw;
      pitchTrim = receivedPacket.trims[0];
      rollTrim = receivedPacket.trims[1];
      yawTrim = receivedPacket.trims[2];

      if(abs(remoteRoll) < 3) remoteRoll = 0;
      if(abs(remotePitch) < 3) remotePitch = 0;
      if(abs(remoteYaw) < 3) remoteYaw = 0;

      desired_pitch = map(remotePitch, 0, 255, -90, 90) + pitchTrim;
      desired_roll = map(remoteRoll, 0, 255, -90, 90) + rollTrim;
      desired_yaw_rate = map(remoteYaw, 0, 255, -20, 20) + yawTrim;

      Kp_roll = receivedPacket.Kp_roll;
      Ki_roll = receivedPacket.Ki_roll;
      Kd_roll = receivedPacket.Kd_roll;

      Kp_pitch = receivedPacket.Kp_pitch;
      Ki_pitch = receivedPacket.Ki_pitch;
      Kd_pitch = receivedPacket.Kd_pitch;

      Kp_yaw = receivedPacket.Kp_yaw;
      Ki_yaw = receivedPacket.Ki_yaw;
      Kd_yaw = receivedPacket.Kd_yaw;
    }
  }

  else {
    if (millis() - timeLastMessage > 5000) {
      timeLastMessage = millis();
      state = DISARMED;
    }
  }

  // IMU
  quad_data_t orientation;
  unsigned long nowIMU = millis();
  if ((ahrs->getQuadOrientation(&orientation))) {
    incoming_pitch_rate = orientation.pitch_rate - IMU_pitch_rate_offset;
    incoming_roll_rate = orientation.roll_rate - IMU_roll_rate_offset;
    incoming_yaw_rate = orientation.yaw_rate - IMU_yaw_rate_offset;

    incoming_pitch = orientation.pitch - IMU_pitch_offset;
    incoming_roll = orientation.roll - IMU_roll_offset;

    if(abs(incoming_pitch_rate) < 0.05) incoming_pitch_rate = 0.0;
    if(abs(incoming_roll_rate) < 0.05) incoming_roll_rate = 0.0;
    if(abs(incoming_yaw_rate) < 0.05) incoming_yaw_rate = 0.0;

    if(abs(incoming_pitch) < 1) incoming_pitch = 0.0;
    if(abs(incoming_roll) < 1) incoming_roll = 0.0;

    cf_pitch = gain * (cf_pitch + (incoming_pitch_rate * RAD_TO_DEG * (nowIMU - lastIMU) / 1000.0)) + ((1.0 - gain) * incoming_pitch);
    cf_roll = gain * (cf_roll + (incoming_roll_rate * RAD_TO_DEG * (nowIMU - lastIMU) / 1000.0)) + ((1.0 - gain) * incoming_roll);  
  
    lastIMU = nowIMU;
  }

  // PID Control
  error_pitch = cf_pitch - desired_pitch;
  error_roll = cf_roll - desired_roll;
  error_yaw = incoming_yaw_rate - desired_yaw_rate;

  sum_error_pitch += 0.5 * (error_pitch + last_error_pitch) * (millis() - lastPIDTime) / 1000.0;
  sum_error_roll += 0.5 * (error_roll + last_error_roll) * (millis() - lastPIDTime) / 1000.0;
  sum_error_yaw += 0.5 * (error_yaw + last_error_yaw) * (millis() - lastPIDTime) / 1000.0;

  derivative_error_pitch = (error_pitch - last_error_pitch) * 1000.0 / (millis() - lastPIDTime + 0.0001);
  derivative_error_roll = (error_roll - last_error_roll) * 1000.0 / (millis() - lastPIDTime + 0.0001);
  derivative_error_yaw = (error_yaw - last_error_yaw) * 1000.0 / (millis() - lastPIDTime + 0.0001);

  last_error_pitch = error_pitch;
  last_error_roll = error_roll;
  last_error_yaw = error_yaw;

  if(Ki_pitch < 0.001) sum_error_pitch = 0.0;
  if(Ki_yaw < 0.001) sum_error_yaw = 0.0;
  if(Ki_roll < 0.001) sum_error_roll = 0.0;
  if(remoteThrottle < 1) {
    sum_error_pitch = 0.0;
    sum_error_yaw = 0.0;
    sum_error_roll = 0.0;
  }

  double control_output_pitch = Kp_pitch * error_pitch + Ki_pitch * sum_error_pitch + Kd_pitch * derivative_error_pitch;
  double control_output_roll = Kp_roll * error_roll + Ki_roll * sum_error_roll + Kd_roll * derivative_error_roll;
  double control_output_yaw = Kp_yaw * error_yaw + Ki_yaw * sum_error_yaw + Kd_yaw * derivative_error_yaw;
  
  lastPIDTime = millis();

  switch (state) {
    case QUAD_RESET:
      spinMotors(0, 0, 0, 0);
      turnOffLEDs();
      break;

    case DISARMED:
      spinMotors(0, 0, 0, 0);
      turnOffLEDs();
      break;

    case ARMED:
      turnOnLEDs();
      spinMotors(remoteThrottle, round(control_output_roll), round(control_output_pitch), round(control_output_yaw));
      break;
  }

  // SEND PACKETS
  if(millis() - timeLastSent > 1000) {
    Packet packetToSend = {MAGIC_NUMBER_QUAD, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, {0}, 0, 0};
    packetToSend.state = state;
    if(resetMode) {
      packetToSend.state = QUAD_RESET;
    }
    packetToSend.batteryVoltage = readBatteryPercentage();
    packetToSend.checksum = wirelessPacket.calculateChecksum(packetToSend);
    wirelessPacket.sendPacket(packetToSend);
    timeLastSent = millis();
  }
}

// void spinMotors(int throttle, int control_output_roll, int control_output_pitch, int control_output_yaw) {
//   if(throttle < 5) {
//     analogWrite(MOTOR1, 0);
//     analogWrite(MOTOR2, 0);
//     analogWrite(MOTOR3, 0);
//     analogWrite(MOTOR4, 0);
//   }
//   else {
//     analogWrite(MOTOR1, min(max(0,throttle + control_output_roll - control_output_pitch - control_output_yaw),255));
//     analogWrite(MOTOR2, min(max(0,throttle + control_output_roll + control_output_pitch + control_output_yaw),255));
//     analogWrite(MOTOR3, min(max(0,throttle - control_output_roll + control_output_pitch - control_output_yaw),255));
//     analogWrite(MOTOR4, min(max(0,throttle - control_output_roll - control_output_pitch + control_output_yaw),255));
//   }

//   // Serial.print(min(max(0,throttle + control_output_roll - control_output_pitch - control_output_yaw),255));
//   // Serial.print(" ");
//   // Serial.print(min(max(0,throttle + control_output_roll + control_output_pitch + control_output_yaw),255));
//   // Serial.print(" ");
//   // Serial.print(min(max(0,throttle - control_output_roll + control_output_pitch - control_output_yaw),255));
//   // Serial.print(" ");
//   // Serial.println(min(max(0,throttle - control_output_roll - control_output_pitch + control_output_yaw),255));

// }

// void spinMotors(int throttle, int control_output_roll, int control_output_pitch, int control_output_yaw) {
//   if(throttle < 5) {
//     analogWrite(MOTOR1, 0);
//     analogWrite(MOTOR2, 0);
//     analogWrite(MOTOR3, 0);
//     analogWrite(MOTOR4, 0);
//   } else {
//     // Calculate the raw values for each motor
//     int motor1_val = throttle + control_output_roll - control_output_pitch - control_output_yaw;
//     int motor2_val = throttle + control_output_roll + control_output_pitch + control_output_yaw;
//     int motor3_val = throttle - control_output_roll + control_output_pitch - control_output_yaw;
//     int motor4_val = throttle - control_output_roll - control_output_pitch + control_output_yaw;

//     // Find the maximum value
//     int max_val = max(max(motor1_val, motor2_val), max(motor3_val, motor4_val));

//     // Check if we need to normalize
//     if (max_val > 255) {
//       float scale = 255.0 / max_val;
//       motor1_val = (int)(motor1_val * scale);
//       motor2_val = (int)(motor2_val * scale);
//       motor3_val = (int)(motor3_val * scale);
//       motor4_val = (int)(motor4_val * scale);
//     }

//     // Write the values to the motors, ensuring they're within the 0-255 range
//     analogWrite(MOTOR1, constrain(motor1_val, 0, 255));
//     analogWrite(MOTOR2, constrain(motor2_val, 0, 255));
//     analogWrite(MOTOR3, constrain(motor3_val, 0, 255));
//     analogWrite(MOTOR4, constrain(motor4_val, 0, 255));
//   }
// }

// void spinMotors(int throttle, int control_output_roll, int control_output_pitch, int control_output_yaw) {
//     if(throttle < 5) {
//         analogWrite(MOTOR1, 0);
//         analogWrite(MOTOR2, 0);
//         analogWrite(MOTOR3, 0);
//         analogWrite(MOTOR4, 0);
//     } else {
//         // Calculate the PID sum for each motor
//         int pid1 = control_output_roll - control_output_pitch - control_output_yaw;
//         int pid2 = control_output_roll + control_output_pitch + control_output_yaw;
//         int pid3 = -control_output_roll + control_output_pitch - control_output_yaw;
//         int pid4 = -control_output_roll - control_output_pitch + control_output_yaw;

//         // Find the maximum and minimum PID values
//         int max_pid = max(max(pid1, pid2), max(pid3, pid4));
//         int min_pid = min(min(pid1, pid2), min(pid3, pid4));

//         // Scale PID values if necessary
//         if (max_pid + throttle > 255 || min_pid + throttle < 0) {
//             float scale_factor;
//             if (max_pid + throttle > 255) {
//                 scale_factor = (255.0 - throttle) / max_pid;
//             } else {
//                 scale_factor = (0.0 - throttle) / min_pid;
//             }
            
//             pid1 *= scale_factor;
//             pid2 *= scale_factor;
//             pid3 *= scale_factor;
//             pid4 *= scale_factor;
//         }

//         // Calculate final motor values and constrain them
//         int motor1_val = constrain(throttle + pid1, 0, 255);
//         int motor2_val = constrain(throttle + pid2, 0, 255);
//         int motor3_val = constrain(throttle + pid3, 0, 255);
//         int motor4_val = constrain(throttle + pid4, 0, 255);

//         // Write the values to the motors
//         analogWrite(MOTOR1, motor1_val);
//         analogWrite(MOTOR2, motor2_val);
//         analogWrite(MOTOR3, motor3_val);
//         analogWrite(MOTOR4, motor4_val);
//     }
// }

void spinMotors(int throttle, int control_output_roll, int control_output_pitch, int control_output_yaw) {
    if (throttle < 5) {
        analogWrite(MOTOR1, 0);
        analogWrite(MOTOR2, 0);
        analogWrite(MOTOR3, 0);
        analogWrite(MOTOR4, 0);
    } else {
        // Calculate the PID sum for each motor
        int pid1 = control_output_roll - control_output_pitch - control_output_yaw;
        int pid2 = control_output_roll + control_output_pitch + control_output_yaw;
        int pid3 = -control_output_roll + control_output_pitch - control_output_yaw;
        int pid4 = -control_output_roll - control_output_pitch + control_output_yaw;

        // Find the maximum PID value
        int max_pid = max(max(pid1, pid2), max(pid3, pid4));

        int adjusted_throttle = throttle;

        // Adjust throttle if necessary to avoid saturation
        if (max_pid + throttle > 255) {
            adjusted_throttle = 255 - max_pid; // Directly calculate the adjusted throttle
        }

        // Calculate final motor values and constrain them
        int motor1_val = constrain(adjusted_throttle + pid1, 0, 255);
        int motor2_val = constrain(adjusted_throttle + pid2, 0, 255);
        int motor3_val = constrain(adjusted_throttle + pid3, 0, 255);
        int motor4_val = constrain(adjusted_throttle + pid4, 0, 255);

        // Write the values to the motors
        analogWrite(MOTOR1, motor1_val);
        analogWrite(MOTOR2, motor2_val);
        analogWrite(MOTOR3, motor3_val);
        analogWrite(MOTOR4, motor4_val);
    }
}


void setupSensor()  {
 if (!lsm.begin_I2C()) {
    Serial.println("Failed to find LSM6DSOX chip");
    while (1) {
      delay(10);
    }
  }
  _accel = lsm.getAccelerometerSensor();
  _gyro = lsm.getGyroSensor();
  ahrs = new Adafruit_Simple_AHRS(_accel, _mag, _gyro);
  lsm.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  lsm.setAccelDataRate(LSM6DS_RATE_208_HZ);
  lsm.setGyroDataRate(LSM6DS_RATE_208_HZ);
  lsm.setAccelCompositeFilter(LSM6DS_CompositeFilter_LPF2, LSM6DS_CompositeFilter_ODR_DIV_20);
}

void turnOnLEDs() {
  digitalWrite(LED_1, HIGH);
  digitalWrite(LED_2, HIGH);
  digitalWrite(LED_3, HIGH);
  digitalWrite(LED_4, HIGH);
  analogWrite(PRETTY_LEDS, 128);
}

void turnOffLEDs() {
  digitalWrite(LED_1, LOW);
  digitalWrite(LED_2, LOW);
  digitalWrite(LED_3, LOW);
  digitalWrite(LED_4, LOW);
  analogWrite(PRETTY_LEDS, 0);
}

int readBatteryPercentage() {
  int raw = analogRead(BAT_SENSE_PIN);
  return map(constrain(raw, lowerBatteryLimit, upperBatteryLimit), lowerBatteryLimit, upperBatteryLimit, 0, 100);
}