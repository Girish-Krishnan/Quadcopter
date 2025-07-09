#include "radio.h"
#include <Wireless_Packet.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <QuadClass_LSM6DSOX.h>
#include <Adafruit_Simple_AHRS.h>
Wireless_Packet wirelessPacket;
#define RAD_TO_DEG 57.295779513082320876798154814105

const int MOTOR1 = 8; // BLUE
const int MOTOR2 = 3; // RED
const int MOTOR3 = 4; // GREEN
const int MOTOR4 = 5; // UN

// PITCH
// Blue and Unmarked are front - negative direction of pitch
// Red and Green are back - positive direction of pitch
const int LED_1 = 16; 
const int LED_2 = 17; 
const int LED_3 = 18; 
const int LED_4 = 36;
const int PRETTY_LEDS = 34;
const int BAT_SENSE_PIN = A7;

const int lowerBatteryLimit = 490;
const int upperBatteryLimit = 887;

 // state
State state = DISARMED;
unsigned long timeLastMessage = millis();
unsigned long timeLastSent = millis();
bool resetMode = true;

// other variables
int remoteThrottle = 0;
int remoteRoll = 0;
int remotePitch = 0;
int remoteYaw = 0;
int countStateChange = 0;
State prevState = DISARMED;

// Create LSM9DS0 board instance.
QuadClass_LSM6DSOX lsm = QuadClass_LSM6DSOX();
Adafruit_Simple_AHRS *ahrs = NULL;
Adafruit_Sensor *_accel = NULL;
Adafruit_Sensor *_gyro = NULL;
Adafruit_Sensor *_mag = NULL;  // No Yaw orientation | Always NULL

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
bool is_calib_IMU = false;
double gain = 0.95;


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

double desired_yaw = 0.0;
double desired_roll = 0.0;
double desired_pitch = 0.0;

double error_roll = 0.0;
double last_error_roll = 0.0;
double sum_error_roll = 0.0;

double error_pitch = 0.0;
double last_error_pitch = 0.0;
double sum_error_pitch = 0.0;

double error_yaw = 0.0;
double last_error_yaw = 0.0;
double sum_error_yaw = 0.0;

unsigned long lastPIDTime = millis();

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
}

unsigned long lastIMU = millis();

void loop() {
  
  // CODE TO READ IMU DATA
  //unsigned long initial_time = millis();
  // quad_data_t orientation;
  // unsigned long nowIMU = millis();
  
  // // COMPLEMENTARY FILTER
  // if ((nowIMU - lastIMU > 0) && (ahrs->getQuadOrientation(&orientation)))
  // {
  //   incoming_pitch_rate = round(orientation.pitch_rate * 100) / 100.0  - IMU_pitch_rate_offset;
  //   incoming_roll_rate = round(orientation.roll_rate * 100) / 100.0  - IMU_roll_rate_offset;
  //   incoming_yaw_rate = round(orientation.yaw_rate * 100) / 100.0  - IMU_yaw_rate_offset;

  //   incoming_pitch = round(orientation.pitch * 100) / 100.0 - IMU_pitch_offset;
  //   incoming_roll = round(orientation.roll * 100) / 100.0 - IMU_roll_offset;

  //   pitch_angle = pitch_angle + (incoming_pitch_rate * RAD_TO_DEG * (nowIMU - lastIMU) / 1000.0);
  //   roll_angle = roll_angle + (incoming_roll_rate * RAD_TO_DEG * (nowIMU - lastIMU) / 1000.0);

  //   cf_pitch = gain * (cf_pitch + (incoming_pitch_rate * RAD_TO_DEG * (nowIMU - lastIMU) / 1000.0)) + ((1.0 - gain) * incoming_pitch);
  //   cf_roll = gain * (cf_roll + (incoming_roll_rate * RAD_TO_DEG * (nowIMU - lastIMU) / 1000.0)) + ((1.0 - gain) * incoming_roll);
  //   // Serial.print(incoming_pitch_rate);
  //   // Serial.print(F(" "));
  //   // Serial.print(incoming_pitch);
  //   // Serial.print(F(" "));
  //   // Serial.println(cf_pitch);       
  
  //   lastIMU = nowIMU;
  // }

  // error_pitch = cf_pitch - desired_pitch;
  // sum_error_pitch = sum_error_pitch + error_pitch * (double)(millis() - lastPIDTime) / 1000.0;

  // error_roll = cf_roll - desired_roll;
  // sum_error_roll = sum_error_roll + error_roll * (double)(millis() - lastPIDTime) / 1000.0;
  
  // if(abs(desired_yaw) < 3) {
  //   desired_yaw = 0.0;
  // }

  // error_yaw = incoming_yaw_rate - desired_yaw;
  // // Serial.println(error_yaw);
  // if(abs(error_yaw) < 0.03) {
  //   error_yaw = 0.0;
  // }
  // sum_error_yaw = sum_error_yaw + error_yaw * (double)(millis() - lastPIDTime) / 1000.0;

  // double control_output_pitch = Kp_pitch * error_pitch + Ki_pitch * sum_error_pitch + Kd_pitch * (error_pitch - last_error_pitch) * 1000.0 / ((double)(millis() - lastPIDTime) + 0.01);
  // double control_output_yaw = Kp_yaw * error_yaw + Ki_yaw * sum_error_yaw + Kd_yaw * (error_yaw - last_error_yaw) * 1000.0 / ((double)(millis() - lastPIDTime) + 0.01);
  // double control_output_roll = Kp_roll * error_roll + Ki_roll * sum_error_roll + Kd_roll * (error_roll - last_error_roll) * 1000.0 / ((double)(millis() - lastPIDTime) + 0.01);

  // Serial.print(error_roll);
  // Serial.print(F(" "));
  // Serial.print(sum_error_roll);
  // Serial.print(F(" "));
  // Serial.println(last_error_roll);
  
  // last_error_roll = error_roll;
  // last_error_pitch = error_pitch;
  // last_error_yaw = error_yaw;

  // lastPIDTime = millis();
  // if(state == ARMED) spinMotors(remoteThrottle, int(control_output_roll), int(control_output_pitch), int(control_output_yaw));

  // // Ki windup issues
  // if(Ki_pitch < 0.001) sum_error_pitch = 0.0;
  // if(Ki_yaw < 0.001) sum_error_yaw = 0.0;
  // if(Ki_roll < 0.001) sum_error_roll = 0.0;
  // if(remoteThrottle < 0.001) {
  //   sum_error_pitch = 0.0;
  //   sum_error_yaw = 0.0;
  //   sum_error_roll = 0.0;
  // }

  // CODE TO SEND AND RECEIVE PACKETS
  // if(millis() - timeLastSent > 1000) {
  //   Packet packetToSend = {MAGIC_NUMBER_QUAD, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, {0}, 0, 0};
  //   packetToSend.state = state;
  //   if(resetMode) {
  //     packetToSend.state = QUAD_RESET;
  //   }
  //   packetToSend.batteryVoltage = readBatteryPercentage();
  //   packetToSend.Kp_yaw = desired_yaw;
  //   packetToSend.Ki_yaw = incoming_yaw_rate;
  //   packetToSend.Kd_yaw = error_yaw;
  //   packetToSend.checksum = wirelessPacket.calculateChecksum(packetToSend);
  //   wirelessPacket.sendPacket(packetToSend);
  //   timeLastSent = millis();
  // }

  // if(millis() - timeLastMessage > 150) {
  //   Packet receivedPacket;
  //   if (wirelessPacket.receivePacket(receivedPacket, false)) {
  //     Serial.println("RECIEVED");
  //     resetMode = false;
  //     timeLastMessage = millis();
  //     if(prevState != (State) receivedPacket.state) {
  //       countStateChange++;
  //       if(countStateChange >= 8) {
  //         state = (State) receivedPacket.state;
  //         prevState = (State) receivedPacket.state;
  //         countStateChange = 0;
  //       }
  //     }
  //     else {
  //       countStateChange = 0;
  //     }
      
  //     //state = (State) receivedPacket.state;

  //     remoteThrottle = receivedPacket.throttle;
  //     remoteRoll = receivedPacket.roll;
  //     remotePitch = receivedPacket.pitch;
  //     remoteYaw = receivedPacket.yaw;
  //     desired_pitch = double(remotePitch - 130) * 90.0 / 130.0;
  //     desired_roll = double(remoteRoll - 130) * 90.0 / 130.0;
  //     desired_yaw = double(remoteYaw - 130) * 90.0 / 130.0;

  //     int deltaPitchOffset = receivedPacket.offsets[0];
  //     int deltaRollOffset = receivedPacket.offsets[1];
  //     int deltaPitchRateOffset = receivedPacket.offsets[2];
  //     int deltaRollRateOffset = receivedPacket.offsets[3];
  //     int deltaYawRateOffset = receivedPacket.offsets[4];

  //     Kp_roll = receivedPacket.Kp_roll;
  //     Ki_roll = receivedPacket.Ki_roll;
  //     Kd_roll = receivedPacket.Kd_roll;

  //     Kp_pitch = receivedPacket.Kp_pitch;
  //     Ki_pitch = receivedPacket.Ki_pitch;
  //     Kd_pitch = receivedPacket.Kd_pitch;

  //     Kp_yaw = receivedPacket.Kp_yaw;
  //     Ki_yaw = receivedPacket.Ki_yaw;
  //     Kd_yaw = receivedPacket.Kd_yaw;

  //     if(deltaPitchOffset == 1) IMU_pitch_offset = IMU_pitch_offset + 0.01;
  //     if(deltaPitchOffset == -1) IMU_pitch_offset = IMU_pitch_offset - 0.01;
  //     if(deltaRollOffset == 1) IMU_roll_offset = IMU_roll_offset + 0.01;
  //     if(deltaRollOffset == -1) IMU_roll_offset = IMU_roll_offset - 0.01;
  //     if(deltaPitchRateOffset == 1) IMU_pitch_rate_offset = IMU_pitch_rate_offset + 0.01;
  //     if(deltaPitchRateOffset == -1) IMU_pitch_rate_offset = IMU_pitch_rate_offset - 0.01;
  //     if(deltaRollRateOffset == 1) IMU_roll_rate_offset = IMU_roll_rate_offset + 0.01;
  //     if(deltaRollRateOffset == -1) IMU_roll_rate_offset = IMU_roll_rate_offset - 0.01;
  //     if(deltaYawRateOffset == 1) IMU_yaw_rate_offset = IMU_yaw_rate_offset + 0.01;
  //     if(deltaYawRateOffset == -1) IMU_yaw_rate_offset = IMU_yaw_rate_offset - 0.01;
  //   }
  // }

  // else {
  //   if (millis() - timeLastMessage > 5000) {
  //     timeLastMessage = millis();
  //     state = DISARMED;
  //   }
  // }

  /*

  switch (state) {
    case QUAD_RESET:
      spinMotors(0, 0, 0, 0);
      turnOffLEDs();
      is_calib_IMU = false;
      IMU_pitch_offset = 0.0;
      IMU_roll_offset = 0.0;
      IMU_pitch_rate_offset = 0.0;
      IMU_roll_rate_offset = 0.0;
      IMU_yaw_rate_offset = 0.0;
      break;

    case DISARMED:
      spinMotors(0, 0, 0, 0);
      turnOffLEDs();
      is_calib_IMU = false;
      IMU_pitch_offset = 0.0;
      IMU_roll_offset = 0.0;
      IMU_pitch_rate_offset = 0.0;
      IMU_roll_rate_offset = 0.0;
      IMU_yaw_rate_offset = 0.0;
      break;

    case ARMED:
      turnOnLEDs();
      if(!is_calib_IMU && (remoteThrottle > 100)) { 

        double avg_incoming_pitch_rate = 0.0;
        double avg_incoming_pitch = 0.0;
        double avg_incoming_roll_rate = 0.0;
        double avg_incoming_roll = 0.0;
        double avg_incoming_yaw_rate = 0.0;
        
        for(int j = 0; j < 100; j++) {
          ahrs->getQuadOrientation(&orientation);
          incoming_pitch_rate = round(orientation.pitch_rate * 100) / 100.0  - IMU_pitch_rate_offset;
          incoming_roll_rate = round(orientation.roll_rate * 100) / 100.0  - IMU_roll_rate_offset;
          incoming_pitch = round(orientation.pitch * 100) / 100.0 - IMU_pitch_offset;
          incoming_roll = round(orientation.roll * 100) / 100.0 - IMU_roll_offset;
          incoming_yaw_rate = round(orientation.yaw_rate * 100) / 100.0  - IMU_yaw_rate_offset;
          cf_pitch = gain * (cf_pitch + (incoming_pitch_rate * RAD_TO_DEG * (10) / 1000.0)) + ((1.0 - gain) * incoming_pitch);

          avg_incoming_pitch_rate = avg_incoming_pitch_rate + incoming_pitch_rate;
          avg_incoming_roll_rate = avg_incoming_roll_rate + incoming_roll_rate;
          avg_incoming_yaw_rate = avg_incoming_yaw_rate + incoming_yaw_rate;
          avg_incoming_pitch = avg_incoming_pitch + cf_pitch;
          avg_incoming_roll = avg_incoming_roll + incoming_roll;

          delay(10);
        }

        avg_incoming_pitch_rate = avg_incoming_pitch_rate / 100.0;
        avg_incoming_roll_rate = avg_incoming_roll_rate / 100.0;
        avg_incoming_pitch = avg_incoming_pitch / 100.0;
        avg_incoming_roll = avg_incoming_roll / 100.0;
        avg_incoming_yaw_rate  = avg_incoming_yaw_rate / 100.0;

        IMU_pitch_offset = avg_incoming_pitch + IMU_pitch_offset;
        IMU_roll_offset = avg_incoming_roll + IMU_roll_offset;
        IMU_pitch_rate_offset = avg_incoming_pitch_rate + IMU_pitch_rate_offset;
        IMU_roll_rate_offset = avg_incoming_roll_rate + IMU_roll_rate_offset;

        //if(remoteThrottle >= 10) IMU_yaw_rate_offset = avg_incoming_yaw_rate + IMU_yaw_rate_offset;

        is_calib_IMU = true;
        Serial.println("CALIBRATED VALUES:");
        Serial.print(IMU_pitch_offset);
        Serial.print(F(" "));
        Serial.print(IMU_roll_offset);
        Serial.print(F(" "));
        Serial.print(IMU_pitch_rate_offset);
        Serial.print(F(" "));
        Serial.print(IMU_roll_rate_offset);
        Serial.print(F(" "));
        Serial.println(IMU_yaw_rate_offset);
      }
      
      break;
  }

  //Serial.println(millis() - initial_time);
  */

  digitalWrite(LED_1, HIGH);
  // digitalWrite(LED_2, HIGH);
  // digitalWrite(LED_4, HIGH);
  // digitalWrite(LED_4, HIGH);

  // digitalWrite(LED_1, LOW);
  //digitalWrite(LED_2, LOW);
  // digitalWrite(LED_3, LOW);
  // digitalWrite(LED_4, LOW);

  // analogWrite(MOTOR1, 5);
  // analogWrite(MOTOR2, 5);
  // analogWrite(MOTOR3, 5);
  // analogWrite(MOTOR4, 5);
  // analogWrite(PRETTY_LEDS, 128);

  
}

void spinMotors(int throttle, int control_output_roll, int control_output_pitch, int control_output_yaw) {
  analogWrite(MOTOR1, min(max(0,throttle + control_output_roll - control_output_pitch + control_output_yaw),255));
  analogWrite(MOTOR2, min(max(0,throttle + control_output_roll + control_output_pitch - control_output_yaw),255));
  analogWrite(MOTOR3, min(max(0,throttle - control_output_roll + control_output_pitch + control_output_yaw),255));
  analogWrite(MOTOR4, min(max(0,throttle - control_output_roll - control_output_pitch - control_output_yaw),255));

  // Serial.print(min(max(0,throttle + control_output_roll - control_output_pitch + control_output_yaw),255));
  // Serial.print(" ");
  // Serial.print(min(max(0,throttle + control_output_roll + control_output_pitch - control_output_yaw),255));
  // Serial.print(" ");
  // Serial.print(min(max(0,throttle - control_output_roll + control_output_pitch + control_output_yaw),255));
  // Serial.print(" ");
  // Serial.println(min(max(0,throttle - control_output_roll - control_output_pitch - control_output_yaw),255));
}

void blinkTopLEDs() {
  static unsigned long lastTime = 0;
  static bool on = false;
  if (millis() - lastTime > 1000) {
    lastTime = millis();
    on = !on;
    digitalWrite(LED_1, on);
    digitalWrite(LED_2, on);
    digitalWrite(LED_3, on);
    digitalWrite(LED_4, on);
  }
}

void blinkBottomLEDs() {
  static unsigned long lastTime = 0;
  for (int i = 0; i < 255; i++) {
    if (millis() - lastTime > 100) {
      lastTime = millis();
      analogWrite(PRETTY_LEDS, i);
    }
  }
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

void setupSensor()
{
  // Set data rate for G and XL.  Set G low-pass cut off.  (Section 7.12)
 // lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG1_G,  ODR_952 | G_BW_G_10 );  //952hz ODR + 63Hz cuttof

  // Enable the XL (Section 7.23)
  //lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG5_XL, XL_ENABLE_X | XL_ENABLE_Y | XL_ENABLE_Z);

  // Set low-pass XL filter frequency divider (Section 7.25)
  //lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG7_XL, HR_MODE | XL_LP_ODR_RATIO_9);
  

  // This only sets range of measurable values for each sensor.  Setting these manually (I.e., without using these functions) will cause incorrect output from the library.
 // lsm.setupAccel(Adafruit_LSM9DS1::LSM9DS1_ACCELRANGE_2G);
  //lsm.setupMag(Adafruit_LSM9DS1::LSM9DS1_MAGGAIN_4GAUSS);
 // lsm.setupGyro(Adafruit_LSM9DS1::LSM9DS1_GYROSCALE_245DPS);

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