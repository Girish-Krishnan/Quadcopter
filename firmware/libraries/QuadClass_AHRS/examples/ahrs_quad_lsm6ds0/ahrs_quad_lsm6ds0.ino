#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <QuadClass_LSM6DSOX.h>
#include <Adafruit_Simple_AHRS.h>

// Create LSM9DS0 board instance.
QuadClass_LSM6DSOX lsm = QuadClass_LSM6DSOX();
Adafruit_Simple_AHRS *ahrs = NULL;
Adafruit_Sensor *_accel = NULL;
Adafruit_Sensor *_gyro = NULL;
Adafruit_Sensor *_mag = NULL;  // No Yaw orientation | Always NULL

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
#
}

void setup(void) 
{
  Serial.begin(115200);
  Serial.println(F("Adafruit LSM9DS1 9 DOF Board AHRS Example")); Serial.println("");
  
 
  // Setup the sensor gain and integration time.
  setupSensor();
}

unsigned int last = millis();
void loop(void) 
{
  quad_data_t orientation;

  int now = millis();
  
  
  // Use the simple AHRS function to get the current orientation.
  if (ahrs->getQuadOrientation(&orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    Serial.print(now - last);
    Serial.print(F(" "));
    Serial.print(orientation.roll);
    Serial.print(F(" "));
    Serial.print(orientation.pitch);
    Serial.print(F(" "));
    Serial.print(orientation.roll_rate);
    Serial.print(F(" "));
    Serial.print(orientation.pitch_rate);
    Serial.print(F(" "));
    Serial.print(orientation.yaw_rate);
    Serial.println(F(""));
  }

  last = now;
}
