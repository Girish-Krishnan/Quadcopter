#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Simple_AHRS.h>

// Create LSM9DS0 board instance.
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
// Create simple AHRS algorithm using the LSM9DS0 instance's accelerometer and magnetometer.
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag(), &lsm.getGyro());

void setupSensor()
{
  // Set data rate for G and XL.  Set G low-pass cut off.  (Section 7.12)
  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG1_G,  ODR_952 | G_BW_G_10 );  //952hz ODR + 63Hz cuttof

  // Enable the XL (Section 7.23)
  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG5_XL, XL_ENABLE_X | XL_ENABLE_Y | XL_ENABLE_Z);

  // Set low-pass XL filter frequency divider (Section 7.25)
  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG7_XL, HR_MODE | XL_LP_ODR_RATIO_9);
  
  // enable mag continuous (Section 8.7)
  lsm.write8(MAGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG3_M, B00000000); // continuous mode

  // This only sets range of measurable values for each sensor.  Setting these manually (I.e., without using these functions) will cause incorrect output from the library.
  lsm.setupAccel(Adafruit_LSM9DS1::LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(Adafruit_LSM9DS1::LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(Adafruit_LSM9DS1::LSM9DS1_GYROSCALE_245DPS);
}

void setup(void) 
{
  Serial.begin(115200);
  Serial.println(F("Adafruit LSM9DS1 9 DOF Board AHRS Example")); Serial.println("");
  
  // Initialise the LSM9DS0 board.
  if(!lsm.begin())
  {
    // There was a problem detecting the LSM9DS0 ... check your connections
    Serial.print(F("Ooops, no LSM9DS1 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  
  // Setup the sensor gain and integration time.
  setupSensor();
}

unsigned int last = millis();
void loop(void) 
{
  quad_data_t orientation;

  int now = millis();
  
  // Use the simple AHRS function to get the current orientation.
  if (ahrs.getQuadOrientation(&orientation))
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
