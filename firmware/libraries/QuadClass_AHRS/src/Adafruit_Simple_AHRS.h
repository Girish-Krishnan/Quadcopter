/*!
 * @file Adafruit_Simple_AHRS.cpp
 */
#ifndef __ADAFRUIT_SIMPLE_AHRS_H__
#define __ADAFRUIT_SIMPLE_AHRS_H__

#include "Adafruit_Sensor_Set.h"
#include <Adafruit_Sensor.h>

typedef struct {
     float roll;
     float pitch;

     float roll_rate;
     float pitch_rate;
     float yaw_rate;
} quad_data_t;

#define AHRS_FLIP_X B00000001
#define AHRS_FLIP_Y B00000010
#define AHRS_FLIP_Z B00000100

/*!
 * @brief Simple sensor fusion AHRS using an accelerometer and magnetometer.
 */
class Adafruit_Simple_AHRS {
public:
  /**************************************************************************/
  /*!
   * @brief Create a simple AHRS from a device with multiple sensors.
   *
   * @param accelerometer The accelerometer to use for this sensor fusion.
   * @param magnetometer The magnetometer to use for this sensor fusion.
   */
  /**************************************************************************/
  Adafruit_Simple_AHRS(Adafruit_Sensor *accelerometer,
                       Adafruit_Sensor *magnetometer,
		       Adafruit_Sensor *gyroscope);

  /**************************************************************************/
  /*!
   * @brief Create a simple AHRS from a device with multiple sensors.
   *
   * @param sensors A set of sensors containing the accelerometer and
   * magnetometer for this sensor fusion.
   */
  /**************************************************************************/
  Adafruit_Simple_AHRS(Adafruit_Sensor_Set &sensors);

  /**************************************************************************/
  /*!
   * @brief Compute orientation based on accelerometer and magnetometer data.
   *
   * @return Whether the orientation was computed.
   */
  /**************************************************************************/
  bool getOrientation(sensors_vec_t *orientation);

  bool getQuadOrientation(quad_data_t* orientation, int orientation_adjustment = 0);

private:
  Adafruit_Sensor *_accel;
  Adafruit_Sensor *_mag;
  Adafruit_Sensor *_gyro;
};

#endif
