/*
 *   auto_calibration.cpp - airspeed auto calibration
 *
 * Algorithm by Paul Riseborough
 *
 */

#include "AP_Strain_config.h"

#if AP_STRAIN_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>

#include "AP_Strain_I2C.h"


// constructor - fill in all the initial values
Strain_Calibration::Strain_Calibration()
    : P(100,   0,         0,
        0,   100,         0,
        0,     0,  0.000001f)

    , Q0(0.01f)

    , Q1(0.0000005f)

    , state(0, 0, 0)

    , DT(1)
{

}


void Strain_Calibration::init(float initial_ratio)
{
    state.z = 1.0f / sqrtf(initial_ratio);
}

/*
  update the state of the airspeed calibration - needs to be called
  once a second
 */
float Strain_Calibration::update()
{
   
}

#endif  // AP_AIRSPEED_ENABLED
