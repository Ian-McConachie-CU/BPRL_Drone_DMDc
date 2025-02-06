#pragma once

#include "AP_Strain_config.h"

#if AP_STRAIN_ENABLED
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>


class AP_Strain_backend;

class AP_Strain_Params {
public:
    // Constructor
    AP_Strain_Params(void);

    // parameters for each instance
    AP_Int32 bus_id;

#ifndef HAL_BUILD_AP_PERIPH
    AP_Float offset;
    AP_Int8  use;
    AP_Int8  pin;
#endif
    AP_Int8  type;
    AP_Int8  bus;

    static const struct AP_Param::GroupInfo var_info[];
};


class Strain_Calibration {
public:
    friend class AP_Strain;
    // constructor
    Strain_Calibration();

    // initialise the calibration
    void init(float initial_ratio);

    float update();

private:

};

class AP_Strain_I2C
{
public:
    friend class AP_Strain_Backend;
    
    // constructor
    AP_Strain_I2C();

    void init(void);
    void allocate();

    // indicate which bit in LOG_BITMASK indicates we should log airspeed readings
    void set_log_bit(uint32_t log_bit) { _log_bit = log_bit; }

    // read the analog source and update airspeed
    void update(void);

    // calibrate the airspeed. This must be called on startup if the
    // altitude/climb_rate/acceleration interfaces are ever used
    void calibrate(bool in_startup);

    // return the current airspeed in m/s
    float get_strain(uint8_t i) const;
    float get_strain(void) const { return get_strain(primary); }


    // return true if airspeed is enabled
    bool enabled(uint8_t i) const;
    bool enabled(void) const { return enabled(primary); }

    // update airspeed ratio calibration
    void update_calibration(const Vector3f &vground, int16_t max_airspeed_allowed_during_cal);

    // return health status of sensor
    bool healthy(uint8_t i) const;
    bool healthy(void) const { return healthy(primary); }

    // return true if all enabled sensors are healthy
    bool all_healthy(void) const;
    
    // return time in ms of last update
    uint32_t last_update_ms(uint8_t i) const { return state[i].last_update_ms; }
    uint32_t last_update_ms(void) const { return last_update_ms(primary); }


    static const struct AP_Param::GroupInfo var_info[];

};


#endif  // AP_AIRSPEED_ENABLED
