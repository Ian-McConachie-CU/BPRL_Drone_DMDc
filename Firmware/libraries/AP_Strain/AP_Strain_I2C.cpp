#include "AP_Airspeed_config.h"

#if AP_STRAIN_ENABLED

#include "AP_Strain_I2C.h"

#include <AP_Vehicle/AP_Vehicle_Type.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/AP_Logger.h>
#include <utility>

extern const AP_HAL::HAL &hal;


#include <AP_Vehicle/AP_FixedWing.h>



AP_Strain_I2C::AP_Strain_I2C()
{
    AP_Param::setup_object_defaults(this, var_info);

    // Setup defaults that only apply to first sensor
    param[0].type.set_default(ARSPD_DEFAULT_TYPE);
#ifndef HAL_BUILD_AP_PERIPH
    param[0].bus.set_default(HAL_AIRSPEED_BUS_DEFAULT);
    param[0].pin.set_default(ARSPD_DEFAULT_PIN);
#endif

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Airspeed must be singleton");
    }
    _singleton = this;
}


// macro for use by HAL_INS_PROBE_LIST
#define GET_I2C_DEVICE(bus, address) hal.i2c_mgr->get_device(bus, address)

bool AP_Strain_I2C::add_backend(AP_Strain_Backend *backend)
{
    if (!backend) {
        return false;
    }
    if (num_sensors >= AIRSPEED_MAX_SENSORS) {
        AP_HAL::panic("Too many strain drivers");
    }
    const uint8_t i = num_sensors;
    sensor[num_sensors++] = backend;
    if (!sensor[i]->init()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Strain %u init failed", i+1);
        delete sensor[i];
        sensor[i] = nullptr;
    }
    return true;
}

/*
  macro to add a backend with check for too many sensors
  We don't try to start more than the maximum allowed
 */
#define ADD_BACKEND(backend) \
    do { add_backend(backend);     \
    if (num_sensors == AIRSPEED_MAX_SENSORS) { return; } \
    } while (0)


void AP_Strain_I2C::init()
{

    convert_per_instance();

#if ENABLE_PARAMETER
    // if either type is set then enable if not manually set
    if (!_enable.configured() && ((param[0].type.get() != TYPE_NONE) || (param[1].type.get() != TYPE_NONE))) {
        _enable.set_and_save(1);
    }

    // Check if enabled
    if (!lib_enabled()) {
        return;
    }
#endif

    if (enabled(0)) {
        allocate();
    }
}

void AP_Strain_I2C::allocate()
{

}


// calibrate the zero offset for the airspeed. This must be called at
// least once before the get_airspeed() interface can be used
void AP_Strain_I2C::calibrate(bool in_startup)
{
#ifndef HAL_BUILD_AP_PERIPH
    if (!lib_enabled()) {
        return;
    }
    if (hal.util->was_watchdog_reset()) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"Airspeed: skipping cal");
        return;
    }
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        if (!enabled(i)) {
            continue;
        }
        if (state[i].use_zero_offset) {
            param[i].offset.set(0);
            continue;
        }
        if (in_startup && param[i].skip_cal) {
            continue;
        }
        if (sensor[i] == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Airspeed %u not initialized, cannot cal", i+1);
            continue;
        }
        state[i].cal.start_ms = AP_HAL::millis();
        state[i].cal.count = 0;
        state[i].cal.sum = 0;
        state[i].cal.read_count = 0;
        calibration_state[i] = CalibrationState::IN_PROGRESS;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"Airspeed %u calibration started", i+1);
    }
#endif // HAL_BUILD_AP_PERIPH
}


// read all airspeed sensors
void AP_Strain_I2C::update()
{
    if (!lib_enabled()) {
        return;
    }

    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        read(i);
    }

#if HAL_GCS_ENABLED
    // debugging until we get MAVLink support for 2nd airspeed sensor
    if (enabled(1)) {
        gcs().send_named_float("AS2", get_airspeed(1));
    }
#endif

#if HAL_LOGGING_ENABLED
    const uint8_t old_primary = primary;
#endif

    // setup primary
    if (healthy(primary_sensor.get())) {
        primary = primary_sensor.get();
    } else {
        for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
            if (healthy(i)) {
                primary = i;
                break;
            }
        }
    }

    check_sensor_failures();

#if HAL_LOGGING_ENABLED
    if (primary != old_primary) {
        AP::logger().Write_Event(LogEvent::AIRSPEED_PRIMARY_CHANGED);
    }
    if (_log_bit != (uint32_t)-1 && AP::logger().should_log(_log_bit)) {
        Log_Airspeed();
    }
#endif
}


#if HAL_LOGGING_ENABLED
// @LoggerMessage: HYGR
// @Description: Hygrometer data
// @Field: TimeUS: Time since system startup
// @Field: Id: sensor ID
// @Field: Humidity: percentage humidity
// @Field: Temp: temperature in degrees C

void AP_Strain_I2C::Log_Strain()
{
    const uint64_t now = AP_HAL::micros64();
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) 
    {
        if (!enabled(i) || sensor[i] == nullptr) {
            continue;
        }
        const struct log_ARSP pkt
        {
            LOG_PACKET_HEADER_INIT(LOG_ARSP_MSG),
            time_us       : now,
            instance      : i
            /////////////////////////////////////////////////////////add logging
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));

    }
}
#endif


/*
  return true if all enabled sensors are healthy
 */
bool AP_Strain_I2C::all_healthy(void) const
{
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        if (enabled(i) && !healthy(i)) {
            return false;
        }
    }
    return true;
}

bool AP_Strain_I2C::lib_enabled() const {
#if ENABLE_PARAMETER
    return _enable > 0;
#endif
    return true;
}

// return true if airspeed is enabled
bool AP_Strain_I2C::enabled(uint8_t i) const {
    if (!lib_enabled()) {
        return false;
    }
    if (i < AIRSPEED_MAX_SENSORS) {
        return param[i].type.get() != TYPE_NONE;
    }
    return false;
}

// return health status of sensor
bool AP_Strain_I2C::healthy(uint8_t i) const {
    if (!enabled(i)) {
        return false;
    }
    bool ok = state[i].healthy && sensor[i] != nullptr;
#ifndef HAL_BUILD_AP_PERIPH
    // sanity check the offset parameter.  Zero is permitted if we are skipping calibration.
    ok &= (fabsf(param[i].offset) > 0 || state[i].use_zero_offset || param[i].skip_cal);
#endif
    return ok;
}

#endif  // AP_STRAIN_ENABLED
