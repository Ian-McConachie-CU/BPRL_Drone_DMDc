


#pragma once

/*
  backend driver class for strain sensor class
 */


// #if AP_STRAIN_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include "AP_Strain_I2C.h"
#include <AP_MSP/msp_sensors.h>

class AP_Strain_Backend 
{

    public:
        AP_Strain_Backend(AP_Strain &frontend, uint8_t instance);
        virtual ~AP_Strain_Backend();
        
        // probe and initialise the sensor
        virtual bool init(void) = 0;

        // return the current differential_pressure in Pascal
        virtual bool get_differential_pressure(float &pressure) {return false;}

        // return the current temperature in degrees C, if available
        virtual bool get_temperature(float &temperature) = 0;

        // true if sensor reads airspeed directly, not via pressure
        virtual bool has_airspeed() {return false;}

        // return airspeed in m/s if available
        virtual bool get_airspeed(float& airspeed) {return false;}

        virtual void handle_msp(const MSP::msp_airspeed_data_message_t &pkt) {}


    protected:
        int8_t get_pin(void) const;

        uint8_t get_bus(void) const;

        bool bus_is_configured(void) const;

        uint8_t get_instance(void) const 
        {
            return instance;
        }

        // semaphore for access to shared frontend data
        HAL_Semaphore sem;


        // set to no zero cal, which makes sense for some sensors
        void set_skip_cal(void) 
        {
            #ifndef HAL_BUILD_AP_PERIPH
                    frontend.param[instance].skip_cal.set(1);
            #endif
        }

        // set use
        void set_use(int8_t use) 
        {
            #ifndef HAL_BUILD_AP_PERIPH
                    frontend.param[instance].use.set(use);
            #endif
        }


        // set bus ID of this instance, for ARSPD_DEVID parameters
        void set_bus_id(uint32_t id);

        enum class DevType {
            SITL     = 0x01,
            MS4525   = 0x02,
            MS5525   = 0x03,
            DLVR     = 0x04,
            MSP      = 0x05,
            SDP3X    = 0x06,
            UAVCAN   = 0x07,
            ANALOG   = 0x08,
            NMEA     = 0x09,
            ASP5033  = 0x0A,
        };
        
    private:
        AP_Strain &frontend;
        uint8_t instance;
};

// #endif  // AP_AIRSPEED_ENABLED
