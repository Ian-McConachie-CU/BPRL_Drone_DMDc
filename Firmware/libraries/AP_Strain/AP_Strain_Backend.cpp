#include "AP_Strain_config.h"

#if AP_STRAIN_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Strain_I2C.h"
#include "AP_Strain_Backend.h"

extern const AP_HAL::HAL &hal;

AP_Strain_Backend::AP_Strain_Backend(AP_AStrain &_frontend, uint8_t _instance) :
    frontend(_frontend),
    instance(_instance)
{
}

AP_Strain_Backend::~AP_Strain_Backend(void)
{
}
 

int8_t AP_Strain_Backend::get_pin(void) const
{
#ifndef HAL_BUILD_AP_PERIPH
    return frontend.param[instance].pin;
#else
    return 0;
#endif
}

uint8_t AP_Strain_Backend::get_bus(void) const
{
    return frontend.param[instance].bus;
}

bool AP_Strain_Backend::bus_is_configured(void) const
{
    return frontend.param[instance].bus.configured();
}

void AP_Strain_Backend::set_bus_id(uint32_t id)
{
    frontend.param[instance].bus_id.set_and_save(int32_t(id));
}

#endif  // AP_STRAIN_ENABLED
