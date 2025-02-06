#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_MSP/AP_MSP_config.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS_config.h>

#ifndef AP_STRAIN_ENABLED
#define AP_STRAIN_ENABLED 1
#endif

#ifndef AP_STRAIN_BACKEND_DEFAULT_ENABLED
#define AP_STRAIN_BACKEND_DEFAULT_ENABLED AP_STRAIN_ENABLED
#endif
