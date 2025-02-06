
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Strain_config.h"

#if AP_STRAIN_ENABLED

    #include "AP_Strain_I2C.h"

    #include <AP_Vehicle/AP_Vehicle_Type.h>


    AP_Strain_Params::AP_Strain_Params(void) {};
    const AP_Param::GroupInfo AP_Strain_Params::var_info[] = { AP_GROUPEND };


#endif  // AP_AIRSPEED_ENABLED
