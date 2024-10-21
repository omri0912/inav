#pragma once

#define WORK_WITHOUT_RC_FROM_CLI    0 // work without RC from CLI 
#define BLACKBOX_USING_DEBUG        0

//#define VGPS                        // do  waypints navigation mission where the dorne flies 5m to a wall and back streaming virutal GPS data (UBLOX protocol) 

#define SWITCH_OPFLOW_EVERY_10SEC   0 // [1] every 10 seconds switch between down/forwards facing opflow 
#define MUX_FOR_OPFLOW_SWITCH       1 // [1] use swtich 49 to enable forward facing opflow (default down facing, must set SWITCH_OPFLOW_EVERY_10SEC to 0)
#define SCALE_ALTITUDE_AT_ALTHOLD   1 // [1] during surface navigation - adjust the throttle span when entering alt hold to achieve smooth transition 
#define DISABLE_GPS_AT_ALTHOLD      0 // [1] when swtiching to alt hold disable GPS and continue using surface navigation 

#include "io/serial.h"

void mtf_01_init(void);
void mtf_01_micolink_decode(serialPortIdentifier_e identifier, uint8_t data);
bool mtf_01_is_micolink(void);
bool mtf_01_is_facing_down(void);
float mtf_01_get_move_cm(int i);

#if DISABLE_GPS_AT_ALTHOLD
void flyz_gps_refresh(bool is_surface_enabled);
void flyz_set_agl(int32_t gps_agl_guess_cm);
bool flyz_is_gps_enable(void);
#endif