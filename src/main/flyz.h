#pragma once

#define WORK_WITHOUT_RC_FROM_CLI                0 // work without RC from CLI 
#define BLACKBOX_USING_DEBUG                    0

//#define VGPS                        // do  waypints navigation mission where the dorne flies 5m to a wall and back streaming virutal GPS data (UBLOX protocol) 

#define FLYZ_VER_MAJOR                          4
#define FLYZ_VER_MINOR                          5
#define FLYZ_VER(MAJOR,MINOR)                   ((FLYZ_VER_MAJOR)==(MAJOR) && (MINOR)==(FLYZ_VER_MINOR))

#if FLYZ_VER(4,5) /* 4/11 - gradual fade out of satelites over 10 sec to rate_accel_limit_roll_pitch or 100+rate_accel_limit_roll_pitch */
#define SWITCH_OPFLOW_EVERY_10SEC               0 
#define MUX_FOR_OPFLOW_SWITCH                   0 
#define SCALE_ALTITUDE_AT_ALTHOLD               1 
#define USE_ABS_POS_WHEN_ENTERING_SURFACE_HOLD  1   
#define DISABLE_GPS_AT_ALTHOLD                  4
#define PITCH_AT_ALTHOLD                        1 
#define INAV_BODY2EARTH_FRAME                   0
#elif FLYZ_VER(4,4) /* 4/11 - gradual fade out of satelites over 10 sec */
#define SWITCH_OPFLOW_EVERY_10SEC               0 
#define MUX_FOR_OPFLOW_SWITCH                   0 
#define SCALE_ALTITUDE_AT_ALTHOLD               1 
#define USE_ABS_POS_WHEN_ENTERING_SURFACE_HOLD  1   
#define DISABLE_GPS_AT_ALTHOLD                  5
#define PITCH_AT_ALTHOLD                        1 
#define INAV_BODY2EARTH_FRAME                   1
#elif FLYZ_VER(4,3) /* 2/11 - gradual fade out of satelites over 10 sec */
#define SWITCH_OPFLOW_EVERY_10SEC               0 
#define MUX_FOR_OPFLOW_SWITCH                   0 
#define SCALE_ALTITUDE_AT_ALTHOLD               1 
#define USE_ABS_POS_WHEN_ENTERING_SURFACE_HOLD  1   
#define DISABLE_GPS_AT_ALTHOLD                  4
#define PITCH_AT_ALTHOLD                        1 
#define INAV_BODY2EARTH_FRAME                   1 
#elif FLYZ_VER(4,2) /* 2/11 - test how move from gps to surface pos-hold works but now with inav earth frame */
#define SWITCH_OPFLOW_EVERY_10SEC               0 
#define MUX_FOR_OPFLOW_SWITCH                   0 
#define SCALE_ALTITUDE_AT_ALTHOLD               1 
#define USE_ABS_POS_WHEN_ENTERING_SURFACE_HOLD  1   
#define DISABLE_GPS_AT_ALTHOLD                  1
#define PITCH_AT_ALTHOLD                        1 
#define INAV_BODY2EARTH_FRAME                   1 
#elif FLYZ_VER(4,1) /* 2/11 - test how suddent disable of gps affect pos hold */
#define SWITCH_OPFLOW_EVERY_10SEC               0 
#define MUX_FOR_OPFLOW_SWITCH                   0 
#define SCALE_ALTITUDE_AT_ALTHOLD               1 
#define USE_ABS_POS_WHEN_ENTERING_SURFACE_HOLD  1   
#define DISABLE_GPS_AT_ALTHOLD                  3
#define PITCH_AT_ALTHOLD                        1 
#define INAV_BODY2EARTH_FRAME                   0 
#elif FLYZ_VER(3,1)
#define SWITCH_OPFLOW_EVERY_10SEC               0 
#define MUX_FOR_OPFLOW_SWITCH                   0 
#define SCALE_ALTITUDE_AT_ALTHOLD               1 
#define USE_ABS_POS_WHEN_ENTERING_SURFACE_HOLD  1   
#define DISABLE_GPS_AT_ALTHOLD                  2
#define PITCH_AT_ALTHOLD                        1 
#define INAV_BODY2EARTH_FRAME                   0 
#elif FLYZ_VER(2,8)
#define SWITCH_OPFLOW_EVERY_10SEC               0 
#define MUX_FOR_OPFLOW_SWITCH                   0 
#define SCALE_ALTITUDE_AT_ALTHOLD               1 
#define USE_ABS_POS_WHEN_ENTERING_SURFACE_HOLD  1   
#define DISABLE_GPS_AT_ALTHOLD                  2
#define PITCH_AT_ALTHOLD                        1 
#define INAV_BODY2EARTH_FRAME                   0 
#elif FLYZ_VER(2,7)
#define SWITCH_OPFLOW_EVERY_10SEC               0 
#define MUX_FOR_OPFLOW_SWITCH                   0 
#define SCALE_ALTITUDE_AT_ALTHOLD               1 
#define USE_ABS_POS_WHEN_ENTERING_SURFACE_HOLD  0
#define DISABLE_GPS_AT_ALTHOLD                  2
#define PITCH_AT_ALTHOLD                        0
#define INAV_BODY2EARTH_FRAME                   0 
#elif FLYZ_VER(2,6)
#define SWITCH_OPFLOW_EVERY_10SEC               0 
#define MUX_FOR_OPFLOW_SWITCH                   0 
#define SCALE_ALTITUDE_AT_ALTHOLD               1 
#define USE_ABS_POS_WHEN_ENTERING_SURFACE_HOLD  0   
#define DISABLE_GPS_AT_ALTHOLD                  1
#define PITCH_AT_ALTHOLD                        0
#define INAV_BODY2EARTH_FRAME                   0 
#elif FLYZ_VER(2,5)
#define SWITCH_OPFLOW_EVERY_10SEC               0 
#define MUX_FOR_OPFLOW_SWITCH                   0 
#define SCALE_ALTITUDE_AT_ALTHOLD               1 
#define USE_ABS_POS_WHEN_ENTERING_SURFACE_HOLD  1   
#define DISABLE_GPS_AT_ALTHOLD                  2
#define PITCH_AT_ALTHOLD                        0
#define INAV_BODY2EARTH_FRAME                   0 
#elif FLYZ_VER(2,4)
#define SWITCH_OPFLOW_EVERY_10SEC               0 
#define MUX_FOR_OPFLOW_SWITCH                   0 
#define SCALE_ALTITUDE_AT_ALTHOLD               1 
#define USE_ABS_POS_WHEN_ENTERING_SURFACE_HOLD  1   
#define DISABLE_GPS_AT_ALTHOLD                  1
#define PITCH_AT_ALTHOLD                        0
#define INAV_BODY2EARTH_FRAME                   0 
#else
#define SWITCH_OPFLOW_EVERY_10SEC               0 // [1] every 10 seconds switch between down/forwards facing opflow 
#define MUX_FOR_OPFLOW_SWITCH                   0 // [1] use swtich 49 to enable forward facing opflow (default down facing, must set SWITCH_OPFLOW_EVERY_10SEC to 0)
#define SCALE_ALTITUDE_AT_ALTHOLD               1 // [1] during surface navigation - adjust the throttle span when entering alt hold to achieve smooth transition 
#define USE_ABS_POS_WHEN_ENTERING_SURFACE_HOLD  1 // [1] force surface abs values on agl values  
#define DISABLE_GPS_AT_ALTHOLD                  4 // [1] when swtiching to alt hold disable GPS and continue using surface navigation 
                                                  // [2] same but do not re-init state
                                                  // [3] just disable gps with no other logic 
                                                  // [4] same as 3 but do it gradually over 10 sec  
#define PITCH_AT_ALTHOLD                        1 // [1] when mux is enabled during pos hold --> do constant pitch adjustment 
                                                  // [2] also do roll auto corrections  
#define INAV_BODY2EARTH_FRAME                   1 // [1] convert opflow to earth frame as inav does 

#endif

#include "io/serial.h"

void mtf_01_init(void);
void mtf_01_micolink_decode(serialPortIdentifier_e identifier, uint8_t data);
bool mtf_01_is_micolink(void);
bool mtf_01_is_facing_down(void);
float mtf_01_get_move_cm(int i);

#if DISABLE_GPS_AT_ALTHOLD
void flyz_gps_refresh(bool is_surface_enabled);
void flyz_set_agl(void);
bool flyz_is_gps_enable(void);
#endif
