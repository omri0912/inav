#pragma once

#define WORK_WITHOUT_RC_FROM_CLI                0 // work without RC from CLI 
#define BLACKBOX_USING_DEBUG                    0
#define FLYZ_OVERRIDE_BB_NAV                    1 // override few BB NAV fields 
#define AUTO_MUX                                1 // 

//#define VGPS                        // do  waypints navigation mission where the dorne flies 5m to a wall and back streaming virutal GPS data (UBLOX protocol) 

#define FLYZ_VER_MAJOR                          4
#define FLYZ_VER_MINOR                          5
#define FLYZ_VER(MAJOR,MINOR)                   ((FLYZ_VER_MAJOR)==(MAJOR) && (MINOR)==(FLYZ_VER_MINOR))

#define SWITCH_OPFLOW_EVERY_10SEC               0 // [1] every 10 seconds switch between down/forwards facing opflow 
#define MUX_FOR_OPFLOW_SWITCH                   1 // [1] use swtich 49 to enable forward facing opflow (default down facing, must set SWITCH_OPFLOW_EVERY_10SEC to 0)
#define DISABLE_GPS_AT_ALTHOLD                  4 // [1] when swtiching to alt hold disable GPS and continue using surface navigation 
                                                  // [2] same but do not re-init state
                                                  // [3] just disable gps with no other logic 
                                                  // [4] same as 3 but do it gradually over 10 sec  
#define PITCH_AT_ALTHOLD                        1 // [1] when mux is enabled during pos hold --> do constant pitch adjustment 
                                                  // [2] also do roll auto corrections  

// flyz_config_val bit values 
#define FLYZ_CONFIG_MASK_SCALE_THROTTLE         1
#define FLYZ_CONFIG_MASK_RESET_POS_HOLD_SWTICH  2
#define FLYZ_CONFIG_MASK_USE_CURR_NAV_POS       4
#define FLYZ_CONFIG_MASK_USE_INAV_EARTH_FRAME   8


/*
    Flyz CLI values:

    set flyz_pitch_force = 1500 
    when "FLYZ PITCH CTRL" MUX is turned on - the given value is foced as constant pitch PWM value 
    default 1500

    set flyz_min_num_sat = 4
    default 4, range 0-199
    set flyz_sat_decay = 0
    default 0, range 0-1000
    when "FLYZ NO GPS" MUX is turned on - the number of satellites is forced to this value.
    the rate of decay in number of satellites is set by flyz_sat_decay [seconds].
    in case flyz_sat_decay is 0, the change is immedate.

    set flyz_config = 15
    bit field mask with multiple configuration options as follows:
    bit 0 (value 1/0) - scale max altitude to throttle value when entring surface pos-hold (0 no scale)
    bit 1 (value 2/0) - while in pos-hold - reset when switching between surface and gps pos hold (0 no reset/scaling)
    bit 2 (value 4/0) - when disabling gps into surface pos-hold - use current nav postion (0 use surface pos as is)
    bit 3 (value 8/0) - convert surface body frame to inav earth frame (0 convert as we did so far x=y y=-x)
    default 15

    set flyz_opflow_port = 2
    set flyz_opflow_is_facing_wall = 0
    sets uart port and facing direction for the MTF01/OPFLOW 
    set flyz_opflow_port to 0 if you have no MTF 

    set inav_allow_dead_reckoning = ON
    set inav_max_surface_altitude = 10000
    set nav_max_terrain_follow_alt = 10000
    set debug_mode = FLOW

    Flyz MUX:

    "FLYZ OPFLW SWTCH" --> when turned on it switches to the wall-facing MTF (must set two MRF PORTS) [BOXLOITERDIRCHN]
    "FLYZ NO GPS" --> when turned on it disables GPS per flyz_config settings (see above "set flyz_config = xx") [BOXTURTLE]
    "FLYZ PITCH CTRL" --> when turned on it forces constant pitch value set by "set flyz_pitch_force = xxx" [BOXAUTOLEVEL]



*/

#include "io/serial.h"
#include "fc/rc_modes.h"

void mtf_01_init(void);
void mtf_01_micolink_decode(serialPortIdentifier_e identifier, uint8_t data);
bool mtf_01_is_micolink(void);
bool mtf_01_is_facing_down(void);
float mtf_01_get_move_cm(int i);

#if DISABLE_GPS_AT_ALTHOLD
void flyz_gps_refresh(bool is_surface_enabled);
void flyz_surface_action_when_leaving_gps_poshold(void);
bool flyz_is_gps_enable(void);
#endif

void flyz_throttle_span_calculate(bool useTerrainFollowing);
void flyz_throttle_span_init(void);

bool flyz_auto_mux_is_on(boxId_e boxId);
void flyz_auto_mux_set(boxId_e boxId, bool is_on); 
