/*
   This file implements virtual GPS 

    vGpsAtMissionStart --> called when pilot activates waypoint mission --> ovewrites the mission with 3 points. 
    vGpsUpdate --> called every 20ms 
 */

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <arm_math.h>
#include <math.h>
#include <stdarg.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/typeconversion.h"
#include "common/gps_conversion.h"
#include "common/maths.h"
#include "common/utils.h"
#include "common/string_light.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/runtime_config.h"
#include "fc/settings.h"
#include "fc/fc_core.h"
#include "fc/rc_modes.h"
#include "fc/rc_controls.h"

#include "blackbox/blackbox.h"

#include "io/serial.h"
#include "io/gps.h"
#include "io/gps_private.h"
#include "io/gps_ublox.h"
#include "io/gps_ublox_utils.h"

#include "scheduler/protothreads.h"

#include "vgps.h"

#include "flight/imu.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"
#include "navigation/navigation_pos_estimator_private.h"

#include "config/feature.h"

#include "sensors/sensors.h"
#include "sensors/barometer.h"


#define VERIFY_DISTANCE                     0 // recalculate distance from calculated coordinates to ensure correctness 
#define VGPS_GROUND_SPEED_CM_SEC            30 // CM/SEC 
#define VGPS_RUNTIME_BEARING                0 // [1] last known bearing for vgps mission [0] use north (0)

#define EARTH_RADIUS                        (double)6371000.0 // Earth's radius in meters
#define RAD_TO_100NANODEG(_X)               (((double)(_X)*1800000000.0l)/(double)PI)
#define _100NANODEG_TO_RAD(_X)              (((double)(_X)*(double)PI)/(double)1800000000.0)
#define DEG_TO_RAD(_X)                      (((double)(_X)*(double)PI)/180.0l)

#define VGPS_CONFIG_VAL                     10000 // vgps secret p1 value 
#define VGPS_CONFIG_DISTANCE_CM(_CM)        (VGPS_CONFIG_VAL+(_CM)) // set distance to wall in cm 
#define VGPS_CONFIG_DISTANCE_CM_GET(_VAL)   ((_VAL)-VGPS_CONFIG_VAL) // get distance to wall in cm 

// vgps states 
typedef enum {
    VGPS_STATE_DISABLE, // inactive - can do whatever, even fly a non vgps mission 
    VGPS_STATE_AT_START, // when entering the 1st vgps wp 
    VGPS_STATE_FLY_TO_WALL, // now flying to wall 
    VGPS_STATE_AT_WALL, // now at wall 
    VGPS_STATE_FLY_FROM_WALL, // now flying back from wall 
    VGPS_STATE_AT_END // now at last vgps wp and clearing out soon 
} vGpsStateType;

// vgps wp 
typedef enum {
    VGPS_WP_START, // wp 0 relative to vGpsWpIndexStart index is the start point   
    VGPS_WP_WALL, // wp 1 relative to vGpsWpIndexStart index is the wall
    VGPS_WP_END, // wp 2 relative to vGpsWpIndexStart index is the end (which is at the same location as the start usually)
    VGPS_WP_REAL_END, // wp 3 correct the error to the actual gps 
    VGPS_WP_NUM
}   VGPS_WP_TYPE;

// types of vgps missions 
typedef enum {
    VGPS_MISSION_NONE, // no vgps mission found
    VGPS_MISSION_AT_START, // mission loaded and the first wp in the mission is the start point for vgps 
    VGPS_MISSION_AT_MID // mission loaded and one of the other missions is a start point for vgps 
} VGPS_MISSION_TYPE;

// externals used in here 
extern bool _new_position;
extern bool _new_speed;
extern uint8_t next_fix_type;
extern uint8_t _ack_state;
extern uint8_t _msg_id;
extern ubx_buffer _buffer;

// global variable 
gpsSolutionData_t vGpsSol;

// static variable 
static vGpsStateType        vGpsState = VGPS_STATE_DISABLE; // state 
static gpsLocation_t        vGpsPosStart; // point T - start position on virtual navigation 
static gpsLocation_t        vGpsPosDest; // destination "wall" position 
static gpsLocation_t        vGpsPosCurrent; // current real time position 
static gpsLocation_t        vGpsPosDebug; // debug for the sensor display 
static double               vGpsDistanceTarget; // target horizontal travel distance in meters 
static double               vGpsDistance; // actual horizontal travel distance in meters 
static double               vGpsGroudSpeed; // flight speed in cm/sec 
static double               vGpsBearing; // deci degrees compass value 
static timeMs_t             vGpsTimeMcuStart; // reference time 
static timeMs_t             vGpsTimeMcuLastGps;
static uint64_t             vGpsMilliSince2020Start;
static time_t               vGpsTimeTmStart; 
static int8_t               vGpsWpIndexStart;
static uint8_t              vGpsNextFixType;
static uint8_t              vGpsMaxSat;
static VGPS_MISSION_TYPE    vGpsMissionType;

// static functions proto 
static void vGpsReset(void);
static void vGpsCalcPos(double distance, double bearing, bool is_current);
static void vGpsCalcBaseTime(void);
static bool vGpsAtStartPos(double distance);
static void vGpsAtStartFly2Dest(void);
static void vGpsAtStartFlyBack(void);
static void vGpsUpdateWaypoints(int wpIndex);
static void vGpsUpdatePositionDuringFlight(void);
static void vGpsUpdateTime(void);
static void vGpsUpdateData(void);
static void vGpsSetState(vGpsStateType state);
static void vGpsSetPosCurrent(uint32_t lon, uint32_t lat, double distnace);
static void vGpsSetPosDest(uint32_t lon, uint32_t lat, double distnace);
static bool vGpsIsVgpsWp(navWaypoint_t *wp);

// reset the entire vgps db 
static void vGpsReset(void)
{
    if ( vGpsIsActive() ) {
        next_fix_type = vGpsNextFixType;
        memcpy(&gpsSol,&vGpsSol,sizeof(gpsSol));
    }
    memset(&vGpsPosStart,0,sizeof(gpsLocation_t));
    memset(&vGpsPosDest,0,sizeof(gpsLocation_t));
    memset(&vGpsPosCurrent,0,sizeof(gpsLocation_t));
    memset(&vGpsPosDebug,0,sizeof(gpsLocation_t));
    vGpsMaxSat = 0;
    vGpsWpIndexStart = 0;
    vGpsSetState(VGPS_STATE_DISABLE); 
    vGpsDistanceTarget = 0.0; 
    vGpsDistance = 0.0;
    vGpsGroudSpeed = 0.0;
    vGpsBearing = 0.0; 
    vGpsTimeMcuStart = millis();
    vGpsTimeMcuLastGps = 0;
    vGpsMilliSince2020Start = 0;
    vGpsTimeTmStart = 0; 
#if  BLACKBOX_USING_DEBUG
    debugMode = DEBUG_VGPS;
#endif    
}

#if FAKE_GPS
// fake constant gps pos with some noise 
static bool FAKE_GPS_active = false;
void vGpsFake(void)
{
    // if we have solid gps time 
    if ( FAKE_GPS_active ) {
        // force position with some noise 
        int32_t t = (int32_t)(micros() & 0xF) - 0x7;
        vGpsPosCurrent.lon = 349983290 + t;
        vGpsPosCurrent.lat = 323723510 + t;

        // update all relevant gps fields 
        vGpsUpdateData();
    }
}
#endif

// init 
void vGpsInit(void)
{
    vGpsMissionType = VGPS_MISSION_NONE;
    vGpsReset();
}

// state getter 
int32_t vGpsGetState(void)
{
    return (int32_t)vGpsState;
}

// returns true if there is loaded mission and this mission contains vgps sub mission 
bool vGpsIsConfig(void)
{
    return vGpsMissionType != VGPS_MISSION_NONE;
}

// return true is vGps is currently in a mission using virtual gps 
bool vGpsIsActive(void)
{
    if ( vGpsState > VGPS_STATE_DISABLE ) {
        return true;
    }
    else {
        return false;
    }
}

// called whenever the real GPS receives time 
void vGpsAtGpsTimeUpdate(void)
{
    vGpsTimeMcuLastGps = millis();
#if FAKE_GPS
    // just get one good time for the GPS module to get going 
    if ( !FAKE_GPS_active ) {
        vGpsCalcBaseTime();
        FAKE_GPS_active = true;
    }
#endif    
}

// clear out vgps 
void vGpsAtMissionEnd(void)
{
    vGpsReset();
}

// when loading mission from eprom 
void vGpsAtWpLoad(void)
{
    // look for vgps wp 
    vGpsMissionType = VGPS_MISSION_NONE;
    for ( int i=0; i<posControl.waypointCount; i++ ) {
        if ( vGpsIsVgpsWp(&posControl.waypointList[i]) ) {
            // if we found a vgps wp and we have at least 2 more to go - vgps can be used in this mission  
            vGpsMissionType = i==0 ? VGPS_MISSION_AT_START : VGPS_MISSION_AT_MID;
            return;
        }
    }
}

// return true if given waypoint is vgps trigerring waypoint 
static bool vGpsIsVgpsWp(navWaypoint_t *wp)
{
    if ( wp->action==NAV_WP_ACTION_HOLD_TIME && 
        wp->p1>=VGPS_CONFIG_DISTANCE_CM(100) && // wall can be from 1m 
        wp->p1<=VGPS_CONFIG_DISTANCE_CM(2000) ) { // up to 20m 
        return true;
    }
    return false;
}

// called whenever there are more and more satellites in a purpose to update the current location in case we're in VGPS_MISSION_AT_START mode 
void vGpsAtMoreSat(uint8_t numSat)
{
    if ( vGpsIsConfig() && numSat >= 8 && numSat >= vGpsMaxSat ) {
        vGpsMaxSat = numSat;
        if ( vGpsState==VGPS_STATE_DISABLE && vGpsMissionType==VGPS_MISSION_AT_START ) {
            navWaypoint_t *wp = &posControl.waypointList[vGpsWpIndexStart];
            for ( int i=0; i<VGPS_WP_NUM; i++, wp++ ) {
                wp->lat = gpsSol.llh.lat;
                wp->lon = gpsSol.llh.lon;
                wp->alt = gpsSol.llh.alt;
            }
        }
    }
}

// this is heading alpha filter code 
#define VGPS_ALPHA 0.9 // filter 
static float vGpsHeadingFiltered = 100000000.0; // big value 
void vGpsHeading(int32_t heading_2d)
{
    int32_t currentHeading = (int32_t)vGpsHeadingFiltered;

    // init to first value 
    if ( currentHeading >= 10000000.0 ) {
        vGpsHeadingFiltered = (float)heading_2d;
        return;
    }

    // if filter is below 180 --> deduct 360 from values in the range of 271-360
    else if ( currentHeading < 1800000 ) {
        if ( heading_2d > 2700000 ) {
            heading_2d -= 3600000;
        }
    }

    // if filter is above 180 --> add 360 to values in the range of 0-89
    else if ( heading_2d < 900000 ) {
        heading_2d += 3600000;
    }

    // 
    int32_t gap = heading_2d > currentHeading ? heading_2d - currentHeading : currentHeading - heading_2d;

    if ( gap >= 900000 ) {
        vGpsHeadingFiltered = (float)heading_2d;
    }
    else {
        vGpsHeadingFiltered = VGPS_ALPHA * vGpsHeadingFiltered + (1.0-VGPS_ALPHA) * (float)heading_2d;
    }
}

// filter gps heading 
uint16_t vGpsHeadingGet(void)
{
    int32_t heading = ((vGpsHeadingFiltered / 10000.0) + 0.5);
    if ( heading >= 360 ) {
        heading -= 360;
    }
    else if ( heading < 0 ) {
        heading += 360;
    }

    return (uint16_t)heading;
}

// return true if given waypoint is vgps trigerring waypoint + update the distance in meters if true 
static bool vGpsIsActivateVgps(int wpIndex, double *distance)
{
    navWaypoint_t *wp = &posControl.waypointList[wpIndex];
    if ( vGpsIsVgpsWp(wp) ) {
        *distance = ((double)VGPS_CONFIG_DISTANCE_CM_GET(wp->p1))/100.0l;
        return true;
    }

    // no vgps trigger point 
    *distance = 0l;
    return false;
}

// called to activate vgps 
static void vGpsActivationHandler(uint8_t index)
{
    // this is another method of entering VGPS - during a mission 
    if ( vGpsState==VGPS_STATE_DISABLE ) {
        double distance_m;
        if ( vGpsIsActivateVgps(index,&distance_m) ) {
            if ( vGpsAtStartPos(distance_m) ) {
                vGpsUpdateWaypoints(index);
            }
        }
    }
}

// called when RC actually switch to a mission 
void vGpsAtMissionStart(void)
{
    // this is one method of entering VGPS - at mission start when the first waypooint is position hold for 10100-12000 seconds (1000 is a marker)
    vGpsActivationHandler(0);
}

// called when reaching waypoint 
void vGpsAtWaypointReached(uint8_t index)
{
    // this is another method of entering VGPS - during a mission 
    if ( vGpsState==VGPS_STATE_DISABLE ) {
        vGpsActivationHandler(posControl.activeWaypointIndex);
        return;
    }

    if ( index < vGpsWpIndexStart ) {
        return;
    }
    index -= vGpsWpIndexStart;
    switch ( index ) {
        case VGPS_WP_WALL:
            vGpsUpdatePositionDuringFlight(); // last pos update before stop
            vGpsGroudSpeed = 0.0l; // we are now not flying 
            vGpsSetState(VGPS_STATE_AT_WALL);
            break;
        case VGPS_WP_END:
            vGpsUpdatePositionDuringFlight(); // last pos update before stop
            vGpsGroudSpeed = 0.0l; // we are now not flying 
            vGpsSetState(VGPS_STATE_AT_END);
            break;
    }
}

// called when leaving waypoint 
void vGpsAtWaypointNext(uint8_t index)
{
    // this is another method of entering VGPS - during a mission 
    if ( vGpsState==VGPS_STATE_DISABLE ) {
        return;
    }

    if ( index < vGpsWpIndexStart ) {
        return;
    }
    index -= vGpsWpIndexStart;
    switch ( index ) {
        case VGPS_WP_START:
            vGpsAtStartFly2Dest();
            break;
        case VGPS_WP_WALL:
            vGpsAtStartFlyBack();
            break;
        case VGPS_WP_END:
            vGpsAtMissionEnd();
            break;
    }
}

// called at 50hz before the rest of the gps module
void vGpsUpdate(void)
{
    // if active 
    if ( vGpsState > VGPS_STATE_DISABLE ) {
        
        // update postion if moving 
        vGpsUpdatePositionDuringFlight();

        // simulate new packet arrived 
        vGpsUpdateData();
    }

#if  BLACKBOX_USING_DEBUG
    // as soon as we have good position - use it as ref for the debug fields 
    if ( gpsSol.numSat >= 8 && vGpsPosDebug.lon==0 ) {
        vGpsPosDebug = gpsSol.llh;
    }
    DEBUG_SET(DEBUG_VGPS, DEBUG_VGPS_SAT, gpsSol.numSat);
    DEBUG_SET(DEBUG_VGPS, DEBUG_VGPS_SPEED, gpsSol.groundSpeed);
    DEBUG_SET(DEBUG_VGPS, DEBUG_VGPS_COURSE, gpsSol.groundCourse );
    if ( vGpsPosDebug.lon>0 ) {
        DEBUG_SET(DEBUG_VGPS, DEBUG_VGPS_DLON, gpsSol.llh.lon - vGpsPosDebug.lon ); 
        DEBUG_SET(DEBUG_VGPS, DEBUG_VGPS_DLAT, gpsSol.llh.lat - vGpsPosDebug.lat );
    }
#endif
}

#if WORK_WITHOUT_RC_FROM_CLI
static bool vGpsIsForceArm = false;
static bool vGpsIsPosHold = false;
static bool vGpsIsPlanWpMission = false;
static bool vGpsIsBlackBox = false;

// fake aux by overriding booleans 
bool vGpsOverideRcSwitches(boxId_e boxId) 
{
    switch ( boxId ) {
        case BOXARM:
        case BOXPREARM:
        case BOXBLACKBOX:
        case BOXNAVPOSHOLD:
        case BOXNAVALTHOLD:
        case BOXHEADINGHOLD: 
        case BOXSURFACE:
            return (vGpsIsForceArm || vGpsIsPosHold);
        case BOXNAVWP:
        //case BOXNAVLAUNCH:
            return vGpsIsPlanWpMission;
        default:
            return false;    
    }
} 
#endif

// parse cli command "vgps"
void vGpsCli(char *cmdline)
{
    int len = strlen(cmdline);
    if ( len > 0) {
#if WORK_WITHOUT_RC_FROM_CLI
        if ( sl_strncasecmp(cmdline, "blackbox", len) == 0) {
            if ( !vGpsIsBlackBox ) {
                vGpsIsBlackBox = true;
                blackboxStart();
            }
            else {
                vGpsIsBlackBox = false;
                blackboxFinish();
            }
        }
        else if ( sl_strncasecmp(cmdline, "arm", len) == 0) {
            vGpsIsForceArm = vGpsIsForceArm ? false : true;
            vGpsIsPosHold = false;
            vGpsIsPlanWpMission = false;
            processRcStickPositions(true);
            processAirmode();
            updateActivatedModes();
        }
        else if ( sl_strncasecmp(cmdline, "mission", len) == 0) {
            if ( vGpsIsForceArm ) {
                vGpsIsPosHold = false;
                vGpsIsPlanWpMission = true;
                updateWaypointsAndNavigationMode();
            }
        }
        else if ( sl_strncasecmp(cmdline, "poshold", len) == 0) {
            if ( vGpsIsForceArm ) {
                vGpsIsPosHold = true;
                vGpsIsPlanWpMission = false;
                updateWaypointsAndNavigationMode();
            }
        }
        else
#endif        
        {
            cliPrintLinef("vgps          --> print status");
#if WORK_WITHOUT_RC_FROM_CLI
            cliPrintLinef("vgps arm      --> arm / disarm");
            cliPrintLinef("vgps blackbox --> start/stop blackbox");
            cliPrintLinef("vgps mission  --> start a wp mission");
            cliPrintLinef("vgps poshold  --> enter position hold");
#endif            
        }
    }
    
    uint32_t dt = millis() - vGpsTimeMcuStart;
    static const char state_str[][12] = { 
        "DISABLE",
        "AT_START",
        "FLY_TO_WALL",
        "AT_WALL",
        "FROM_WALL",
        "AT_END"
    };
    cliPrintLinef("%d.%d %s %d/%d %d/%d/%d/%d %d/%d/%d/%d %d/%d-->%d/%d-->%d/%d distance[cm]=%d/%d speed[cm/sec]=%d",
        dt/1000, dt%1000,
        state_str[vGpsState],
        gpsSol.groundCourse, vGpsHeadingGet(), 
        gpsSol.numSat,  gpsSol.llh.lon,  gpsSol.llh.lat,  gpsSol.llh.alt,
        vGpsSol.numSat, vGpsSol.llh.lon, vGpsSol.llh.lat, vGpsSol.llh.alt, 
        vGpsPosStart.lon, vGpsPosStart.lat,
        vGpsPosCurrent.lon, vGpsPosCurrent.lat, 
        vGpsPosDest.lon, vGpsPosDest.lat,
        (int)(vGpsDistance*100.0l), (int)(vGpsDistanceTarget*100.0l),
        (int)vGpsGroudSpeed );

    void vGpsNavPrint(void);
    vGpsNavPrint();

}

// state setter 
static void vGpsSetState(vGpsStateType state)
{
    if ( vGpsState==state ) {
        return;
    }

    vGpsState = state;
#if BLACKBOX_USING_DEBUG
    DEBUG_SET(DEBUG_VGPS, DEBUG_VGPS_STATE, vGpsState);
#endif
}    

// force current (given) pos and distance 
static void vGpsSetPosCurrent(uint32_t lon, uint32_t lat, double distnace)
{
    vGpsPosCurrent.lon = lon; 
    vGpsPosCurrent.lat = lat;
    vGpsDistance = distnace;
#if BLACKBOX_USING_DEBUG
    DEBUG_SET(DEBUG_VGPS, DEBUG_VGPS_DIST, (int32_t)((vGpsDistanceTarget-distnace)*100.0l)); // cm
#endif    
}

// sets destination (wall or home) 
static void vGpsSetPosDest(uint32_t lon, uint32_t lat, double distnace)
{
    vGpsPosDest.lon = lon; 
    vGpsPosDest.lat = lat;
    vGpsDistanceTarget = distnace;
#if BLACKBOX_USING_DEBUG
    DEBUG_SET(DEBUG_VGPS, DEBUG_VGPS_DIST, (int32_t)(distnace*100.0l)); // cm
#endif    
}

// set postion to "distance" meters from current postion in given bearing (degrees, 0=north, 90=east)
static void vGpsCalcPos(double distance, double bearing, bool is_current ) 
{
    double angularDistance = distance / EARTH_RADIUS;
    double bearingRad = DEG_TO_RAD(bearing);
    double latRad = _100NANODEG_TO_RAD(vGpsPosStart.lat); // Convert microdegrees to degrees to radians
    double lonRad = _100NANODEG_TO_RAD(vGpsPosStart.lon); // Convert microdegrees to degrees to radians
    double newLatRad = asin(sin(latRad) * cos(angularDistance) + cos(latRad) * sin(angularDistance) * cos(bearingRad));
    double newLonRad = lonRad + atan2(sin(bearingRad) * sin(angularDistance) * cos(latRad), cos(angularDistance) - sin(latRad) * sin(newLatRad));
    uint32_t lon = (int32_t)(RAD_TO_100NANODEG(newLonRad));
    uint32_t lat = (int32_t)(RAD_TO_100NANODEG(newLatRad));

    if ( is_current ) {
        vGpsSetPosCurrent(lon,lat,distance);
    }
    else {
        vGpsSetPosDest(lon,lat,distance);
    }   

#if VERIFY_DISTANCE
    // calculate distance in meters     
    double dlon = _100NANODEG_TO_RAD(pos->lon - vGpsPosStart.lon); // delta lon in radians
    double dlat = _100NANODEG_TO_RAD(pos->lat - vGpsPosStart.lat); // delta lat in radians
    double lat_start = _100NANODEG_TO_RAD(vGpsPosStart.lat); // start lat in radians
    double lat_dest = _100NANODEG_TO_RAD(pos->lat); // dest lat in radians
    double sin_dlat = sin(dlat / 2);
    double sin_dlon = sin(dlon / 2);
    double a = sin_dlat*sin_dlat + cos(lat_start) * cos(lat_dest) * sin_dlon * sin_dlon;
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
#endif    
}   

// called when reaching  to save where we start from 
// set distance in meters from current postion based on the last ground course (heading)
static bool vGpsAtStartPos(double distance)
{
    // already active - ignore 
    if ( vGpsState > VGPS_STATE_DISABLE ) {
        return true;
    }

    // in real mode do some checking to ensure data its a good startign point 
#if 0
    // must have gps enabled 
    if ( !feature(FEATURE_GPS) || !sensors(SENSOR_GPS) ) {
        return false;
    }

    // in case GPS is not "perfect" - do not enter virtual GPS mode 
    if ( gpsSol.eph > 200 || gpsSol.epv > 200 || !gpsSol.flags.validEPE || 
        gpsSol.fixType != GPS_FIX_3D ||
        gpsSol.hdop > 200 ||
        gpsSol.numSat < 8 ) {
        return false;
    }
#endif        

    // new state - assume we're holding position now 
    vGpsSetState(VGPS_STATE_AT_START);
    vGpsNextFixType = next_fix_type;
    memcpy(&vGpsSol,&gpsSol,sizeof(gpsSol));

    // use last known gps time as ref for fake time later 
    vGpsCalcBaseTime();

    // capture horizontal position for later use 
    vGpsPosStart.lon = gpsSol.llh.lon; 
    vGpsPosStart.lat = gpsSol.llh.lat; 
    
#if VGPS_RUNTIME_BEARING    
    // set bearing to current bearing in degrees 
    vGpsBearing = ((double)gpsSol.groundCourse)/10.0l;
#else
    vGpsBearing = 0; // north 
#endif

    // set destination position based on given distance and bearing 
    vGpsCalcPos(distance, vGpsBearing, false);

    // reset all the rest of the data 
    vGpsSetPosCurrent(vGpsPosStart.lon,vGpsPosStart.lat,0.0l);
    vGpsGroudSpeed = 0.0l; // we are now not flying 

    return true;
}

// called once we start moving from start to end at a constant given ground speed 
static void vGpsAtStartFly2Dest(void)
{
    // save speed 
    vGpsGroudSpeed = VGPS_GROUND_SPEED_CM_SEC;

    // take ref time 
    vGpsTimeMcuStart = millis();

    // we're flying at groundSpeed to vGpsPosDest at a straight horizontal flight line 
    vGpsSetState(VGPS_STATE_FLY_TO_WALL);
}

// called when starting to fly back from wall 
static void vGpsAtStartFlyBack(void)
{
    // flip start and dest and force current to new start 
    gpsLocation_t tmp = vGpsPosStart; 
    vGpsPosStart = vGpsPosDest;
    vGpsSetPosCurrent(vGpsPosStart.lon, vGpsPosStart.lat,0.0l);
    vGpsSetPosDest(tmp.lon, tmp.lat, vGpsDistanceTarget);
    
    // set bearing to current bearing in degrees 
    vGpsBearing += 180.0l; 
    if ( vGpsBearing >= 360.0l ) {
        vGpsBearing -= 360.0l; 
    }

    // save speed 
    vGpsGroudSpeed = VGPS_GROUND_SPEED_CM_SEC;

    // take ref time 
    vGpsTimeMcuStart = millis();

    // update state 
    vGpsSetState(VGPS_STATE_FLY_FROM_WALL);
}

// called when finding the wp index where vgps starts. it forces vgps values to therelevant vgps mission portion 
static void vGpsUpdateWaypoints(int wpIndex)
{
    // save ref index of waypooint 
    vGpsWpIndexStart = wpIndex;

    //extern navigationPosEstimator_t posEstimator;
    int32_t alt = (int32_t)posEstimator.est.pos.z; // gpsSol.llh.alt

    navWaypoint_t *wp = &posControl.waypointList[wpIndex];
    wp->p1 = 1; // hold time in sec for action=NAV_WP_ACTION_HOLD_TIME
    if ( wp->action != NAV_WP_ACTION_HOLD_TIME ) {
        wp->action = NAV_WP_ACTION_HOLD_TIME;
        wp->p2 = VGPS_GROUND_SPEED_CM_SEC; // default ground speed 
        wp->flag = 0;
    }
    wp->lat = vGpsPosStart.lat;
    wp->lon = vGpsPosStart.lon;
    wp->alt = alt;

    wp++;
    if ( wp->action != NAV_WP_ACTION_HOLD_TIME ) {
        wp->action = NAV_WP_ACTION_HOLD_TIME;
        wp->p1 = 1; // hold time in sec for action=NAV_WP_ACTION_HOLD_TIME
        wp->p2 = VGPS_GROUND_SPEED_CM_SEC; // default ground speed 
    }
    wp->lat = vGpsPosDest.lat;
    wp->lon = vGpsPosDest.lon;
    wp->alt = alt;

    // last two wp is at start and later the next one is adjusted to the actual gps reading 
    for ( int i=0; i<2; i++ ) {
        wp++;
        if ( wp->action != NAV_WP_ACTION_HOLD_TIME ) {
            wp->action = NAV_WP_ACTION_HOLD_TIME;
            wp->p1 = 1; // hold time in sec for action=NAV_WP_ACTION_HOLD_TIME
            wp->p2 = VGPS_GROUND_SPEED_CM_SEC; // default ground speed 
        }
        wp->lat = vGpsPosStart.lat;
        wp->lon = vGpsPosStart.lon;
        wp->alt = alt;
    }
}

// called at any desired refresh rate (say 100hz) while flying from vGpsPosStart to vGpsPosDest as speed of vGpsGroudSpeed cm/sec.
// It calculates and updates our expected current position 
static void vGpsUpdatePositionDuringFlight(void) 
{
    if ( vGpsGroudSpeed>0 ) {
        // calcualte elapsed time 
        int32_t elapsedTimeMs = (int32_t)(millis() - vGpsTimeMcuStart);

        // distance covered in given time interal (cm) 
        double distanceCm = vGpsGroudSpeed * (double)elapsedTimeMs / 1000.0l;

        vGpsCalcPos(distanceCm/100.0l, vGpsBearing, true);

    }
}

// time calculations when leaving real gps into virtual gps 
static void vGpsCalcBaseTime(void)
{
    // take current (last known) real gps time 
    struct tm tm_now = {0};
    tm_now.tm_year = gpsSol.time.year - 1900; // Years since 1900
    tm_now.tm_mon = gpsSol.time.month - 1;    // Months since January (0-11)
    tm_now.tm_mday = gpsSol.time.day;         // Day of the month (1-31)
    tm_now.tm_hour = gpsSol.time.hours;        // Hours since midnight (0-23)
    tm_now.tm_min = gpsSol.time.minutes;       // Minutes after the hour (0-59)
    tm_now.tm_sec = gpsSol.time.seconds;       // Seconds after the minute (0-59)
    uint64_t now = (uint64_t)1000 * (uint64_t)mktime(&tm_now) + (uint64_t)gpsSol.time.millis;

    // take ref of 1/1/2020 00:00 
    struct tm tm_base = {0};
    tm_base.tm_year = 2020-1900; 
    tm_base.tm_mon = 0;
    tm_base.tm_mday = 1;
    tm_base.tm_hour = 0;
    tm_base.tm_min = 0; 
    tm_base.tm_sec = 0; 
    vGpsTimeTmStart = mktime(&tm_base);
    uint64_t base = (uint64_t)1000 * (uint64_t)vGpsTimeTmStart;

    // record the delta in milliseconds from 1/1/2020 00:00 to the last known real gps time 
    vGpsMilliSince2020Start = now - base;
}

// time update 
static void vGpsUpdateTime(void)
{
    uint64_t milli_since_2020 = vGpsMilliSince2020Start + ((uint64_t)millis() - vGpsTimeMcuLastGps);
    time_t seconds_since_1900 = (uint32_t)(milli_since_2020 / 1000) + vGpsTimeTmStart; 
    struct tm *timeinfo;
    timeinfo = gmtime(&seconds_since_1900);
    gpsSol.time.year = timeinfo->tm_year + 1900;
    gpsSol.time.month = timeinfo->tm_mon + 1;
    gpsSol.time.day = timeinfo->tm_mday;
    gpsSol.time.hours = timeinfo->tm_hour;
    gpsSol.time.minutes = timeinfo->tm_min;
    gpsSol.time.seconds = timeinfo->tm_sec;
    gpsSol.time.millis = milli_since_2020 % 1000;
    gpsSol.flags.validTime = true;
}

// refresh real gps data per vgps state 
static void vGpsUpdateData(void)
{
    // force position 
    gpsSol.llh.lon = vGpsPosCurrent.lon;
    gpsSol.llh.lat = vGpsPosCurrent.lat;

    // force speed/direction 
    gpsSol.groundSpeed = (int16_t)(vGpsGroudSpeed+0.5l); // cm/s 
    gpsSol.groundCourse = (int16_t)(vGpsBearing*10.0l+0.5l); // degrees to decidegrees 
    gpsSol.flags.validVelNE = true;
    gpsSol.flags.validVelD = true;
    gpsSol.flags.validEPE = true;

    // fake global variables related to type of data received 
    _new_position = true;
    _new_speed = true;
    next_fix_type = GPS_FIX_3D;
    _ack_state = UBX_ACK_GOT_ACK;

    vGpsUpdateTime();

    // force best quality signals 
    gpsSol.eph = 100; // 1
    gpsSol.epv = 100; // 1 
    gpsSol.flags.validEPE = true;
    gpsSol.fixType = GPS_FIX_3D; // simulate good fix 
    gpsSol.hdop = 100; // 100/100 --> Horizontal Dilution of Precision - the accuracy of the GPS horizontal position. A lower HDOP value indicates better accuracy and reliability of GPS data
    gpsSol.numSat = 20;
    gpsSol.flags.validTime = true;
}

// mirror of function gpsMapFixType
static uint8_t vGpsMapFixType(bool fixValid, uint8_t ubloxFixType)
{
    if (fixValid && ubloxFixType == FIX_2D)
        return GPS_FIX_2D;
    if (fixValid && ubloxFixType == FIX_3D)
        return GPS_FIX_3D;
    return GPS_NO_FIX;
}

// mirror of funciton gpsParseFrameUBLOX
bool vGpsParseFrame(void)
{
    static bool v_new_position = false;
    static bool v_new_speed = false;

    // if VGPS is active - parse to vGpsSol mirror 
    switch (_msg_id) {
    case MSG_POSLLH:
        vGpsSol.llh.lon = _buffer.posllh.longitude;
        vGpsSol.llh.lat = _buffer.posllh.latitude;
        vGpsSol.llh.alt = _buffer.posllh.altitude_msl / 10;  //alt in cm
        vGpsSol.eph = gpsConstrainEPE(_buffer.posllh.horizontal_accuracy / 10);
        vGpsSol.epv = gpsConstrainEPE(_buffer.posllh.vertical_accuracy / 10);
        vGpsSol.flags.validEPE = true;
        if (vGpsNextFixType != GPS_NO_FIX)
            vGpsSol.fixType = vGpsNextFixType;
        v_new_position = true;
        break;
    case MSG_STATUS:
        vGpsNextFixType = vGpsMapFixType(_buffer.status.fix_status & NAV_STATUS_FIX_VALID, _buffer.status.fix_type);
        if (vGpsNextFixType == GPS_NO_FIX)
            vGpsSol.fixType = GPS_NO_FIX;
        break;
    case MSG_SOL:
        vGpsNextFixType = vGpsMapFixType(_buffer.solution.fix_status & NAV_STATUS_FIX_VALID, _buffer.solution.fix_type);
        if (vGpsNextFixType == GPS_NO_FIX)
            vGpsSol.fixType = GPS_NO_FIX;
        vGpsSol.numSat = _buffer.solution.satellites;
        vGpsSol.hdop = gpsConstrainHDOP(_buffer.solution.position_DOP);
        break;
    case MSG_VELNED:
        vGpsSol.groundSpeed = _buffer.velned.speed_2d;    // cm/s
        vGpsSol.groundCourse = (uint16_t) (_buffer.velned.heading_2d / 10000);     // Heading 2D deg * 100000 rescaled to deg * 10
        vGpsSol.velNED[X] = _buffer.velned.ned_north;
        vGpsSol.velNED[Y] = _buffer.velned.ned_east;
        vGpsSol.velNED[Z] = _buffer.velned.ned_down;
        vGpsSol.flags.validVelNE = true;
        vGpsSol.flags.validVelD = true;
        v_new_speed = true;
        break;
    case MSG_TIMEUTC:
        if (UBX_VALID_GPS_DATE_TIME(_buffer.timeutc.valid)) {
            vGpsSol.time.year = _buffer.timeutc.year;
            vGpsSol.time.month = _buffer.timeutc.month;
            vGpsSol.time.day = _buffer.timeutc.day;
            vGpsSol.time.hours = _buffer.timeutc.hour;
            vGpsSol.time.minutes = _buffer.timeutc.min;
            vGpsSol.time.seconds = _buffer.timeutc.sec;
            vGpsSol.time.millis = _buffer.timeutc.nano / (1000*1000);
            vGpsSol.flags.validTime = true;
        } else {
            vGpsSol.flags.validTime = false;
        }
        break;
    case MSG_PVT:
        vGpsNextFixType = vGpsMapFixType(_buffer.pvt.fix_status & NAV_STATUS_FIX_VALID, _buffer.pvt.fix_type);
        vGpsSol.fixType = vGpsNextFixType;
        vGpsSol.llh.lon = _buffer.pvt.longitude;
        vGpsSol.llh.lat = _buffer.pvt.latitude;
        vGpsSol.llh.alt = _buffer.pvt.altitude_msl / 10;  //alt in cm
        vGpsSol.velNED[X]=_buffer.pvt.ned_north / 10;  // to cm/s
        vGpsSol.velNED[Y]=_buffer.pvt.ned_east / 10;   // to cm/s
        vGpsSol.velNED[Z]=_buffer.pvt.ned_down / 10;   // to cm/s
        vGpsSol.groundSpeed = _buffer.pvt.speed_2d / 10;    // to cm/s
        vGpsSol.groundCourse = (uint16_t) (_buffer.pvt.heading_2d / 10000);     // Heading 2D deg * 100000 rescaled to deg * 10
        vGpsSol.numSat = _buffer.pvt.satellites;
        vGpsSol.eph = gpsConstrainEPE(_buffer.pvt.horizontal_accuracy / 10);
        vGpsSol.epv = gpsConstrainEPE(_buffer.pvt.vertical_accuracy / 10);
        vGpsSol.hdop = gpsConstrainHDOP(_buffer.pvt.position_DOP);
        vGpsSol.flags.validVelNE = true;
        vGpsSol.flags.validVelD = true;
        vGpsSol.flags.validEPE = true;

        if (UBX_VALID_GPS_DATE_TIME(_buffer.pvt.valid)) {
            vGpsSol.time.year = _buffer.pvt.year;
            vGpsSol.time.month = _buffer.pvt.month;
            vGpsSol.time.day = _buffer.pvt.day;
            vGpsSol.time.hours = _buffer.pvt.hour;
            vGpsSol.time.minutes = _buffer.pvt.min;
            vGpsSol.time.seconds = _buffer.pvt.sec;
            vGpsSol.time.millis = _buffer.pvt.nano / (1000*1000);
            vGpsSol.flags.validTime = true;
        } else {
            vGpsSol.flags.validTime = false;
        }
        v_new_position = true;
        v_new_speed = true;
        break;
    case MSG_VER:
        break;
    case MSG_MON_GNSS:
        break;
    case MSG_ACK_ACK:
        break;
    case MSG_ACK_NACK:
        break;
    default:
        return false;
    }

    // we only return true when we get new position and speed data
    // this ensures we don't use stale data
    if (v_new_position && v_new_speed) {
        v_new_speed = v_new_position = false;
        return true;
    }

    return false;
}
