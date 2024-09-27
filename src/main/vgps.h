/*
   This file implements virtual GPS 
 */

#include <navigation/navigation.h>
#include "fc/rc_modes.h"
#include "flyz.h"

#define FAKE_GPS 0 // set to 1 if GPS chip defected 

extern void vGpsAtGpsTimeUpdate(void);
extern void vGpsFake(void);
extern void vGpsInit(void);
extern bool vGpsIsConfig(void);
extern bool vGpsIsActive(void);
extern void vGpsAtMissionStart(void);
extern void vGpsAtMissionEnd(void);
extern void vGpsAtWaypointReached(uint8_t index);
extern void vGpsAtWaypointNext(uint8_t index);
extern void vGpsAtMoreSat(uint8_t numSat);
extern void vGpsHeading(int32_t heading_2d);
extern uint16_t vGpsHeadingGet(void);
extern void vGpsUpdate(void);
extern void vGpsCli(char *cmdline);
extern bool vGpsParseFrame(void);
extern void vGpsAtWpLoad(void);
extern int32_t vGpsGetState(void);
#if WORK_WITHOUT_RC_FROM_CLI
extern bool vGpsOverideRcSwitches(boxId_e boxId);
#endif
extern void cliPrintLinef(const char *format, ...);

#if BLACKBOX_USING_DEBUG
typedef enum {
    DEBUG_VGPS_STATE,
    DEBUG_VGPS_SAT,
    DEBUG_VGPS_DLON,
    DEBUG_VGPS_DLAT,
    DEBUG_VGPS_COURSE,
    DEBUG_VGPS_DIST,
    DEBUG_VGPS_SPEED,
    DEBUG_VGPS_EVENT
} DEBUG_VGPS_TYPE;
#endif

