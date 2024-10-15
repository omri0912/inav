/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>

#include "platform.h"
#include "flyz.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"

#include "io/serial.h"

#include "target/common.h"

#if defined(USE_RANGEFINDER_MSP)

#include "drivers/rangefinder/rangefinder_virtual.h"
#include "drivers/time.h"
#include "io/rangefinder.h"
#include "msp/msp_protocol_v2_sensor_msg.h"


static bool hasNewData = false;
static int32_t sensorData = RANGEFINDER_NO_NEW_DATA;
#if defined(VERTICAL_OPFLOW) && !defined(USE_OPFLOW_MICOLINK)
float sensorDataF = 0; // omri-todo this one has down drift over time related to something in the opflow. check if rate is stady and if it is same on any hardware 
#endif

static bool mspRangefinderDetect(void)
{
    // Always detectable
    return true;
}

static void mspRangefinderInit(void)
{
}

static void mspRangefinderUpdate(void)
{
}

int32_t mspRangefinderGetDistanceLast(void)
{
    return sensorData;
}

static int32_t mspRangefinderGetDistance(void)
{
    if (hasNewData) {
        hasNewData = false;
        return (sensorData > 0) ? sensorData : RANGEFINDER_OUT_OF_RANGE;
    }
    else {
        return RANGEFINDER_NO_NEW_DATA;
    }
}

#if defined(VERTICAL_OPFLOW) && !defined(USE_OPFLOW_MICOLINK) // get callback by the the opflow whenever it has new data and its x value is used as z for the range finder 
void opflowSyncRangefinder(float dMm)
{
    sensorDataF += dMm; 

    // omri-todo there is neg drift resulting probably from the opflow sensor ~-1mm every 10sec 
    if ( sensorDataF <= -200.0 ) {
        sensorDataF = 0.0;
    }

    // note - here we also convert from mm to cm 
    sensorData = 20 + (int32_t)(sensorDataF/10.0+0.5); // assume 0.2m AGL as init value 1m=100mm 
//    DEBUG_SET(DEBUG_FLOW, 5, sensorData);
    hasNewData = true;
}
#endif

void mspRangefinderReceiveNewData(uint8_t * bufferPtr)
{
    const mspSensorRangefinderDataMessage_t * pkt = (const mspSensorRangefinderDataMessage_t *)bufferPtr;
#ifdef VERTICAL_OPFLOW_DEMO
    DEBUG_SET(DEBUG_FLOW, 5, (pkt->distanceMm/10));
#endif    
#if defined(VERTICAL_OPFLOW) && !defined(USE_OPFLOW_MICOLINK) // let the opflow use this new z value as x 
    void rangefinderSyncOpflow(int32_t zFromRangeFinder);
    rangefinderSyncOpflow(pkt->distanceMm);
#else
    sensorData = pkt->distanceMm / 10;
    hasNewData = true;
#endif    
}

virtualRangefinderVTable_t rangefinderMSPVtable = {
    .detect = mspRangefinderDetect,
    .init = mspRangefinderInit,
    .update = mspRangefinderUpdate,
    .read = mspRangefinderGetDistance
};

#endif
