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

#if defined(USE_OPFLOW_MSP)

#include "drivers/opflow/opflow_virtual.h"
#include "drivers/time.h"
#include "io/opflow.h"

#include "msp/msp_protocol_v2_sensor_msg.h"


static bool hasNewData = false;
static timeUs_t updatedTimeUs = 0;
#ifdef VERTICAL_OPFLOW // define this to get callback whenever the range finder is updated 
static int32_t rangeFinderLastValueMm = -1;
static timeUs_t rangeFinderLastTimeUsec = 0;
#endif
static opflowData_t sensorData = { 0 };

static bool mspOpflowDetect(void)
{
    // Always detectable
    return true;
}

static bool mspOpflowInit(void)
{
    return true;
}

static bool mspOpflowUpdate(opflowData_t * data)
{
    if (hasNewData) {
        hasNewData = false;
        *data = sensorData;
        return true;
    }

    return false;
}

#ifdef VERTICAL_OPFLOW // define this to get callback whenever the range finder is updated 

// return milimetric multiplier 
static float mspCalcFactor(int32_t usec, int32_t rangeMm)
{
    float factor = ((float)usec * // delta in microseconds 
                    (float)rangeMm * // distance of the OPFLOW from the surfface in mm (this translates pixels later)
                    (float)0.76772807007083) // tan(radians(42/2))*2 where 42 is the point of view of the sensor in degrees 
                    / (float)300000.0; // 30 pixels resolution * 10E4 to match the microseconds in 100hz and get a value in seconds 
    return factor;
}

static float pixelsPerSec2mmDelta( int32_t pixelsPerSec, int32_t usec, int32_t rangeMm)
{
    // calc mm factor assuming 42 degrees field view 
    float factor = mspCalcFactor(usec,rangeMm);

    // convert the delta in pixels to delta in cm     
    return pixelsPerSec * factor;
}

// mmDelta is the delta in the range finder during the past usec time 
// mmDelta is convertedto pixels and is actually delta in the x axis in case the point of view was top/down 
static float mm2pixelsPerSec( int32_t usec, int32_t mmDelta)
{
    // the distance the virtual range finder believes it sees (it is actually the accumulation of dx from the OPFLOW)
    int32_t mspRangefinderGetDistanceLast(void);
    int32_t virtualDistanceMm = mspRangefinderGetDistanceLast()*10;
    if ( virtualDistanceMm<10 ) {
        return 0;
    }

    // calc mm factor assuming 42 degrees field view 
    float factor = mspCalcFactor(usec,virtualDistanceMm);

    // convert from mm to pixels for given usec interval 
    float pixels = mmDelta / factor;

    // convert from usec to sec and get pixels per sec 
    float pixelsPerSec = pixels * (float)1000000.0 / (float)usec; 
    return pixelsPerSec;
}

void rangefinderSyncOpflow(int32_t zFromRangeFinder)
{
    const timeUs_t currentTimeUs = micros(); // real time 
    if ( rangeFinderLastValueMm >= 0 && currentTimeUs > rangeFinderLastTimeUsec ) {
        int32_t dt = currentTimeUs - rangeFinderLastTimeUsec;
        int32_t dz = zFromRangeFinder - rangeFinderLastValueMm;
        //sensorData.deltaTime = dt; // currentTimeUs - updatedTimeUs; // detla time in usec 
        float pixelsPerSec = mm2pixelsPerSec( dt, dz );
        sensorData.flowRateRaw[1] = -pixelsPerSec/100;  // convert to pixels per 100hz/10ms 
        hasNewData = true; // let the module know new data is pending 

        static int32_t omri_remove_me1 = 0;
        static float omri_remove_me2 = 0;
        omri_remove_me1 += dz;
        omri_remove_me2 += (pixelsPerSec*(float)dt/1000000.0);

        DEBUG_SET(DEBUG_FLOW_RAW, 5, omri_remove_me1);
        DEBUG_SET(DEBUG_FLOW_RAW, 6, (((int32_t)omri_remove_me2)));

        DEBUG_SET(DEBUG_FLOW_RAW, 4, dz );

        DEBUG_SET(DEBUG_FLOW_RAW, 7, ((int32_t)pixelsPerSec));
    }
    rangeFinderLastValueMm = zFromRangeFinder;
    rangeFinderLastTimeUsec = updatedTimeUs = currentTimeUs; // save time for next round 
}
#endif

// we get in here every either 10 or 20 msec even though sensor sends every 10 msec. 
void mspOpflowReceiveNewData(uint8_t * bufferPtr)
{
    const timeUs_t currentTimeUs = micros();
    const mspSensorOpflowDataMessage_t * pkt = (const mspSensorOpflowDataMessage_t *)bufferPtr;

    sensorData.deltaTime = currentTimeUs - updatedTimeUs;
#ifdef VERTICAL_OPFLOW // inform the range finder that we have a new y value (used as z)
    if ( rangeFinderLastValueMm >= 0 && currentTimeUs > updatedTimeUs ) {
        // convert motionX (speed in cm/sec) to dx in mm - dividve by 10E6 to get micros and multiple by 10 for mm --> 10E5  
        float mmDelta = pixelsPerSec2mmDelta(pkt->motionY, sensorData.deltaTime, rangeFinderLastValueMm);
        void opflowSyncRangefinder(float dMm);
        opflowSyncRangefinder(-mmDelta);
        sensorData.flowRateRaw[0] = pkt->motionX;
//        DEBUG_SET(DEBUG_FLOW_RAW, 1, pkt->motionY);
        sensorData.flowRateRaw[2] = 0;
        sensorData.quality = (int)pkt->quality * 100 / 255;
        hasNewData = true;

#ifdef VERTICAL_OPFLOW_DEMO
        static float VERTICAL_OPFLOW_DEMO_x = 0.0;
        static float VERTICAL_OPFLOW_DEMO_y = 0.0;
        static float VERTICAL_OPFLOW_DEMO_x1 = 0;
        static float VERTICAL_OPFLOW_DEMO_y1 = 0;

        // assume motionX is in pixels per 100hz --> pixels per 10ms --> looks good 
        VERTICAL_OPFLOW_DEMO_x1 += ((float)pkt->motionX * (float)sensorData.deltaTime / (float)10000.0);
        VERTICAL_OPFLOW_DEMO_y1 += ((float)pkt->motionY * (float)sensorData.deltaTime / (float)10000.0);
        VERTICAL_OPFLOW_DEMO_y += mmDelta/10.0;
        mmDelta = pixelsPerSec2mmDelta(pkt->motionX, sensorData.deltaTime, rangeFinderLastValueMm);
        VERTICAL_OPFLOW_DEMO_x += (mmDelta/10.0);
        DEBUG_SET(DEBUG_FLOW, 3, VERTICAL_OPFLOW_DEMO_x);
        DEBUG_SET(DEBUG_FLOW, 4, VERTICAL_OPFLOW_DEMO_y);
        DEBUG_SET(DEBUG_FLOW, 6, VERTICAL_OPFLOW_DEMO_x1);
        DEBUG_SET(DEBUG_FLOW, 7, VERTICAL_OPFLOW_DEMO_y1);

        if ( VERTICAL_OPFLOW_DEMO_x > 100 || VERTICAL_OPFLOW_DEMO_x < -100 ) VERTICAL_OPFLOW_DEMO_x = 0;
        if ( VERTICAL_OPFLOW_DEMO_y > 100 || VERTICAL_OPFLOW_DEMO_y < -100 ) VERTICAL_OPFLOW_DEMO_y = 0;
        if ( VERTICAL_OPFLOW_DEMO_x1 > 10000 || VERTICAL_OPFLOW_DEMO_x1 < -10000 ) VERTICAL_OPFLOW_DEMO_x1 = 0;
        if ( VERTICAL_OPFLOW_DEMO_y1 > 10000 || VERTICAL_OPFLOW_DEMO_y1 < -10000 ) VERTICAL_OPFLOW_DEMO_y1 = 0;
#endif    

//        DEBUG_SET(DEBUG_FLOW_RAW, 0, rangeFinderLastValueMm);
//        DEBUG_SET(DEBUG_FLOW_RAW, 2, sensorData.deltaTime);
//        DEBUG_SET(DEBUG_FLOW_RAW, 3, ((int32_t)mmDelta) );
        //extern float sensorDataF; 
        //DEBUG_SET(DEBUG_FLOW_RAW, 4, ((int32_t)(sensorDataF+0.5)) );
    }
#else
    sensorData.flowRateRaw[0] = pkt->motionX;
    sensorData.flowRateRaw[1] = pkt->motionY;
    sensorData.flowRateRaw[2] = 0;
    sensorData.quality = (int)pkt->quality * 100 / 255;
    hasNewData = true;
#endif    

    updatedTimeUs = currentTimeUs;
}

virtualOpflowVTable_t opflowMSPVtable = {
    .detect = mspOpflowDetect,
    .init = mspOpflowInit,
    .update = mspOpflowUpdate
};
#endif