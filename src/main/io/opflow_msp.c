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



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef USE_OPFLOW_MICOLINK

#include <string.h>
#include "flight/imu.h"

#define MICOLINK_MSG_HEAD            0xEF
#define MICOLINK_MAX_PAYLOAD_LEN     64
#define MICOLINK_MAX_LEN             MICOLINK_MAX_PAYLOAD_LEN + 7
#define MICOLINK_MSG_ID_RANGE_SENSOR 0x51 // Range Sensor

// Message Structure Definition
typedef struct
{
    uint8_t head;                      
    uint8_t dev_id;                          
    uint8_t sys_id;						
    uint8_t msg_id;                        
    uint8_t seq;                          
    uint8_t len;                               
    uint8_t payload[MICOLINK_MAX_PAYLOAD_LEN]; 
    uint8_t checksum;                          

    uint8_t status;                           
    uint8_t payload_cnt;                       
} MICOLINK_MSG_t;

// Payload Definition
#pragma pack (1)
typedef struct
{
    uint32_t  time_ms;		    // System time in ms
    uint32_t  distance;		    // distance(mm), 0 Indicates unavailable
    uint8_t   strength;	            // signal strength
    uint8_t   precision;	    // distance precision
    uint8_t   dis_status;	    // distance status
    uint8_t  reserved1;	            // reserved
    int16_t   flow_vel_x;	    // optical flow velocity in x
    int16_t   flow_vel_y;	    // optical flow velocity in y
    uint8_t   flow_quality;	    // optical flow quality
    uint8_t   flow_status;	    // optical flow status
    uint16_t  reserved2;	    // reserved
} MICOLINK_PAYLOAD_RANGE_SENSOR_t;
#pragma pack ()

static MICOLINK_PAYLOAD_RANGE_SENSOR_t mtf_01;
static float mtf_01_vel_cm_sec[3];
static float mtf_01_move_cm[3]; 
static bool mtf_01_is_init = false;

static bool micolink_check_sum(MICOLINK_MSG_t* msg)
{
    uint8_t length = msg->len + 6;
    uint8_t temp[MICOLINK_MAX_LEN];
    uint8_t checksum = 0;

    memcpy(temp, msg, length);

    for(uint8_t i=0; i<length; i++)
    {
        checksum += temp[i];
    }

    if(checksum == msg->checksum)
        return true;
    else
        return false;
}

static bool micolink_parse_char(MICOLINK_MSG_t* msg, uint8_t data)
{
    switch(msg->status)
    {
    case 0:    
        if(data == MICOLINK_MSG_HEAD)
        {
            msg->head = data;
            msg->status++;
            msg->payload_cnt = 0;
        }
        break;
        
    case 1:     // device id
        if ( data==0x0F ) {
            msg->dev_id = data;
            msg->status++;
        }
        else msg->status = 0;
        break;
    
    case 2:     // system id
        if ( data==0 ) {
            msg->sys_id = data;
            msg->status++;
        }
        else msg->status = 0;
        break;
    
    case 3:     // message id 
        if ( data==0x51 ) {
            msg->msg_id = data;
            msg->status++;
        }
        else msg->status = 0;
        break;
    
    case 4:     // 
        msg->seq = data;
        msg->status++;
        break;
    
    case 5:     // payload length
        if ( data==20 ) {
            msg->len = 20;
            msg->status++;
            msg->payload_cnt = 0;
        }
        else msg->status = 0;
        break;
        
    case 6:     // payload receive
        msg->payload[msg->payload_cnt++] = data;
        if(msg->payload_cnt >= msg->len)
        {
            msg->payload_cnt = 0;
            msg->status++;
        }
        break;
        
    case 7:     // check sum
        msg->checksum = data;
        msg->status = 0;
        if(micolink_check_sum(msg))
        {
            return true;
        }
        msg->payload_cnt = 0;
        break;
        
    default:
        msg->status = 0;
        msg->payload_cnt = 0;
        break;
    }

    return false;
}

bool mtf_01_is_valid(void)
{
    return mtf_01_is_init; // after 2 consecutive good samples 
}

float mtf_01_get_move_cm(int i)
{
    return mtf_01_move_cm[i];
}

float mtf_01_get_velocity_cm_sec(int i)
{
    return mtf_01_vel_cm_sec[i];
}

void micolink_decode(uint8_t data)
{
    static MICOLINK_MSG_t msg;

    if(micolink_parse_char(&msg, data) == false)
        return;
    
    switch(msg.msg_id)
    {
        case MICOLINK_MSG_ID_RANGE_SENSOR:
        {
            memcpy(&mtf_01, msg.payload, msg.len);

#if 0
            // "send" msp packet to the rangefinder 
            void mspRangefinderReceiveNewData(uint8_t * bufferPtr);
            static mspSensorRangefinderDataMessage_t fake_msp;
            fake_msp.distanceMm = mtf_01.distance; // mm 
            fake_msp.quality = mtf_01.strength; // 0-255 
            mspRangefinderReceiveNewData((uint8_t*)&fake_msp);
#endif

            static float distnace_prev = 0.0;
            static uint32_t time_ms_prev = 0;

            // if all valid 
            if ( mtf_01.dis_status>0 && mtf_01.flow_status>0 ) {

                // if this is the 2nd time all is valid --> we can get delta's 
                float tilt = calculateCosTiltAngle(); // do pitch angle tilt correction 
                DEBUG_SET(DEBUG_FLOW_RAW, 7, (tilt*100.0) );
                float distance_cm = (float)mtf_01.distance / 10.0 * tilt; // do pitch angle tilt correction 
                static float distance_cm_filtered = 0;
                
                distance_cm_filtered = distance_cm * 0.1 + distance_cm_filtered * 0.9;
                if ( mtf_01_is_init ) {

                    // distance prep 
                    float distance_factor = (float)mtf_01.distance / 1000.0; // 1000mm <--> 1m
                    float delta_distance = distance_cm_filtered - distnace_prev;

                    // time prep 
                    uint32_t dt = mtf_01.time_ms - time_ms_prev;
                    float time_factor = 1000.0 / (float)dt;
                    DEBUG_SET(DEBUG_FLOW_RAW, 6, dt );

#ifdef VERTICAL_OPFLOW
                    // calculate velocities 
                    mtf_01_vel_cm_sec[0] = (float)mtf_01.flow_vel_x /* cm/sec @1m */ * distance_factor;
                    mtf_01_vel_cm_sec[1] = (float)-delta_distance * time_factor;
                    mtf_01_vel_cm_sec[2] = (float)-mtf_01.flow_vel_y /* cm/sec @1m */ * distance_factor;
#else
                    // calculate velocities 
                    mtf_01_vel_cm_sec[0] = (float)mtf_01.flow_vel_x /* cm/sec @1m */ * distance_factor;
                    mtf_01_vel_cm_sec[1] = (float)mtf_01.flow_vel_y /* cm/sec @1m */ * distance_factor;
                    mtf_01_vel_cm_sec[2] = (float)delta_distance * time_factor;
#endif

                    // calcualte distances 
                    for ( int i=0; i<3; i++ ) {
                        mtf_01_move_cm[i] += mtf_01_vel_cm_sec[i] / time_factor;
                    }
                }
                else {
                    distance_cm_filtered = distance_cm;
                }

                // save for next time 
                mtf_01_is_init = true;
                distnace_prev = distance_cm_filtered;
                time_ms_prev = mtf_01.time_ms;
            }
            else {
                mtf_01_is_init = false; 
                for ( int i=0; i<3; i++ ) {
                    mtf_01_move_cm[i] = 0; // NA 
                }
            }

            DEBUG_SET(DEBUG_FLOW, 3, mtf_01_move_cm[0]);
            DEBUG_SET(DEBUG_FLOW, 4, mtf_01_move_cm[1]);
            DEBUG_SET(DEBUG_FLOW, 5, mtf_01_move_cm[2]);

            DEBUG_SET(DEBUG_FLOW_RAW, 0, mtf_01_vel_cm_sec[0]);
            DEBUG_SET(DEBUG_FLOW_RAW, 1, mtf_01_vel_cm_sec[1]);
            DEBUG_SET(DEBUG_FLOW_RAW, 2, mtf_01_vel_cm_sec[2]);
            DEBUG_SET(DEBUG_FLOW_RAW, 3, mtf_01_move_cm[0]);
            DEBUG_SET(DEBUG_FLOW_RAW, 4, mtf_01_move_cm[1]);
            DEBUG_SET(DEBUG_FLOW_RAW, 5, mtf_01_move_cm[2]);
            break;
        } 

        default:
            break;
        }
}

#endif // USE_OPFLOW_MICOLINK

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(USE_OPFLOW_MSP)

#include "drivers/opflow/opflow_virtual.h"
#include "drivers/time.h"
#include "io/opflow.h"

#include "msp/msp_protocol_v2_sensor_msg.h"


static bool hasNewData = false;
static timeUs_t updatedTimeUs = 0;
#ifdef VERTICAL_OPFLOW // define this to get callback whenever the range finder is updated 
static int32_t rangeFinderLastValueMm = -1;
static timeUs_t updatedTimeUsY = 0;
static timeDelta_t deltaTimeY = 0; // Integration timeframe of motionX/Y
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

#define PIXELS 30 // 30 x 30 
#define MUL_FACTOR ((float)0.0255909356690277*(float)30.0/(float)PIXELS)

#define CRAZY_FACTOR 

static float pixelsPerSec2mmDelta( int32_t deltaPixelsPer10msec, int32_t intervalUSec, int32_t rangeMm)
{
    // how many pixels shift we got during intervalUSec 
    float deltaPixelsPerIntervalUSec = (float)deltaPixelsPer10msec / 10000.0 * (float)intervalUSec;

    // how many mm shift we got during intervalUSec
    float expectedMmDelta = (deltaPixelsPerIntervalUSec * (float)rangeMm * MUL_FACTOR); // 0.0255909356690277 = 2*tan(radians(42/2))/30

#ifdef CRAZY_FACTOR 
    // divide the result be value between 4 and 12 
    float mmDelta = expectedMmDelta / 4.0; 
    if ( rangeMm > 100 ) {
        float div2 = 1.0 + (float)(rangeMm - 100)/1100.0; 
        mmDelta /= div2;
    }

    return mmDelta;
#else
    return expectedMmDelta;
#endif    
}

// mmDelta is the delta in the range finder during the past usec time 
// mmDelta is convertedto pixels and is actually delta in the x axis in case the point of view was top/down 
static float mm2pixels(int32_t mmDelta)
{
    // the distance the virtual range finder believes it sees (it is actually the accumulation of dx from the OPFLOW)
    int32_t mspRangefinderGetDistanceLast(void);
    int32_t virtualDistanceMm = mspRangefinderGetDistanceLast()*10;
    if ( virtualDistanceMm<10 ) {
        return 0;
    }

    // calc mm factor assuming 42 degrees field view 
    float expPixels = (float)mmDelta / (float)virtualDistanceMm / MUL_FACTOR; // distance of the OPFLOW from the surfface in mm (this translates pixels later)

#ifdef CRAZY_FACTOR 
    // divide the result be value between 4 and 12 
    float pixels = expPixels * 4.0; 
    if ( virtualDistanceMm > 100 ) {
        float div2 = 1.0 + (float)(virtualDistanceMm - 100)/1100.0; 
        pixels *= div2;
    }

    return pixels;
#else
    return expPixels;
#endif    
}

void rangefinderSyncOpflow(int32_t zFromRangeFinder)
{
    const timeUs_t currentTimeUs = micros(); // real time 
    if ( rangeFinderLastValueMm >= 0 && currentTimeUs > updatedTimeUsY ) {
        deltaTimeY = currentTimeUs - updatedTimeUsY;
        int32_t dz = zFromRangeFinder - rangeFinderLastValueMm;
        float pixels = mm2pixels(dz);
        sensorData.flowRateRaw[1] = -pixels; 
    }
    rangeFinderLastValueMm = zFromRangeFinder;
    updatedTimeUsY = currentTimeUs; // save time for next round 
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
        if ( deltaTimeY > 0 ) { // sync to same time base 
            sensorData.flowRateRaw[1] *= (sensorData.deltaTime/deltaTimeY); 
        }
        sensorData.flowRateRaw[2] = 0;
        sensorData.quality = (int)pkt->quality * 100 / 255;
        hasNewData = true;
#ifdef VERTICAL_OPFLOW_DEMO
        // cm 

        static float cm_accumulated_y = 0.0;
        cm_accumulated_y += mmDelta/10.0;

        //DEBUG_SET(DEBUG_FLOW_RAW, 0, pkt->motionY);
        //DEBUG_SET(DEBUG_FLOW_RAW, 1, sensorData.deltaTime);
        //DEBUG_SET(DEBUG_FLOW_RAW, 2, rangeFinderLastValueMm);
        //DEBUG_SET(DEBUG_FLOW_RAW, 3, (mmDelta*100.0));
        //DEBUG_SET(DEBUG_FLOW_RAW, 4, cm_accumulated_y);

        DEBUG_SET(DEBUG_FLOW, 4, cm_accumulated_y);
        static float cm_accumulated_x = 0.0;
        mmDelta = pixelsPerSec2mmDelta(pkt->motionX, sensorData.deltaTime, rangeFinderLastValueMm);
        cm_accumulated_x += (mmDelta/10.0);
        DEBUG_SET(DEBUG_FLOW, 3, cm_accumulated_x);

        // pixels 
        static float pixels_accumulated_x = 0;
        pixels_accumulated_x += ((float)pkt->motionX * (float)sensorData.deltaTime / (float)10000.0);
        DEBUG_SET(DEBUG_FLOW, 6, pixels_accumulated_x);
        static float pixels_accumulated_y = 0;
        pixels_accumulated_y += ((float)pkt->motionY * (float)sensorData.deltaTime / (float)10000.0);
        DEBUG_SET(DEBUG_FLOW, 7, pixels_accumulated_y);
#endif    
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
#if 0
void all_in_one()
{
    // opflow.bodyRate = accumulated DPS during opflow.dev.rawData.deltaTime microseconds 
    // opflow.flowRate = accumulated pixels shift during opflow.dev.rawData.deltaTime microseconds 
    // (float)opticalFlowConfig()->opflow_scale is usually 5.4 

    const float integralToRateScaler = (1.0e6f / opflow.dev.rawData.deltaTime) / (float)opticalFlowConfig()->opflow_scale;

    // Calculate flow rate and accumulated body rate
    opflow.flowRate[X] = opflow.dev.rawData.flowRateRaw[X] * integralToRateScaler;
    opflow.flowRate[Y] = opflow.dev.rawData.flowRateRaw[Y] * integralToRateScaler;

    opflow.bodyRate[X] = DEGREES_TO_RADIANS(opflow.bodyRate[X]);
    opflow.bodyRate[Y] = DEGREES_TO_RADIANS(opflow.bodyRate[Y]);
    opflow.flowRate[X] = DEGREES_TO_RADIANS(opflow.flowRate[X]);
    opflow.flowRate[Y] = DEGREES_TO_RADIANS(opflow.flowRate[Y]);

    // Calculate linear velocity based on angular velocity and altitude
    // Technically we should calculate arc length here, but for fast sampling this is accurate enough
    fpVector3_t flowVel = {
        .x = - (opflow.flowRate[Y] - opflow.bodyRate[Y]) * posEstimator.surface.alt,
        .y =   (opflow.flowRate[X] - opflow.bodyRate[X]) * posEstimator.surface.alt,
        .z =    posEstimator.est.vel.z
    };

    // From body frame to earth frame
    typedef struct {
        float q0, q1, q2, q3;
    } fpQuaternion_t;
    extern FASTRAM fpQuaternion_t orientation;
    quaternionRotateVectorInv(&flowVel, &flowVel, &orientation);

    // HACK: This is needed to correctly transform from NED (sensor frame) to NEU (navigation)
    flowVel.y = -flowVel.y;

    // At this point flowVel will hold linear velocities in earth frame

    // Calculate velocity correction
    const float flowVelXInnov = flowVel.x - posEstimator.est.vel.x;
    const float flowVelYInnov = flowVel.y - posEstimator.est.vel.y;

    ctx->estVelCorr.x = flowVelXInnov * positionEstimationConfig()->w_xy_flow_v * ctx->dt;
    ctx->estVelCorr.y = flowVelYInnov * positionEstimationConfig()->w_xy_flow_v * ctx->dt;

    posEstimator.est.flowCoordinates[X] += flowVel.x * ctx->dt;
    posEstimator.est.flowCoordinates[Y] += flowVel.y * ctx->dt;

    const float flowResidualX = posEstimator.est.flowCoordinates[X] - posEstimator.est.pos.x;
    const float flowResidualY = posEstimator.est.flowCoordinates[Y] - posEstimator.est.pos.y;

    ctx->estPosCorr.x = flowResidualX * positionEstimationConfig()->w_xy_flow_p * ctx->dt;
    ctx->estPosCorr.y = flowResidualY * positionEstimationConfig()->w_xy_flow_p * ctx->dt;

    ctx->newEPH = updateEPE(posEstimator.est.eph, ctx->dt, calc_length_pythagorean_2D(flowResidualX, flowResidualY), positionEstimationConfig()->w_xy_flow_p);
}
#endif
#endif