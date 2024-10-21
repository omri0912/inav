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

#include <string.h>
#include "flight/imu.h"
#include "sensors/opflow.h"
#include "sensors/rangefinder.h"

#define MICOLINK_MSG_HEAD            0xEF
#define MICOLINK_MAX_PAYLOAD_LEN     64
#define MICOLINK_MAX_LEN             MICOLINK_MAX_PAYLOAD_LEN + 7
#define MICOLINK_MSG_ID_RANGE_SENSOR 0x51 // Range Sensor

#if SWITCH_OPFLOW_EVERY_10SEC 
#include "drivers/time.h"
#else
#include "fc/rc_modes.h"
#endif

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
static float mtf_01_move_cm_gyro[2];
static bool mtf_01_is_init = false;
static bool mtf_01_swap = false;
static MICOLINK_MSG_t mtf_01_msg;
bool mtf_01_facing_down = true;
serialPortIdentifier_e mtf_01_identifier = SERIAL_PORT_NONE;


void mtf_01_init(void)
{
    // 115 --> uart5 facing the wall  
    // 104 --> uart5 facing down 
    uint32_t val = (uint32_t)(opticalFlowConfig()->opflow_scale);
    uint32_t uart = val % 10;
    val /= 10;
    uint32_t is_facing_wall = val % 10;
    val /= 10;

// serial 0 0 115200 115200 0 115200
// serial 1 1 115200 115200 0 115200
// set opflow_scale = 114.0
// set opflow_scale = 102.0

    // 1 is just a "secret" code 1xy 
    if ( val==1 && is_facing_wall<2 && uart>0 && uart<9 ) {
        mtf_01_facing_down = is_facing_wall ? false : true;
        mtf_01_identifier = uart-1;
    }
    else {
        mtf_01_facing_down = true;
        mtf_01_identifier = SERIAL_PORT_NONE; // MSP 
    }

#if SCALE_ALTITUDE_AT_ALTHOLD
    void flyz_throttle_span_init(void);
    flyz_throttle_span_init();
#endif
}

static void mtf_01_init_at_runtime(bool is_facing_wall)
{
    mtf_01_facing_down = is_facing_wall ? false : true;
    mtf_01_swap = true;
    mtf_01_msg.status = 0;
}

bool mtf_01_is_micolink(void)
{
    return mtf_01_identifier != SERIAL_PORT_NONE;
}

bool mtf_01_is_facing_down(void)
{
    return mtf_01_facing_down;
}

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

#if DISABLE_GPS_AT_ALTHOLD
void flyz_set_agl(int32_t gps_agl_guess_cm)
{
    // force new AGL for GPS 
    if ( mtf_01_is_init ) {
        mtf_01_move_cm[2] = (float)gps_agl_guess_cm;
        (void)rangefinderProcess(1.0);
    }
}
#endif

void mtf_01_micolink_decode(serialPortIdentifier_e identifier, uint8_t data)
{
    // if this uart (identifier) is not a micolink uart - do nothing 
    if ( mtf_01_swap ? identifier==mtf_01_identifier : identifier!=mtf_01_identifier ) {
        return;
    }

    if(micolink_parse_char(&mtf_01_msg, data) == false)
        return;
    
    switch(mtf_01_msg.msg_id)
    {
        case MICOLINK_MSG_ID_RANGE_SENSOR:
        {
            memcpy(&mtf_01, mtf_01_msg.payload, mtf_01_msg.len);

            static float distance_cm_prev = 0.0;
            static uint32_t time_ms_prev = 0;
            static float fdistance_cm = 0;

            if ( mtf_01_swap ) {
                mtf_01_swap = false;
                mtf_01_identifier = identifier;
                mtf_01_is_init = false;
                fdistance_cm = 0;
            }

            // if all valid 
            if ( mtf_01.dis_status>0 && mtf_01.flow_status>0 ) {

                // do pitch angle tilt correction - tilt is the current tilt. no averaging or accumulating is done here 
                // omri-todo - inav-bug - get average titl in this same interval (typically this is a 10ms interval - sampiling at 100hz)
                float tilt = calculateCosTiltAngle(); 
                float tilt_percent = (1-tilt)*100;
                DEBUG_SET(DEBUG_FLOW_RAW, 7, tilt_percent);
                float distance_cm = (float)mtf_01.distance / 10.0 * tilt; // do pitch angle tilt correction 
                if ( fdistance_cm==0 ) {
                    fdistance_cm = distance_cm;
                }
                else {
                    fdistance_cm = 0.9 * fdistance_cm + distance_cm * 0.1;
                }

                // if this is the 2nd time (mtf_01_is_init is true) all is valid --> we can get delta's 
                if ( mtf_01_is_init ) {

                    // distance prep 
                    float distance_1m_factor = (float)mtf_01.distance / 1000.0; // 1000mm <--> 1m
                    float delta_distance_cm = fdistance_cm - distance_cm_prev;

                    // time prep 
                    uint32_t dt_ms = mtf_01.time_ms - time_ms_prev;
                    float time_1sec_factor = 1000.0 / (float)dt_ms;
                    DEBUG_SET(DEBUG_FLOW_RAW, 6, dt_ms );

                    // x velocity in cm per second deducting gyro rotational effect in the same plane 
                    bool is_garbage = false;
                    float vel_x = (float)mtf_01.flow_vel_x /* cm/sec @1m */ * distance_1m_factor;
                    if ( opflow.gyroBodyRateTimeUs > 0) {    
                        float gyro_dps = opflow.gyroBodyRateAcc[0] / (float)opflow.gyroBodyRateTimeUs; // gyroBodyRateAcc = sum(dps[i]*period_in_us[i]) --> avg_dps = gyroBodyRateAcc/sum(period_in_us[i])
                        gyro_dps = DEGREES_TO_RADIANS(gyro_dps);
                        float vel_gyro = gyro_dps * fdistance_cm; 
                        mtf_01_move_cm_gyro[0] += vel_gyro / time_1sec_factor;
                        vel_x += vel_gyro; // add since direction is inverted with the gyro and the opflow in the x axis (omri-todo - investage why) 
                    }
                    if ( vel_x > 2000 || vel_x < -2000 ) { // ~70 km/h max theoretic drone speed 
                        is_garbage = true;
                    }

                    // y velocity in cm per second deducting gyro rotational effect in the same plane 
                    float vel_y = (float)mtf_01.flow_vel_y /* cm/sec @1m */ * distance_1m_factor;
                    if (opflow.gyroBodyRateTimeUs > 0) {
                        float gyro_dps = opflow.gyroBodyRateAcc[1] / (float)opflow.gyroBodyRateTimeUs; // gyroBodyRateAcc = sum(dps[i]*period_in_us[i]) --> avg_dps = gyroBodyRateAcc/sum(period_in_us[i])
                        gyro_dps = DEGREES_TO_RADIANS(gyro_dps);
                        float vel_gyro = gyro_dps * fdistance_cm; 
                        mtf_01_move_cm_gyro[1] += vel_gyro / time_1sec_factor;
                        vel_y -= vel_gyro;
                        vel_y = -vel_y; // invert 
                    }
                    if ( vel_y > 2000 || vel_y < -2000 ) { // 2000 cm/sec = 0.02*3600 km/h = 72 km/h max theoretic drone speed 
                        is_garbage = true;
                    }

                    // z velocity in cm per second - gyro not relevant here, just the tilt, which was already deducted 
                    float vel_z = (float)delta_distance_cm * time_1sec_factor;
                    if ( vel_z > 1000 || vel_z < -1000 ) { // 35 km/h 
                        is_garbage = true;
                    }

                    // rare event of garbage --> ignore it 
                    if ( is_garbage ) {
                        opflow.gyroBodyRateAcc[0] = 0;
                        opflow.gyroBodyRateAcc[1] = 0;
                        opflow.gyroBodyRateTimeUs = 0;
                        return;
                    }

                    if ( !mtf_01_facing_down ) {
                        // set velocities relative to body frame - when sensor is facing the wall y/z are flipped and their signes are flipped as well
                        mtf_01_vel_cm_sec[0] = vel_x; // drone moves right/left --> indiferent to wall facing or ground facing 
                        mtf_01_vel_cm_sec[1] = -vel_z; // drone moves forwards/backwards --> this is 'z' which is the rangefinder delta's 
                        mtf_01_vel_cm_sec[2] = vel_y; // drone moves up/down --> this is the 'y' relative to the vertical wall we're facing 
                    }
                    else {
                        // set velocities relative to body frame - when sensor is facing down al axis are remain aligned 
                        mtf_01_vel_cm_sec[0] = vel_x; // drone moves right/left 
                        mtf_01_vel_cm_sec[1] = vel_y; // drone moves forwards/backwards 
                        mtf_01_vel_cm_sec[2] = vel_z; // drone moves up/down 
                    }

                    // accumulate distances 
                    mtf_01_move_cm[0] += mtf_01_vel_cm_sec[0] / time_1sec_factor;
                    mtf_01_move_cm[1] += mtf_01_vel_cm_sec[1] / time_1sec_factor;
                    mtf_01_move_cm[2] += mtf_01_vel_cm_sec[2] / time_1sec_factor;
                }

                // save for next time 
                mtf_01_is_init = true; // mark this the next round may use delta's for sure 
                distance_cm_prev = fdistance_cm;
                time_ms_prev = mtf_01.time_ms;
            }

            // we're lost --> reset 
            else {
                // we're lost!
                mtf_01_is_init = false; 
            }

            // zero gyro accumulators every time we visit here 
            opflow.gyroBodyRateAcc[0] = 0;
            opflow.gyroBodyRateAcc[1] = 0;
            opflow.gyroBodyRateTimeUs = 0;

            DEBUG_SET(DEBUG_FLOW, 3, mtf_01_move_cm[0]);
            DEBUG_SET(DEBUG_FLOW, 4, mtf_01_move_cm[1]);
            DEBUG_SET(DEBUG_FLOW, 5, mtf_01_move_cm[2]);

            DEBUG_SET(DEBUG_FLOW_RAW, 0, mtf_01_move_cm_gyro[0]);
            DEBUG_SET(DEBUG_FLOW_RAW, 1, mtf_01_move_cm_gyro[1]);
            DEBUG_SET(DEBUG_FLOW_RAW, 2, mtf_01_vel_cm_sec[2]);
            DEBUG_SET(DEBUG_FLOW_RAW, 3, mtf_01_move_cm[0]);
            DEBUG_SET(DEBUG_FLOW_RAW, 4, mtf_01_move_cm[1]);
            DEBUG_SET(DEBUG_FLOW_RAW, 5, mtf_01_move_cm[2]);
            
#if SWITCH_OPFLOW_EVERY_10SEC 
            uint32_t t = millis();
            static uint32_t timeout = 20000;
            if ( t >= timeout ) {
                timeout += 10000;
                mtf_01_init_at_runtime(mtf_01_facing_down);
                DEBUG_SET(DEBUG_FLOW, 6, timeout/10000);
                DEBUG_SET(DEBUG_FLOW, 7, mtf_01_facing_down);
            }
#elif MUX_FOR_OPFLOW_SWITCH
            // at end of a packet - check if AUX changed 
            if ( IS_RC_MODE_ACTIVE(BOXLOITERDIRCHN) ? mtf_01_facing_down : !mtf_01_facing_down ) {
                mtf_01_init_at_runtime(mtf_01_facing_down);
                DEBUG_SET(DEBUG_FLOW, 7, mtf_01_facing_down);
            }
#endif
            break;
        } 

        default:
            break;
        }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(USE_OPFLOW_MSP)

#include "drivers/opflow/opflow_virtual.h"
#include "drivers/time.h"
#include "io/opflow.h"

#include "msp/msp_protocol_v2_sensor_msg.h"


static bool hasNewData = false;
static timeUs_t updatedTimeUs = 0;
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

void mspOpflowReceiveNewData(uint8_t * bufferPtr)
{
    const timeUs_t currentTimeUs = micros();
    const mspSensorOpflowDataMessage_t * pkt = (const mspSensorOpflowDataMessage_t *)bufferPtr;

    sensorData.deltaTime = currentTimeUs - updatedTimeUs;
    sensorData.flowRateRaw[0] = pkt->motionX;
    sensorData.flowRateRaw[1] = pkt->motionY;
    sensorData.flowRateRaw[2] = 0;
    sensorData.quality = (int)pkt->quality * 100 / 255;
    hasNewData = true;

    updatedTimeUs = currentTimeUs;
}

virtualOpflowVTable_t opflowMSPVtable = {
    .detect = mspOpflowDetect,
    .init = mspOpflowInit,
    .update = mspOpflowUpdate
};

#endif
