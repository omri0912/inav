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
#include <math.h>
#include <string.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"
#include "navigation/navigation_pos_estimator_private.h"

#include "sensors/rangefinder.h"
#include "sensors/opflow.h"

#include "flight/imu.h"

#include "flyz.h"

extern navigationPosEstimator_t posEstimator;

#ifdef USE_OPFLOW
/**
 * Read optical flow topic
 *  Function is called by OPFLOW task as soon as new update is available
 */
void updatePositionEstimator_OpticalFlowTopic(timeUs_t currentTimeUs)
{
    posEstimator.flow.lastUpdateTime = currentTimeUs;
    posEstimator.flow.isValid = opflow.isHwHealty && (opflow.flowQuality == OPFLOW_QUALITY_VALID);
    posEstimator.flow.flowRate[X] = opflow.flowRate[X];
    posEstimator.flow.flowRate[Y] = opflow.flowRate[Y];
    posEstimator.flow.bodyRate[X] = opflow.bodyRate[X];
    posEstimator.flow.bodyRate[Y] = opflow.bodyRate[Y];
}
#endif

bool estimationCalculateCorrection_XY_FLOW(estimationContext_t * ctx)
{
#if defined(USE_RANGEFINDER) && defined(USE_OPFLOW)

#ifdef VERTICAL_OPFLOW_DEMO
    static float  x = 0.0;
    static float y = 0.0;
    x += ((posEstimator.flow.flowRate[X] - posEstimator.flow.bodyRate[X]) * posEstimator.surface.alt)*ctx->dt;
    y += ((posEstimator.flow.flowRate[Y] - posEstimator.flow.bodyRate[Y]) * posEstimator.surface.alt)*ctx->dt;
    DEBUG_SET(DEBUG_FLOW, 0, x);
    DEBUG_SET(DEBUG_FLOW, 1, y);
    DEBUG_SET(DEBUG_FLOW, 2, posEstimator.surface.alt);
#endif
/* 1001100
    EST_GPS_XY_VALID            = (1 << 0), // 0
    EST_GPS_Z_VALID             = (1 << 1), // 0
    EST_BARO_VALID              = (1 << 2), // 1
    EST_SURFACE_VALID           = (1 << 3), // 1
    EST_FLOW_VALID              = (1 << 4), // 0 !!!
    EST_XY_VALID                = (1 << 5), // 0 !!!
    EST_Z_VALID                 = (1 << 6), // 1 */

    if (!((ctx->newFlags & EST_FLOW_VALID) && (ctx->newFlags & EST_SURFACE_VALID) && (ctx->newFlags & EST_Z_VALID))) {
        DEBUG_SET(DEBUG_FLOW, 0, ctx->newFlags);
        DEBUG_SET(DEBUG_FLOW, 1, posEstimator.surface.reliability );
        return false;
    }

    // FIXME: flow may use AGL estimate if available
    const bool canUseFlow = (posEstimator.surface.reliability >= RANGEFINDER_RELIABILITY_LOW_THRESHOLD);

    if (!canUseFlow) {
        DEBUG_SET(DEBUG_FLOW, 0, ctx->newFlags);
        DEBUG_SET(DEBUG_FLOW, 1, posEstimator.surface.reliability );
        return false;
    }

    // Calculate linear velocity based on angular velocity and altitude
    // Technically we should calculate arc length here, but for fast sampling this is accurate enough
#if defined(USE_OPFLOW_MICOLINK) 
    extern int   USE_OPFLOW_MICOLINK_flip; // 1 
    extern float USE_OPFLOW_MICOLINK_signx; // -1 1
    extern float USE_OPFLOW_MICOLINK_signy; // 1 

    // opflow_scale = 111.0 --> [1/2]=signy [1/2]=signx [1/2]=is_flip_xy default=110 
    float mtf_01_get_velocity_cm_sec(int i);
    fpVector3_t flowVel = {
        .x = USE_OPFLOW_MICOLINK_signx*mtf_01_get_velocity_cm_sec(USE_OPFLOW_MICOLINK_flip), // -Y 
        .y = USE_OPFLOW_MICOLINK_signy*mtf_01_get_velocity_cm_sec(1-USE_OPFLOW_MICOLINK_flip), // +X
        .z =  mtf_01_get_velocity_cm_sec(2)  // +Z
    };
#else        
    fpVector3_t flowVel = {
        .x = - (posEstimator.flow.flowRate[Y] - posEstimator.flow.bodyRate[Y]) * posEstimator.surface.alt,
        .y =   (posEstimator.flow.flowRate[X] - posEstimator.flow.bodyRate[X]) * posEstimator.surface.alt,
        .z =    posEstimator.est.vel.z
    };
#endif        

#if 1 //
    static float x = 0.0;
    static float y = 0.0;
    static float z = 0.0;
    x += flowVel.x*ctx->dt;
    y += flowVel.y*ctx->dt;
    z += flowVel.z*ctx->dt;
    DEBUG_SET(DEBUG_FLOW, 0, x);
    DEBUG_SET(DEBUG_FLOW, 1, y);
    DEBUG_SET(DEBUG_FLOW, 2, z);
#endif

    // At this point flowVel will hold linear velocities in earth frame
    imuTransformVectorBodyToEarth(&flowVel);

    // Calculate velocity correction
    const float flowVelXInnov = flowVel.x - posEstimator.est.vel.x;
    const float flowVelYInnov = flowVel.y - posEstimator.est.vel.y;

    ctx->estVelCorr.x = flowVelXInnov * positionEstimationConfig()->w_xy_flow_v * ctx->dt;
    ctx->estVelCorr.y = flowVelYInnov * positionEstimationConfig()->w_xy_flow_v * ctx->dt;

    // Calculate position correction if possible/allowed
    if ((ctx->newFlags & EST_GPS_XY_VALID)) {
        // If GPS is valid - reset flow estimated coordinates to GPS
        posEstimator.est.flowCoordinates[X] = posEstimator.gps.pos.x;
        posEstimator.est.flowCoordinates[Y] = posEstimator.gps.pos.y;
    }
    else if (positionEstimationConfig()->allow_dead_reckoning) {
        posEstimator.est.flowCoordinates[X] += flowVel.x * ctx->dt;
        posEstimator.est.flowCoordinates[Y] += flowVel.y * ctx->dt;

        const float flowResidualX = posEstimator.est.flowCoordinates[X] - posEstimator.est.pos.x;
        const float flowResidualY = posEstimator.est.flowCoordinates[Y] - posEstimator.est.pos.y;

        ctx->estPosCorr.x = flowResidualX * positionEstimationConfig()->w_xy_flow_p * ctx->dt;
        ctx->estPosCorr.y = flowResidualY * positionEstimationConfig()->w_xy_flow_p * ctx->dt;

#if 1 //
        DEBUG_SET(DEBUG_FLOW, 6, posEstimator.est.flowCoordinates[X]);
        DEBUG_SET(DEBUG_FLOW, 7, posEstimator.est.flowCoordinates[Y]);
#endif

        ctx->newEPH = updateEPE(posEstimator.est.eph, ctx->dt, calc_length_pythagorean_2D(flowResidualX, flowResidualY), positionEstimationConfig()->w_xy_flow_p);
    }

#if 0 // ndef VERTICAL_OPFLOW_DEMO
    DEBUG_SET(DEBUG_FLOW, 0, RADIANS_TO_DEGREES(posEstimator.flow.flowRate[X]));
    DEBUG_SET(DEBUG_FLOW, 1, RADIANS_TO_DEGREES(posEstimator.flow.flowRate[Y]));
    DEBUG_SET(DEBUG_FLOW, 2, posEstimator.est.flowCoordinates[X]);
    DEBUG_SET(DEBUG_FLOW, 3, posEstimator.est.flowCoordinates[Y]);
    DEBUG_SET(DEBUG_FLOW, 4, ((int32_t)(10*ctx->estPosCorr.y)));
    DEBUG_SET(DEBUG_FLOW, 7, (posEstimator.surface.alt));
#endif

    return true;
#else
    UNUSED(ctx);
    return false;
#endif
}
