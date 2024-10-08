#pragma once

#define WORK_WITHOUT_RC_FROM_CLI    0 // work without RC from CLI 
#define BLACKBOX_USING_DEBUG        0

//#define VGPS                        // do  waypints navigation mission where the dorne flies 5m to a wall and back streaming virutal GPS data (UBLOX protocol) 
#define VERTICAL_OPFLOW             // use MTF-01 optical-flow and rangefinder sensor pointing forwards instead of downwards 
//#define VERTICAL_OPFLOW_DIRECT
#define VERTICAL_OPFLOW_DEMO        // stream data to DEBUG_FLOW to allow nice tracking of the workings of VERTICAL_OPFLOW feature throuhg the Configurator Sensors tab 
