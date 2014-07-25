// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-


#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "AC_OpticalFlowPX4_Pixhawk.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_px4flow.h>
#include <drivers/drv_hrt.h>


extern const AP_HAL::HAL& hal;



bool AC_OpticalFlowPX4_Pixhawk::init()
{
 hal.scheduler->delay(10);
 //hal.uartA->printf("flujo_init");
 
 _px4flow = open(PX4FLOW_DEVICE_PATH, O_RDONLY);
        if (_px4flow < 0) {
            hal.scheduler->panic("Unable to open " PX4FLOW_DEVICE_PATH);
			hal.uartA->printf("flujo_no_ok");
        }else{
			hal.uartA->printf("flujo_ok");
		}
    /* Initialize the PX4FLOW SENSOR*/
     ioctl(_px4flow, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MAX);
     ioctl(_px4flow, SENSORIOCSQUEUEDEPTH, 1);
        hal.scheduler->delay(40);
	
}


void AC_OpticalFlowPX4_Pixhawk::update()
{
struct px4flow_report px4flow_report;
    while (::read(_px4flow, &px4flow_report, sizeof(px4flow_report)) == sizeof(px4flow_report) &&
           px4flow_report.timestamp != _last_timestamp) {
        _last_timestamp = px4flow_report.timestamp;
		flow_comp_m_x=px4flow_report.flow_comp_x_m;
		flow_comp_m_y=px4flow_report.flow_comp_y_m;
		quality=px4flow_report.quality;
    }  
}
#endif
