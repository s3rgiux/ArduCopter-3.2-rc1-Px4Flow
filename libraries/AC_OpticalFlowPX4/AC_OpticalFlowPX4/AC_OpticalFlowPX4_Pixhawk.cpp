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

const AP_Param::GroupInfo AC_OpticalFlowPX4_Pixhawk::var_info[] PROGMEM = {
AP_GROUPINFO("ALPHA_OF",  0,  AC_OpticalFlowPX4_Pixhawk, _alpha_of, ALPHA_OF_DEFAULT),
AP_GROUPINFO("KP_POS",  1,  AC_OpticalFlowPX4_Pixhawk, _kp_pos_of, KP_POS_OF_DEFAULT),
AP_GROUPEND
};

void AC_OpticalFlowPX4_Pixhawk::init()
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

void AC_OpticalFlowPX4_Pixhawk::get_of_angles(float vx_in,float vy_in, int16_t &pitch_out, int16_t &roll_out, float G_Dt)
{
    Vector3f vel_of_error,angles_target;
	
	
    vx_in = constrain_float(vx_in, -200, 200);//max velocity
    vy_in = constrain_float(vy_in, -200, 200);

	//filtering of
	vel_of_x=vel_of_x*_alpha_of+((1-_alpha_of)*(flow_comp_m_x*100));//en cm
    vel_of_y=vel_of_y*_alpha_of+((1-_alpha_of)*(flow_comp_m_y*100));//en cm
	
	pos_of_x+=vel_of_x*G_Dt;
    pos_of_y+=vel_of_y*G_Dt;
	
	vel_of_error.x = (vx_in - vel_of_x);
    vel_of_error.y =(vy_in - vel_of_y);
  
    angles_target.x = _pid_optflow.get_pid(vel_of_error.x, G_Dt);
    angles_target.y = _pid_optflow.get_pid(vel_of_error.y, G_Dt);

    target_roll = constrain_float(safe_asin(angles_target.y)*(18000/M_PI), -1500, 1500);
    target_pitch = constrain_float(safe_asin(-angles_target.x)*(18000/M_PI),-1500, 1500);

    
    //targetsAng_XY.x=roll_target;
    //targetsAng_XY.y=pitch_target;
  
  tmr_of_rst=hal.scheduler->millis();
  if(tmr_of_rst>=tmr_of_rst_n+40000){
    pos_of_x=0;
    pos_of_y=0;
    tmr_of_rst_n=hal.scheduler->millis();
  }
  
  
    roll_out = (int16_t)target_roll;
    pitch_out = (int16_t)target_pitch;
}


void AC_OpticalFlowPX4_Pixhawk::get_sp_vel(int16_t in_x,int16_t in_y, float &v_out_x, float &v_out_y)
{
	if(labs(in_y)>300||labs(in_x)>300){
		v_out_x=-in_x/20;
		v_out_y=in_y/20;
		pos_of_x=0;pos_of_y=0;
	}else{
		v_out_x=_kp_pos_of*(0-pos_of_x);
		v_out_y=_kp_pos_of*(0-pos_of_y);
	}
}


#endif
