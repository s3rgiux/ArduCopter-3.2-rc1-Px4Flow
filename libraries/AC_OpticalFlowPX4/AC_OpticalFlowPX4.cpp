// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "AC_OpticalFlowPX4.h"
#include <AP_Common.h>
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_OpticalFlowPX4::var_info[] PROGMEM = {
AP_GROUPINFO("ALPHA_OF",  0,  AC_OpticalFlowPX4, _alpha_of, ALPHA_OF_DEFAULT),
AP_GROUPINFO("KP_POS",  1,  AC_OpticalFlowPX4, _kp_pos_of, KP_POS_OF_DEFAULT),
AP_GROUPEND
};


void AC_OpticalFlowPX4::init()
{
 hal.scheduler->delay(10);
 _i2c_sem = hal.i2c->get_semaphore();
    if (!_i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        hal.scheduler->panic(PSTR("Failed to get Optical Flow semaphore"));
    }
	_i2c_sem->give();
}

// init_targets - resets target angles to current angles
void AC_OpticalFlowPX4::update()
{

    if (!_i2c_sem->take(1)) {
       // the bus is busy - try again later
       return;
   }
    uint8_t stat = hal.i2c->readRegisters((uint8_t)OP_FLOW_ADRESS,0x00,22, data_of);
  if (stat==0){
    //frame_count=data_of[1]<<8|data_of[0];
	//pixel_flow_x_sum=data_of[3]<<8|data_of[2];
	//pixel_flow_y_sum=data_of[5]<<8|data_of[4];
	flow_comp_m_x_i=(data_of[7]<<8|data_of[6]);
	flow_comp_m_y_i=(data_of[9]<<8|data_of[8]);
	flow_comp_m_x=(float)flow_comp_m_x_i/1000;
	flow_comp_m_y=(float)flow_comp_m_y_i/1000;
	quality=data_of[11]<<8|data_of[10];
	//gyro_x_rate=data_of[13]<<8|data_of[12];
	//gyro_y_rate=data_of[15]<<8|data_of[14];
	//gyro_z_rate=data_of[17]<<8|data_of[16];
	//gyro_range=data_of[18];
	//sonar_timestamp=data_of[19];
	ground_distance=(data_of[21]<<8|data_of[20])/1000;

  }
   _i2c_sem->give();
  //vel_of_x=vel_of_x*_filter+((1-_filter)*flow_comp_m_x/10);
  //vel_of_y=vel_of_y*_filter+((1-_filter)*flow_comp_m_y/10);
  
}

//static void get_of_angles(int16_t roll_in, int16_t pitch_in, int16_t &roll_out, int16_t &pitch_out)
void AC_OpticalFlowPX4::get_of_angles(float vx_in,float vy_in, int16_t &pitch_out, int16_t &roll_out, float G_Dt)
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


void AC_OpticalFlowPX4::get_sp_vel(int16_t in_x,int16_t in_y, float &v_out_x, float &v_out_y)
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

