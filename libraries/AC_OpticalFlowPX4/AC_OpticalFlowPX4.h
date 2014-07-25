// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file    AC_AttitudeControl.h
/// @brief   ArduCopter attitude control library

#ifndef AC_OpticalFlowPX4_H
#define AC_OpticalFlowPX4_H

#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_InertialSensor.h>
#include <AP_AHRS.h>
#include <AP_Motors.h>
#include <DataFlash.h>
#include <AC_PID.h>
#include <AC_P.h>

#define OP_FLOW_ADRESS  0x42

#define ALPHA_OF_DEFAULT 0.95f
#define KP_POS_OF_DEFAULT   0.99f

class AC_OpticalFlowPX4 {
public:

	//AC_OpticalFlow(int_t adress) :
	
		//_adress(adress)
		//AC_OpticalFlow(float filter) :
		//_filter(filter)
		AC_OpticalFlowPX4(AC_PID& pid_optflow):
        _pid_optflow(pid_optflow)
		{
			//inits
			
			AP_Param::setup_object_defaults(this, var_info);
		}
		
static const struct AP_Param::GroupInfo        var_info[];		
uint8_t data_of[25];
uint16_t frame_count;// counts created I2C frames
int16_t pixel_flow_x_sum;// accumulated x flow in pixels*10 since last I2C frame
int16_t pixel_flow_y_sum;// accumulated y flow in pixels*10 since last I2C frame
int16_t flow_comp_m_x_i;
int16_t flow_comp_m_y_i;
float flow_comp_m_x;// x velocity*1000 in meters / timestep
float flow_comp_m_y;// y velocity*1000 in meters / timestep
int16_t quality;// Optical flow quality / confidence 0: bad, 255: maximum quality
int16_t gyro_x_rate; //gyro x rate
int16_t gyro_y_rate; //gyro y rate
int16_t gyro_z_rate; //gyro z rate
uint8_t gyro_range; // gyro range
uint8_t sonar_timestamp;// timestep in milliseconds between I2C frames
float ground_distance;// Ground distance in meters*1000. Positive value: distance known. Negative value: Unknown distance
//double alfa_of=0.5;
float vel_of_x;
float vel_of_y;
float pos_of_x;
float pos_of_y;
float tmr_of_rst;
float tmr_of_rst_n;
float target_roll,target_pitch;
AP_Float _alpha_of;
AP_Float _kp_pos_of;
AC_PID&  _pid_optflow;

    
    void init(void);
	void update(void);
    void get_of_angles(float vx_in,float vy_in, int16_t &pitch_out, int16_t &roll_out, float G_Dt);
	void get_sp_vel(int16_t in_x,int16_t in_y, float &v_out_x, float &v_out_y);

	
protected:
//float _filter;
AP_HAL::Semaphore*  _i2c_sem;

};
#endif //AC_AttitudeControl_H

