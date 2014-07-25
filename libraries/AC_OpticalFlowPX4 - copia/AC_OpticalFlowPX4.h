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

class AC_OpticalFlowPX4 {
public:
	//AC_OpticalFlow(int_t adress) :
	
		//_adress(adress)
		//AC_OpticalFlow(float filter) :
		//_filter(filter)
		AC_OpticalFlowPX4()	{
			//inits
		}
		
		
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
//double vel_of_x,vel_of_y;
//double pos_of_x,pos_of_y;
    
    void init(void);
	void update(void);
    
	
	
protected:
//float _filter;
AP_HAL::Semaphore*  _i2c_sem;

};
#endif //AC_AttitudeControl_H
