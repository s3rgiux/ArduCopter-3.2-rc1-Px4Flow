// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "AC_OpticalFlowPX4.h"
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;



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

