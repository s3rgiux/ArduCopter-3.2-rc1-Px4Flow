/*
 *       Example of PX4Flow Library
 *       
 */

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AC_PID.h>
#include <AC_HELI_PID.h>
#include <AC_OpticalFlowPX4.h>
#include <AC_OpticalFlowPX4_Pixhawk.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// default PID values
#define TEST_P 0.0017
#define TEST_I 0.0
#define TEST_D 0.0
#define TEST_IMAX 100

// setup function
AC_PID pid(TEST_P, TEST_I, TEST_D, TEST_IMAX * 100);
AC_OpticalFlowPX4 ofPX4(pid);
int16_t pitch_out,roll_out;
Vector3f vel_target_rot;
void setup()
{
    hal.console->println("Optical Flow Px4Flow library test");
    ofPX4.init();
    hal.scheduler->delay(1000);
}

// main loop
void loop()
{
         ofPX4.update();//update the data from i2c
		 ofPX4.get_sp_vel(g.rc_2.control_in,g.rc_1.control_in,vel_target_rot.x,vel_target_rot.y);//desired velocities fron radio
         ofPX4.get_of_angles(vel_target_rot.x,vel_target_rot.y,pitch_out,roll_out,0.05);//filter and obtains the desired angles las parameter is dt
		 
         hal.console->printf("%f,%f,%d,%d",ofPX4.target_roll,ofPX4.target_pitch,roll_out,pitch_out);//
         hal.console->println();
        hal.scheduler->delay(50);
}

AP_HAL_MAIN();
