#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <sensors/VL53L0X/VL53L0X.h>

#include <TOF_driver.h>

#define IR_THRESHOLD 2000 //threshold is fixed in order for the robot to try and avoid touching objects, previously 3000
#define DIST 70

static uint8_t object = 0;
static uint8_t sensor_number = 0;

static THD_WORKING_AREA(watof_analyse, 1024);
static THD_FUNCTION(tof_analyse, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;


    while(1)
    {
    	object = 0;
    	time = chVTGetSystemTime();
    	if(VL53L0X_get_dist_mm() < DIST)
    	{
    		object = 1;
    	}
    	chThdSleepUntilWindowed(time, time + MS2ST(10));
    }


}

uint8_t TOF_check (void)
{
	return object;
}


void tof_analyse_start(void){
	chThdCreateStatic(watof_analyse, sizeof(watof_analyse), NORMALPRIO, tof_analyse, NULL);
}
