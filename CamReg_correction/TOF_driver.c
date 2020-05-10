#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <sensors/VL53L0X/VL53L0X.h>

#include <TOF_driver.h>


static uint8_t object = 0;
static uint8_t i = 0;
static uint32_t count = 0;

static THD_WORKING_AREA(watof_analyse, 256);
static THD_FUNCTION(tof_analyse, arg)
//this thread checks the values of the TOF sensor
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    systime_t time;

    while(1)
    {
    	object = 0;
    	uint32_t measure = VL53L0X_get_dist_mm();
    	if(measure < DIST)
    	{
    		count++;
    	}
    	if(count >= TOF_MARGIN)//if TOF_MARGIN or more measures are inferior to DIST, we consider that there is an obtsacle
    	{
    		object = 1;
    	}
		i++;
		if(i == (MEASURES-1))
		{
			i=0;
			count=0;
		}
		time = chVTGetSystemTime();
		chThdSleepUntilWindowed(time, time + MS2ST(100));//thread frequency 10Hz
    }


}

uint32_t get_distance(void)
//gives the distance to the user
//return: distance in mm
{
	uint32_t distance_mm = VL53L0X_get_dist_mm();
	return distance_mm;
}

uint8_t TOF_check (void)
//says if an object is detected
//return: object
{
	return object;
}


void tof_analyse_start(void)
//starts the TOF analyse thread
{
	chThdCreateStatic(watof_analyse, sizeof(watof_analyse), NORMALPRIO, tof_analyse, NULL);
}
