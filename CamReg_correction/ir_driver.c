#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <sensors/proximity.h>

#include <ir_driver.h>

static uint8_t object = 0;
static uint8_t sensor_number = 0;

static THD_WORKING_AREA(wair_analyse, 256);
static THD_FUNCTION(ir_analyse, arg)
//this thread checks the values of the IR sensors
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    systime_t time;

    while(1)
    {
    	object = 0;
    	time = chVTGetSystemTime();
		for(uint8_t sensor_id = 0 ; sensor_id < IR_SENSOR_COUNT ; sensor_id++)//gets values for each one of the 8 sensors
		{
			if(get_prox(sensor_id) > IR_THRESHOLD)
			{
				object = 1;
				sensor_number = sensor_id;
			}
		}
		chThdSleepUntilWindowed(time, time + MS2ST(10));//thread frequency 100Hz
    }


}

uint8_t IR_check (void)
//says if an object has been detected
//return: object
{
	return object;
}

uint8_t sensor_feedback(void)
//says which sensor detected the object
//return: sensor_number
{
	return sensor_number;
}

void ir_analyse_start(void)
//starts the IR analyse thread
{
	chThdCreateStatic(wair_analyse, sizeof(wair_analyse), NORMALPRIO, ir_analyse, NULL);
}
