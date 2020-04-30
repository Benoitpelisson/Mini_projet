#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <sensors/proximity.h>

#include <ir_driver.h>

#define IR_THRESHOLD 3000

static uint8_t object = 0;

static THD_WORKING_AREA(wair_analyse, 1024);
static THD_FUNCTION(ir_analyse, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;


    while(1)
    {
    	object = 0;
    	time = chVTGetSystemTime();
		for(uint8_t i = 0 ; i < 8 ; i++)
		{
			//uint16_t value = get_prox(0);
			//chprintf((BaseSequentialStream *)&SD3, "value = %d \n", value);
			if(get_prox(i) > IR_THRESHOLD)
			{
				object = 1;
			}
		}
		chThdSleepUntilWindowed(time, time + MS2ST(10));
    }


}

uint8_t object_check (void)
{
	return object;
}

void ir_analyse_start(void){
	chThdCreateStatic(wair_analyse, sizeof(wair_analyse), NORMALPRIO, ir_analyse, NULL);
}
