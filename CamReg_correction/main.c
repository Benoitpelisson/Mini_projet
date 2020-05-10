#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <sensors/proximity.h>
#include <chprintf.h>
#include <sensors/VL53L0X/VL53L0X.h>

#include <ir_driver.h>
#include <control_motor.h>
#include <process_image.h>
#include <TOF_driver.h>


messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


int main(void)
{

    halInit();
    chSysInit();
    mpu_init();
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //initialization section
    //starts the camera
    dcmi_start();
	po8030_start();
	//inits the motors
	motors_init();
	//starts the IR sensors
	proximity_start();
	//starts the TOF
	VL53L0X_start();
	//stars the threads to control the robot
	control_motor_start();
	process_image_start();
	ir_analyse_start();
	tof_analyse_start();


    /* Infinite loop. */
    while (1) {
    	//waits 1 second
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
