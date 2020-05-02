#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <control_motor.h>
#include <process_image.h>
#include <ir_driver.h>
#include <leds.h>

#define VITESSE_MOTOR 500
#define DETECTED 1
#define UNDETECTED 0

//simple PI regulator implementation
void stop_motor()
{
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}

void turn_right()
{
	right_motor_set_speed(-VITESSE_MOTOR);
	left_motor_set_speed(VITESSE_MOTOR);
}

void turn_left()
{
	right_motor_set_speed(VITESSE_MOTOR);
	left_motor_set_speed(-VITESSE_MOTOR);
}

void avancer()
{
	right_motor_set_speed(VITESSE_MOTOR);
	left_motor_set_speed(VITESSE_MOTOR);
}

void reculer()
{
	right_motor_set_speed(-VITESSE_MOTOR);
	left_motor_set_speed(-VITESSE_MOTOR);
}

void escape_obstacle_right(void){
	while(1)
	{
		if(object_check() == DETECTED)
		{
			switch(sensor_feedback())
			        			{
			        			case 0:
			        				turn_left();
			        				break;
			        			case 1:
			        				turn_left();
			        				break;
			        			case 2:
			        				avancer();
			        				break;
			        			case 3:
			        				break;
			        			case 4:
			        				break;
			        			case 5:
			        				break;
			        			case 6:
			        				break;
			        			case 7:
			        				break;
			        			}
		}
		else
		{
			systime_t time;
			time = chVTGetSystemTime();
			avancer();
			chThdSleepUntilWindowed(time, time + MS2ST(1500));
			turn_right();
			time = chVTGetSystemTime();
			chThdSleepUntilWindowed(time, time + MS2ST(500));
			break;
			//avancer encore quelques temps
		}
	}
}


void escape_obstacle_left(void){
	while(1)
	{
		if(object_check() == DETECTED)
		{
			switch(sensor_feedback())
			        			{
			        			case 0:
			        				break;
			        			case 1:
			        				break;
			        			case 2:
			        				break;
			        			case 3:
			        				break;
			        			case 4:
			        				break;
			        			case 5:
			        				avancer();
			        				break;
			        			case 6:
			        				turn_right();
			        				break;
			        			case 7:
			        				turn_right();
			        				break;
			        			}
		}
		else
		{
			systime_t time;
			time = chVTGetSystemTime();
			avancer();
			chThdSleepUntilWindowed(time, time + MS2ST(1500));
			turn_left();
			time = chVTGetSystemTime();
			chThdSleepUntilWindowed(time, time + MS2ST(500));
			break;
			//avancer encore quelques temps
		}
	}
}
static THD_WORKING_AREA(waControlDirection, 256);
static THD_FUNCTION(ControlDirection, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
        time = chVTGetSystemTime();

        if(object_check() == UNDETECTED)
        {
        	set_led( 1 , 0);
        	set_led( 3 , 0);
			if(get_line_detected() == DETECTED)
			{
				//stop_motor(); 1st version
				turn_left();
			}
			else
				avancer();
        }
        else
        {
        	set_led( 1 , 2);
        	set_led( 3 , 2);
        	switch(sensor_feedback())
        			{
        			case 0:
        				escape_obstacle_right();
        				break;
        			case 1:
        				escape_obstacle_right();
        				break;
        			case 2:
        				escape_obstacle_right();
        				break;
        			case 3:
        				break;
        			case 4:
        				break;
        			case 5:
        				escape_obstacle_left();
        				break;
        			case 6:
        				escape_obstacle_left();
        				break;
        			case 7:
        				escape_obstacle_left();
        				break;
        			}
        }
        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}



void control_motor_start(void){
	chThdCreateStatic(waControlDirection, sizeof(waControlDirection), NORMALPRIO, ControlDirection, NULL);
}



