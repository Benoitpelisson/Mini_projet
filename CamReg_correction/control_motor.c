#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <control_motor.h>
#include <process_image.h>

#define VITESSE_MOTOR 500

//simple PI regulator implementation
void stop_motor()
{
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}

void turn_right()
{
	right_motor_set_speed(VITESSE_MOTOR);
	left_motor_set_speed(-VITESSE_MOTOR);
}

void turn_left()
{
	right_motor_set_speed(-VITESSE_MOTOR);
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
	left_motor_set_speed(VITESSE_MOTOR);
}

static THD_WORKING_AREA(waControlDirection, 256);
static THD_FUNCTION(ControlDirection, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
        time = chVTGetSystemTime();


        if(get_line_detected() == 1)
        {
        	//stop_motor(); 1st version
        	turn_right();
        }
        else
        	avancer();


        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void control_motor_start(void){
	chThdCreateStatic(waControlDirection, sizeof(waControlDirection), NORMALPRIO, ControlDirection, NULL);
}



