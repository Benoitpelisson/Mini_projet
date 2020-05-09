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
#include <TOF_driver.h>

/*#define VITESSE_MOTOR 500
#define DETECTED 1
#define UNDETECTED 0
#define	SCAN_LEFT 0
#define	SCAN_RIGHT 1
#define EPUCK_RADIUS 539
#define FINISH_MARGIN 800
#define ESCAPE_OBJECT 300
#define INF_DIST 10
#define LED_ON 2
#define LED_OFF 0
#define REAR_LEFT 4
#define REAR_RIGHT 3
#define NINETY_DEGREES 640
#define WHEEL_CONST 0.13*/


static bool comeback = 0;
static uint8_t epuck_scan_direction;

uint32_t distance_to_ms(uint32_t distance)
{
	uint32_t ms = distance/(VITESSE_MOTOR * WHEEL_CONST);
	return ms;
}

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

void stop(int stop_time){
	systime_t time;
	time = chVTGetSystemTime();
	stop_motor();
	chThdSleepUntilWindowed(time, time + MS2ST(stop_time));

}

void avancer_ms(uint16_t command)
{
	systime_t time;
	avancer();
	time = chVTGetSystemTime();
	chThdSleepUntilWindowed(time, time + MS2ST(command));
}

void reculer_ms(uint16_t command)
{
	systime_t time;
	reculer();
	time = chVTGetSystemTime();
	chThdSleepUntilWindowed(time, time + MS2ST(command));
}

void ninety_turn_right(void){
	systime_t time;
	turn_right();
	time = chVTGetSystemTime();
	chThdSleepUntilWindowed(time, time + MS2ST(NINETY_DEGREES));
}

void forward_turn_right(void){
	systime_t time;
	uint16_t distance = 0;
	avancer();
	while(distance < (2*EPUCK_RADIUS))//this loop is here to check if we encounter any line when the robot is doing a u-turn
	{
		time = chVTGetSystemTime();
		chThdSleepUntilWindowed(time, time + MS2ST(INF_DIST));
		distance += INF_DIST;
		if(get_line_detected() == DETECTED)//this condition means we have reached a corner, we toggle the scan direction
		{
			epuck_scan_direction = (1-epuck_scan_direction);
			break;
		}
		if(object_check() == DETECTED)//when doing a u-turn, f the ir sensors sense a collision we back up before finishing the u-turn
		{
			reculer_ms(ESCAPE_OBJECT);
			break;
		}
	}
	turn_right();
	time = chVTGetSystemTime();
	chThdSleepUntilWindowed(time, time + MS2ST(NINETY_DEGREES));
}

void ninety_turn_left(void){
	systime_t time;
	turn_left();
	time = chVTGetSystemTime();
	chThdSleepUntilWindowed(time, time + MS2ST(NINETY_DEGREES));
}

void forward_turn_left(void){
	systime_t time;
	uint16_t distance = 0;
	avancer();
	while(distance < (2*EPUCK_RADIUS))//this loop is here to check if we encounter any line when the robot is doing a u-turn
	{
		time = chVTGetSystemTime();
		chThdSleepUntilWindowed(time, time + MS2ST(INF_DIST));
		distance += INF_DIST;
		if(get_line_detected() == DETECTED)//this condition means we have reached a corner, we toggle the scan direction
		{
			epuck_scan_direction = (1-epuck_scan_direction);
			break;
		}
		if(object_check() == DETECTED)
		{
			reculer_ms(ESCAPE_OBJECT);
			break;
		}
	}
	turn_left();
	time = chVTGetSystemTime();
	chThdSleepUntilWindowed(time, time + MS2ST(NINETY_DEGREES));
}

void u_turn(void){
	ninety_turn_left();
	ninety_turn_left();
}

//this function checks for possibilities where the robot can go when it sees a line for the first time
uint8_t analysing_situation(void){
	ninety_turn_left();
	bool can_go_left = (1-get_line_detected()); //if no line, can go left
	stop(100);
	u_turn();
	bool can_go_right = (1-get_line_detected());
	stop(100);
	ninety_turn_left();
	stop(100);
	if(can_go_left)
		return SCAN_LEFT;
	if(can_go_right)
		return SCAN_RIGHT;
	return SCAN_LEFT;
}

void scan_left(void){
	if(!comeback) //if the robot is going, we make it do a left u turn
	{
		ninety_turn_left();
		comeback = 1;
		forward_turn_left();
	}
	else //if the robot is coming back, we make it do a right u turn
	{
		ninety_turn_right();
		comeback = 0;
		forward_turn_right();
	}

}

void scan_right(void){
	if(!comeback) //if the robot is going, we make it do a right u turn
	{
		ninety_turn_right();
		comeback = 1;
		forward_turn_right();
	}
	else //if the robot is coming back, we make it do a left u turn
	{
		ninety_turn_left();
		comeback = 0;
		forward_turn_left();
	}

}

static THD_WORKING_AREA(waControlDirection, 256);
static THD_FUNCTION(ControlDirection, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    bool first_collision = 0;

    time = chVTGetSystemTime();
    chThdSleepUntilWindowed(time, time + MS2ST(10000));//waits before starting in order to have the right setup


    while(1){
        time = chVTGetSystemTime();
		if(TOF_check() == UNDETECTED)
		{
			if(get_line_detected() == DETECTED)
			{
				//avancer_ms(FINISH_MARGIN);
				if(!first_collision)
				{
					epuck_scan_direction = analysing_situation();
					first_collision = 1;
				}
				switch(epuck_scan_direction)
				{
					case SCAN_LEFT:
					{
						scan_left();
						break;
					}
					case SCAN_RIGHT:
					{
						scan_right();
						break;
					}
				}
			}
			else if(object_check() == DETECTED)//if tof didnt detect the obstacle, i.e the obstacle isnt directly in front of epuck
			{
				reculer_ms(EPUCK_RADIUS);
				switch(epuck_scan_direction)
				{
					case SCAN_LEFT:
					{
						scan_left();
						break;
					}
					case SCAN_RIGHT:
					{
						scan_right();
						break;
					}
				}
			}
			else
			{
					avancer();
			}
		}
		else
		{
			set_body_led(LED_ON);
			avancer_ms(distance_to_ms(get_distance()));
			if(!first_collision)
			{
				epuck_scan_direction = analysing_situation();
				first_collision = 1;
			}
			reculer_ms(ESCAPE_OBJECT);
			if(epuck_scan_direction == SCAN_LEFT)
			{
				set_body_led(LED_OFF);
				scan_left();
			}
			if(epuck_scan_direction == SCAN_RIGHT)
			{
				set_body_led(LED_OFF);
				scan_right();
			}
		}
        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}



void control_motor_start(void){
	chThdCreateStatic(waControlDirection, sizeof(waControlDirection), NORMALPRIO, ControlDirection, NULL);
}



