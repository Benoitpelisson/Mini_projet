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
#define	SCAN_LEFT 0
#define	SCAN_RIGHT 1

static bool comeback = 0;
static uint8_t epuck_scan_direction;


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

void ninety_turn_right(void){
	systime_t time;
	turn_right();
	time = chVTGetSystemTime();
	chThdSleepUntilWindowed(time, time + MS2ST(640));//DEFINE
}

void forward_turn_right(void){
	systime_t time;
	avancer();
	time = chVTGetSystemTime();
	chThdSleepUntilWindowed(time, time + MS2ST(1000));//DEFINE
	turn_right();
	time = chVTGetSystemTime();
	chThdSleepUntilWindowed(time, time + MS2ST(640));//DEFINE
}

void ninety_turn_left(void){
	systime_t time;
	turn_left();
	time = chVTGetSystemTime();
	chThdSleepUntilWindowed(time, time + MS2ST(640));//DEFINE
}

void forward_turn_left(void){
	systime_t time;
	avancer();
	time = chVTGetSystemTime();
	chThdSleepUntilWindowed(time, time + MS2ST(1000));//DEFINE
	turn_left();
	time = chVTGetSystemTime();
	chThdSleepUntilWindowed(time, time + MS2ST(640));//DEFINE
}

void u_turn(void){
	ninety_turn_left();
	ninety_turn_left();
}

void stop(int stop_time){
	systime_t time;
	time = chVTGetSystemTime();
	stop_motor();
	chThdSleepUntilWindowed(time, time + MS2ST(stop_time));

}
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
}

void scan_left(void){
	if(!comeback) //if the robot is going, we make it do a left u turn
	{
		ninety_turn_left();
		comeback = 1;
		if(get_line_detected() == DETECTED)//this condition in functions scan left and scan right is here to make the robot turn back when it reaches the end of the black rectangle
		{
			epuck_scan_direction = SCAN_RIGHT;
			ninety_turn_left();
			return;
		}
		forward_turn_left();
	}
	else //if the robot is coming back, we make it do a right u turn
	{
		ninety_turn_right();
		comeback = 0;
		if(get_line_detected() == DETECTED)
		{
			epuck_scan_direction = SCAN_RIGHT;
			ninety_turn_right();
			return;
		}
		forward_turn_right();
	}

}

void scan_right(void){
	if(!comeback) //if the robot is going, we make it do a right u turn
	{
		ninety_turn_right();
		comeback = 1;
		if(get_line_detected() == DETECTED)
		{
			epuck_scan_direction = SCAN_LEFT;
			ninety_turn_right();
			return;
		}
		forward_turn_right();
	}
	else //if the robot is coming back, we make it do a left u turn
	{
		ninety_turn_left();
		comeback = 0;
		if(get_line_detected() == DETECTED)
		{
			epuck_scan_direction = SCAN_LEFT;
			ninety_turn_left();
			return;
		}
		forward_turn_left();
	}

}

void escape_obstacle_right(void){
	while(1)
	{
		if(object_check() == DETECTED)
		{
			systime_t time;
			switch(sensor_feedback())
			        			{
			        			case 0:
			        				time = chVTGetSystemTime();
			        				turn_left();
			        				chThdSleepUntilWindowed(time, time + MS2ST(500));
			        				break;
			        			case 1:
			        				time = chVTGetSystemTime();
			        				turn_left();
			        				chThdSleepUntilWindowed(time, time + MS2ST(332));
			        				break;
			        			case 2:
			        				time = chVTGetSystemTime();
			        				avancer();
			        				chThdSleepUntilWindowed(time, time + MS2ST(166));
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
			break;
		}
	}
}


void escape_obstacle_left(void){
	while(1)
	{
		if(object_check() == DETECTED)
		{
			systime_t time;
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
			        				time = chVTGetSystemTime();
			        				avancer();
			        				chThdSleepUntilWindowed(time, time + MS2ST(166));
			        				break;
			        			case 6:
			        				time = chVTGetSystemTime();
			        				turn_right();
			        				chThdSleepUntilWindowed(time, time + MS2ST(332));
			        				break;
			        			case 7:
			        				time = chVTGetSystemTime();
			        				turn_right();
			        				chThdSleepUntilWindowed(time, time + MS2ST(500));
			        				break;
			        			}
		}
		else
		{
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
    //bool turned_left = 0;
    //bool turned_right = 0;
    bool first_collision = 0;

    time = chVTGetSystemTime();
    chThdSleepUntilWindowed(time, time + MS2ST(10000));//waits before starting in order to have the right setup


    while(1){
        time = chVTGetSystemTime();
		if(object_check() == UNDETECTED)
		{
			set_led( 1 , 0);
			set_led( 3 , 0);
			if(get_line_detected() == DETECTED)
			{
				if(!first_collision)
				{
					epuck_scan_direction = analysing_situation();
					first_collision = 1;
				}
				if(epuck_scan_direction == SCAN_LEFT){
					set_led( 1 , 2);
					scan_left();
					set_led( 1 , 0);
				}
				if(epuck_scan_direction == SCAN_RIGHT){
					set_led( 3 , 2);
					scan_right();
					set_led( 3 , 0);
				}
				/*if(comeback == 0) //if the robot is going, we make it do a left u turn
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
				}*/
			}
			else
			{
					avancer();
			}
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



