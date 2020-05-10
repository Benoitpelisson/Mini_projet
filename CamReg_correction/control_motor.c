#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <stdbool.h>

#include <main.h>
#include <motors.h>
#include <control_motor.h>
#include <process_image.h>
#include <ir_driver.h>
#include <leds.h>
#include <TOF_driver.h>


static bool comeback = 0;
static uint8_t epuck_scan_direction;

uint32_t distance_to_ms(uint32_t distance)
//converts a distance to ms
//param: distance in mm
//return: distance in ms
{
	uint32_t ms = distance/(MOTOR_SPEED * WHEEL_CONST);
	return ms;
}

void stop_motor()
//sets both motors speed to 0
{
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}

void turn_right()
//sets motors speed so the robot turns to the right
{
	right_motor_set_speed(-MOTOR_SPEED);
	left_motor_set_speed(MOTOR_SPEED);
}

void turn_left()
//sets motors speed so the robot turns to the left
{
	right_motor_set_speed(MOTOR_SPEED);
	left_motor_set_speed(-MOTOR_SPEED);
}

void move_forward()
//sets motors speed so the robot goes forward
{
	right_motor_set_speed(MOTOR_SPEED);
	left_motor_set_speed(MOTOR_SPEED);
}

void move_backward()
//sets motors speed so the robot goes backward
{
	right_motor_set_speed(-MOTOR_SPEED);
	left_motor_set_speed(-MOTOR_SPEED);
}

void stop(int stop_time)
//stops the motors for stop_time milliseconds
//param: stop_time in ms
{
	systime_t time;
	time = chVTGetSystemTime();
	stop_motor();
	chThdSleepUntilWindowed(time, time + MS2ST(stop_time));

}

void move_forward_ms(uint16_t command)
//the robot goes forward for command ms
//param: command in ms
{
	systime_t time;
	move_forward();
	time = chVTGetSystemTime();
	chThdSleepUntilWindowed(time, time + MS2ST(command));
}

void move_backward_ms(uint16_t command)
//the robot goes backward for command ms
//param: command in ms
{
	systime_t time;
	move_backward();
	time = chVTGetSystemTime();
	chThdSleepUntilWindowed(time, time + MS2ST(command));
}

void ninety_turn_right(void)
//the robot does a ninety degrees right turn
{
	systime_t time;
	turn_right();
	time = chVTGetSystemTime();
	chThdSleepUntilWindowed(time, time + MS2ST(NINETY_DEGREES));
}

void forward_turn_right(void)
//the robot goes forward and does a ninety degrees right turn
{
	systime_t time;
	uint16_t distance = 0;
	move_forward();
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
		if(IR_check() == DETECTED)//when doing a u-turn, f the ir sensors sense a collision we back up before finishing the u-turn
		{
			move_backward_ms(ESCAPE_OBJECT);
			break;
		}
	}
	turn_right();
	time = chVTGetSystemTime();
	chThdSleepUntilWindowed(time, time + MS2ST(NINETY_DEGREES));
}

void ninety_turn_left(void)
//the robot does a ninety degrees left turn
{
	systime_t time;
	turn_left();
	time = chVTGetSystemTime();
	chThdSleepUntilWindowed(time, time + MS2ST(NINETY_DEGREES));
}

void forward_turn_left(void)
//the robot goes forward and does a ninety degrees left turn
{
	systime_t time;
	uint16_t distance = 0;
	move_forward();
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
		if(IR_check() == DETECTED)
		{
			move_backward_ms(ESCAPE_OBJECT);
			break;
		}
	}
	turn_left();
	time = chVTGetSystemTime();
	chThdSleepUntilWindowed(time, time + MS2ST(NINETY_DEGREES));
}

void u_turn(void)
//the robot does a 180 degrees left turn
{
	ninety_turn_left();
	ninety_turn_left();
}


uint8_t analysing_situation(void)
//this function checks for possibilities where the robot can go when it sees a line for the first time
//return: scan direction of the robot
{
	bool can_go_left = 0;
	bool can_go_right = 0;

	ninety_turn_left();
	can_go_left = (1-get_line_detected()); //if no line, can go left
	stop(100);
	u_turn();
	can_go_right = (1-get_line_detected());
	stop(100);
	ninety_turn_left();
	stop(100);
	if(can_go_left)
		return SCAN_LEFT;
	if(can_go_right)
		return SCAN_RIGHT;
	return SCAN_LEFT;//returns default scan direction
}

void scan_left(void)
//in left direction state, performs necessary steps to cover ground from right to left
{
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

void scan_right(void)
//in right direction state, performs necessary steps to cover ground from right to left
{
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
static THD_FUNCTION(ControlDirection, arg)
//main thread that manages the robot according to the constraints
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    bool first_collision = 0;

    time = chVTGetSystemTime();
    chThdSleepUntilWindowed(time, time + MS2ST(5000));//waits before starting in order to have the right setup


    while(1){
        time = chVTGetSystemTime();
		if(TOF_check() == UNDETECTED)//if no obstacle is detected with TOF
		{
			if(get_line_detected() == DETECTED)//if a line is detected with camera
			{
				if(!first_collision)//if it is the first collision with a line
				{
					epuck_scan_direction = analysing_situation();
					first_collision = 1;
				}
				switch(epuck_scan_direction)//line detected!: gives instructions to the robot
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
			else if(IR_check() == DETECTED)//if TOF didnt detect the obstacle, IR sensors check for an object
			{
				move_backward_ms(EPUCK_RADIUS);
				switch(epuck_scan_direction)//object detected!: gives instructions to the robot
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
			else//if nothing is detected, the robot moves forward
			{
					move_forward();
			}
		}
		else//if TOF detects an object
		{
			set_body_led(LED_ON);
			move_forward_ms(distance_to_ms(get_distance()));
			if(!first_collision)//if it is the first collision with a obstacle
			{
				epuck_scan_direction = analysing_situation();
				first_collision = 1;
			}
			move_backward_ms(ESCAPE_OBJECT);
			switch(epuck_scan_direction)//object detected!: gives instructions to the robot
			{
				case SCAN_LEFT:
				{
					set_body_led(LED_OFF);
					scan_left();
					break;
				}
				case SCAN_RIGHT:
				{
					set_body_led(LED_OFF);
					scan_right();
					break;
				}
			}
		}
        chThdSleepUntilWindowed(time, time + MS2ST(10));//thread frequency is 100Hz
    }
}



void control_motor_start(void)
//starts the ControlDirection thread
{
	chThdCreateStatic(waControlDirection, sizeof(waControlDirection), NORMALPRIO, ControlDirection, NULL);
}



