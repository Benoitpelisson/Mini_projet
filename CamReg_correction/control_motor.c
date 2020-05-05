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
#define LEFT_SENSOR 5
#define RIGHT_SENSOR 2
#define EPUCK_RADIUS 539
#define RADIUS_MARGIN 50
#define FINISH_MARGIN 800
#define ESCAPE_OBJECT 300
#define PASS 380 //arbitrary for now
#define INF_DIST 10

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
	chThdSleepUntilWindowed(time, time + MS2ST(640));//DEFINE
}

void forward_turn_right(void){
	systime_t time;
	uint16_t distance = 0;
	avancer();
	while(distance < (2*EPUCK_RADIUS))
	{
		time = chVTGetSystemTime();
		chThdSleepUntilWindowed(time, time + MS2ST(INF_DIST));//DEFINE
		distance += INF_DIST;
		if(get_line_detected() == DETECTED)
			break;
	}
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
	uint16_t distance = 0;
	avancer();
	while(distance < (2*EPUCK_RADIUS))
	{
		time = chVTGetSystemTime();
		chThdSleepUntilWindowed(time, time + MS2ST(INF_DIST));//DEFINE
		distance += INF_DIST;
		if(get_line_detected() == DETECTED)
			break;
	}
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
		if(object_check() == DETECTED)//if we encounter an object during a u turn, this part "avoids" it
		{
			ninety_turn_right();
			reculer_ms(ESCAPE_OBJECT);
			ninety_turn_left();
		}
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
		if(object_check() == DETECTED)//if we encounter an object during a u turn, this part "avoids" it
		{
			ninety_turn_left();
			reculer_ms(ESCAPE_OBJECT);
			ninety_turn_right();
		}
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
		if(object_check() == DETECTED)//if we encounter an object during a u turn, this part "avoids" it
		{
			ninety_turn_left();
			reculer_ms(ESCAPE_OBJECT);
			ninety_turn_right();
		}
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
		if(object_check() == DETECTED)//if we encounter an object during a u turn, this part "avoids" it
		{
			ninety_turn_right();
			reculer_ms(ESCAPE_OBJECT);
			ninety_turn_left();
		}
	}

}

void escape_obstacle_right(void){
	systime_t time1;
	systime_t time2;
	systime_t timesizeobject; //necessary time to come back to original "line" of progression
	time1 = chVTGetSystemTime();
	ninety_turn_left();
	while((object_check() == DETECTED) && (sensor_feedback() == RIGHT_SENSOR))
	{
		avancer();
	}
	time2 = chVTGetSystemTime();
	timesizeobject = time2-time1;
	forward_turn_right();
	avancer_ms(2000);
	while((object_check() == DETECTED) && (sensor_feedback() == RIGHT_SENSOR))
	{
		avancer();
	}
	forward_turn_right();
	avancer_ms(timesizeobject);
	ninety_turn_left();
}


void escape_obstacle_left(void){
	systime_t time1;
	systime_t time2;
	systime_t timesizeobject;
	time1 = chVTGetSystemTime();
	ninety_turn_right();
	while((object_check() == DETECTED) && (sensor_feedback() == LEFT_SENSOR))
	{
		avancer();
	}
	time2 = chVTGetSystemTime();
	timesizeobject = time2-time1;
	forward_turn_left();
	avancer_ms(2000); //EPUCK_RADIUS+RADIUS_MARGIN
	while((object_check() == DETECTED) && (sensor_feedback() == LEFT_SENSOR))
	{
		avancer();
	}
	forward_turn_left();
	avancer_ms(timesizeobject);
	ninety_turn_right();

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
		if(object_check() == UNDETECTED)
		{
			set_led( 1 , 0);
			set_led( 3 , 0);
			if(get_line_detected() == DETECTED)
			{
				//avancer_ms(FINISH_MARGIN); WARNING, THIS IS TO BE TESTED
				if(!first_collision)
				{
					epuck_scan_direction = analysing_situation();
					first_collision = 1;
				}
				if(epuck_scan_direction == SCAN_LEFT){
					if(object_check() == DETECTED)
						reculer_ms(EPUCK_RADIUS);
					set_led( 1 , 2);
					scan_left();
					set_led( 1 , 0);
				}
				if(epuck_scan_direction == SCAN_RIGHT){
					if(object_check() == DETECTED)
						reculer_ms(EPUCK_RADIUS);
					set_led( 3 , 2);
					scan_right();
					set_led( 3 , 0);
				}
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
			/*switch(sensor_feedback())
					{
					case 0:
						escape_obstacle_right();
						break;
					case 1:
						escape_obstacle_right();
						break;
					case 2:
						break;
					case 3:
						break;
					case 4:
						break;
					case 5:
						break;
					case 6:
						escape_obstacle_left();
						break;
					case 7:
						escape_obstacle_left();
						break;
					}*/
			if(epuck_scan_direction == SCAN_LEFT){
				reculer_ms(EPUCK_RADIUS);
				set_led( 1 , 2);
				scan_left();
				set_led( 1 , 0);
			}
			if(epuck_scan_direction == SCAN_RIGHT){
				reculer_ms(EPUCK_RADIUS);
				set_led( 3 , 2);
				scan_right();
				set_led( 3 , 0);
			}

		}
        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}



void control_motor_start(void){
	chThdCreateStatic(waControlDirection, sizeof(waControlDirection), NORMALPRIO, ControlDirection, NULL);
}



