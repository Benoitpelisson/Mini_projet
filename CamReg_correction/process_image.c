#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>
#include <leds.h>
#include <process_image.h>

static uint8_t line_detected = 0;


//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */
uint8_t extract_info_line(uint8_t *buffer)
//this function analyses the image
//param: pointer on the image *buffer
//return: line_in_front
{

	uint8_t line_in_front = 0;
	uint16_t count = 0;

	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++)
	{
		if((buffer[i] < THRESHOLD) && ((i >= 200) && (i <= 440))) //checks if a pixel in the center of the camera has a value inferior to THRESHOLD
		{
			count++;
		}
	}
	if(count >= MARGIN)//if enough pixels have a weak intensity, we consider that there is a line in front
	{
		line_in_front = 1;
	}
	return line_in_front;
}
static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg)
//comes from tp4 and is used in the project
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 475+476 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 475, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1); //previously 475
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg)
//comes from tp4 and is used in the project
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}

		//Look if a black line is in front of the robot
		line_detected = extract_info_line(image);

    }
}

uint8_t get_line_detected(void)
//tells if a line is detected
//return: global variable line_detected
{
	return line_detected;
}

void process_image_start(void)
//starts the two threads necessary to analyse an image
{
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
