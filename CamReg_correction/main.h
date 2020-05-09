#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2 
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			10.0f
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define KP						800.0f
#define KI 						3.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)

#define VITESSE_MOTOR 500
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
#define WHEEL_CONST 0.13
#define IR_THRESHOLD 2000
#define DIST 70
#define TOF_MARGIN 5
#define MEASURES 10
#define THRESHOLD 25		//previously 40
#define MARGIN 50

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
