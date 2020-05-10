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
#define MOTOR_SPEED 500
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
#define NINETY_DEGREES 640
#define WHEEL_CONST 0.13f
#define IR_THRESHOLD 2000
#define DIST 70
#define TOF_MARGIN 5
#define MEASURES 10
#define THRESHOLD 25
#define MARGIN 50
#define IR_SENSOR_COUNT 8

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;


#ifdef __cplusplus
}
#endif

#endif
