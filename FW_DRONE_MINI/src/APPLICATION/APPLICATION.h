
#ifndef __APPLICATION__
#define __APPLICATION__

#include "GLOBAL/GLOBAL.h"
#include "IMU/IMU.h"
#include "MOTOR/MOTOR.h"
#include "LCD/LCD.h"
#include "RECEIVER/RECEIVER.h"
#include "SENSOR/SENSOR.h"
#include "PID/PID.h"

#define TimeCalPID  (float(5.0)) //ms

extern void APP_INIT(void);
extern void APP_MAIN(void);

#endif /*__APPLICATION__*/
