
#ifndef __IMU__
#define __IMU__

#include "GLOBAL/GLOBAL.h"

typedef struct  
{
  float Yaw;
  float Pitch;
  float Roll;
  bool Error;
}IMU_DATA_TYPEDEF;

IMU_DATA_TYPEDEF GetIMUValue(void);
void setupMPU(void);
float IMU_Cal_YAW_Value(float *YawValue); 

#endif /*__IMU__*/
