
#ifndef __PID_BASIC__
#define __PID_BASIC__

#include "GLOBAL/GLOBAL.h"

typedef struct 
{
  float CheckSavePID;
  float Dummy_1;
  float Dummy_2;
  float Dummy_3;
  float KP;
  float KI;
  float KD;
  float KP_YAW;
  float KI_YAW;
  float KD_YAW;  
}PID_VALUE_typedef;

typedef struct
{
  float SetValue = 0.0;
  float ReadValue = 0.0;
  float Error = 0.0;
  float DeltaTime = 0.0;
  float Last_error = 0.0;
  float Integral = 0.0;
  float Derivative = 0.0;
  float Output = 0.0;
  uint32_t LastTimeCalPID=0;
  PID_VALUE_typedef *PIDValue;
}PID_OBJECT_typedef;

void PID_CALCULATOR(PID_OBJECT_typedef *PID_Object);
void PID_CAL_YAW(PID_OBJECT_typedef *PID_Object);
void PID_RESET_DATA(PID_OBJECT_typedef *PID_Object);

#endif /*__PID_BASIC__*/
