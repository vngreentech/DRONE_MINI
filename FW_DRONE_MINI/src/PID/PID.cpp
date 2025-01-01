
#include "PID.h"

#ifdef FIRMWARE_VERSION
void PID_CALCULATOR(PID_OBJECT_typedef *PID_Object)
{
  PID_Object->DeltaTime = (float)(MILLIS - PID_Object->LastTimeCalPID) / 1000.0;

  PID_Object->Error = (PID_Object->SetValue) - (PID_Object->ReadValue);
  PID_Object->Integral += (PID_Object->Error * PID_Object->DeltaTime);
  PID_Object->Integral = constrain(PID_Object->Integral, -500, 500);
  PID_Object->Derivative = (PID_Object->Error - PID_Object->Last_error) / PID_Object->DeltaTime;

  PID_Object->Output =  (PID_Object->PIDValue->KP * PID_Object->Error)\
                        + (PID_Object->PIDValue->KI * PID_Object->Integral)\
                        + (PID_Object->PIDValue->KD * PID_Object->Derivative);

  PID_Object->Last_error = PID_Object->Error;
}

void PID_CAL_YAW(PID_OBJECT_typedef *PID_Object)
{
  PID_Object->DeltaTime = (float)(MILLIS - PID_Object->LastTimeCalPID) / 1000.0;

  PID_Object->Error = (PID_Object->SetValue) - (PID_Object->ReadValue);
  
  if (PID_Object->Error > 180) PID_Object->Error -= 360;
  else if (PID_Object->Error < -180) PID_Object->Error += 360;

  PID_Object->Integral += (PID_Object->Error * PID_Object->DeltaTime);
  PID_Object->Integral = constrain(PID_Object->Integral, -500, 500);
  PID_Object->Derivative = (PID_Object->Error - PID_Object->Last_error) / PID_Object->DeltaTime;

  PID_Object->Output =  (PID_Object->PIDValue->KP_YAW * PID_Object->Error)\
                        + (PID_Object->PIDValue->KI_YAW * PID_Object->Integral)\
                        + (PID_Object->PIDValue->KD_YAW * PID_Object->Derivative);

  PID_Object->Last_error = PID_Object->Error;
}

void PID_RESET_DATA(PID_OBJECT_typedef *PID_Object)
{
  PID_Object->Error=0;
  PID_Object->Integral=0;
  PID_Object->Derivative=0;
  PID_Object->Last_error=0;
  PID_Object->DeltaTime=0;
}
#endif /* FIRMWARE_VERSION */

