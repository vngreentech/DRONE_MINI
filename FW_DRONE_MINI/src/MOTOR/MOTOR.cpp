
#include "MOTOR.h"

#ifdef FIRMWARE_VERSION

static Servo Motor_1;  // ESC 1
static Servo Motor_2;  // ESC 2
static Servo Motor_3;  // ESC 3
static Servo Motor_4;  // ESC 4

void Motor_Stop(void)
{
  Motor_1.write(ZERO);
  Motor_2.write(ZERO);
  Motor_3.write(ZERO);
  Motor_4.write(ZERO);
}

void Motor_Control(MOTOR_typedef MOTOR, uint8_t SpeedValue)
{
  #ifndef STOP_FOR_TEST
  if(MOTOR==MOTOR_1)
  {
    if(SpeedValue<=ZERO) Motor_1.write(ZERO);
    else if(SpeedValue>=MOTOR_1_MAX) Motor_1.write(MOTOR_1_MAX);
    else Motor_1.write(SpeedValue);
  }
  else if(MOTOR==MOTOR_2)
  {
    if(SpeedValue<=ZERO) Motor_2.write(ZERO);
    else if(SpeedValue>=MOTOR_2_MAX) Motor_2.write(MOTOR_2_MAX);
    else Motor_2.write(SpeedValue);
  } 
  else if(MOTOR==MOTOR_3)
  {
    if(SpeedValue<=ZERO) Motor_3.write(ZERO);
    else if(SpeedValue>=MOTOR_3_MAX) Motor_3.write(MOTOR_3_MAX);
    else Motor_3.write(SpeedValue);
  } 
  else if(MOTOR==MOTOR_4)
  {
    if(SpeedValue<=ZERO) Motor_4.write(ZERO);
    else if(SpeedValue>=MOTOR_4_MAX) Motor_4.write(MOTOR_4_MAX);
    else Motor_4.write(SpeedValue);
  }
  else if(MOTOR==ALL_MOTOR)
  {
    if(SpeedValue<=ZERO)
    {
      Motor_1.write(ZERO);
      Motor_2.write(ZERO);
      Motor_3.write(ZERO);
      Motor_4.write(ZERO);
    }
    else if(SpeedValue>=SERVO_MAX)
    {
      Motor_1.write(SERVO_MAX);
      Motor_2.write(SERVO_MAX);
      Motor_3.write(SERVO_MAX);
      Motor_4.write(SERVO_MAX);
    }
    else 
    {
      Motor_1.write(SpeedValue);
      Motor_2.write(SpeedValue);
      Motor_3.write(SpeedValue);
      Motor_4.write(SpeedValue);
    }
  }  
  else Motor_Stop();
  #endif /* STOP_FOR_TEST */

  #ifdef STOP_FOR_TEST
  Motor_Stop();
  #endif /* STOP_FOR_TEST */
}

void Motor_Init(void)
{
  Motor_1.attach(ESC_1_PIN,PPM_MIN,PPM_MAX);
  Motor_2.attach(ESC_2_PIN,PPM_MIN,PPM_MAX);
  Motor_3.attach(ESC_3_PIN,PPM_MIN,PPM_MAX);
  Motor_4.attach(ESC_4_PIN,PPM_MIN,PPM_MAX);  
  Motor_Stop();    
}
#endif /* FIRMWARE_VERSION */

