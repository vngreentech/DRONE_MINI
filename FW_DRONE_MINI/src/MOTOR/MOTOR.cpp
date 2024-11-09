
#include "MOTOR.h"

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

void Motor_Control(uint8_t SpeedValue)
{
  if(SpeedValue<=SERVO_MIN)
  {
    Motor_1.write(SERVO_MIN);
    Motor_2.write(SERVO_MIN);
    Motor_3.write(SERVO_MIN);
    Motor_4.write(SERVO_MIN);
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

void Motor_Init(void)
{
  Motor_1.attach(ESC_1_PIN,PPM_MIN,PPM_MAX);
  Motor_2.attach(ESC_2_PIN,PPM_MIN,PPM_MAX);
  Motor_3.attach(ESC_3_PIN,PPM_MIN,PPM_MAX);
  Motor_4.attach(ESC_4_PIN,PPM_MIN,PPM_MAX);  
  Motor_Stop();    
}
