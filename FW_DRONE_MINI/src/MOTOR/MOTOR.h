
#ifndef __MOTOR__
#define __MOTOR__

#include "GLOBAL/GLOBAL.h"

typedef enum 
{
  ALL_MOTOR,
  MOTOR_1,
  MOTOR_2,
  MOTOR_3,
  MOTOR_4
}MOTOR_typedef;

typedef struct 
{
  uint8_t Motor_1;
  uint8_t Motor_2;
  uint8_t Motor_3;
  uint8_t Motor_4;
}MOTOR_Speed_typedef;

void Motor_Init(void);
void Motor_Stop(void);
void Motor_Control(MOTOR_typedef MOTOR, uint8_t SpeedValue);

#endif /*__MOTOR__*/
