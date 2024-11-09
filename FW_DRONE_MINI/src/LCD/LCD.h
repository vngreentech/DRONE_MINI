
#ifndef __LCD__
#define __LCD__

#include "GLOBAL/GLOBAL.h"

void LCD_Init(void);
void LCD_Menu_Main(void);
void LCD_Menu_IMU(float PITCH, float ROLL, float YAW);
void LCD_Menu_PID(PID_typedef PID_Select, float PID_Value);

#endif /*__LCD__*/
