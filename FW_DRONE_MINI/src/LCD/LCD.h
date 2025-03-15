
#ifndef __LCD__
#define __LCD__

#include "GLOBAL/GLOBAL.h"

void LCD_Init(void);
void LCD_Menu_IMU(float PITCH, float ROLL, float YAW);
void LCD_Menu_PID(PID_typedef PID_Select, float PID_Value);
void LCD_Menu_RESET(void);
void LCD_Menu_SAVE(void);
void LCD_Menu_SET_DEFAULT_PID_VALUE(uint8_t Step);

#endif /*__LCD__*/
