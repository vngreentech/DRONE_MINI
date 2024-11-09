
#ifndef __RECEIVER__
#define __RECEIVER__

#include "GLOBAL/GLOBAL.h"

typedef struct 
{
  volatile uint8_t CH1;
  volatile uint8_t CH2;
  volatile uint8_t CH3;
  volatile uint8_t CH4;
}Channel_Typedef;

void Receiver_Init(void);
Channel_Typedef Receiver_Read_Value(void);

#endif /*__RECEIVER__*/

