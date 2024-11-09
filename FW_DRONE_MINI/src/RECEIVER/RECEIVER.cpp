
#include "RECEIVER.h"

static volatile uint32_t CH1_Start_Time = 0, CH2_Start_Time = 0, CH3_Start_Time = 0, CH4_Start_Time = 0;
static volatile uint32_t CH1_PPM=0, CH2_PPM=0, CH3_PPM=0, CH4_PPM=0;

void ISR_CH1() 
{
  if (micros() > CH1_Start_Time)
  {
    CH1_PPM = micros() - CH1_Start_Time;
    CH1_Start_Time = micros();
  }
}

void ISR_CH2() 
{
  if (micros() > CH2_Start_Time)
  {
    CH2_PPM = micros() - CH2_Start_Time;
    CH2_Start_Time = micros();
  }
}

void ISR_CH3() 
{
  if (micros() > CH3_Start_Time)
  {
    CH3_PPM = micros() - CH3_Start_Time;
    CH3_Start_Time = micros();
  }
}

void ISR_CH4() 
{
  if (micros() > CH4_Start_Time)
  {
    CH4_PPM = micros() - CH4_Start_Time;
    CH4_Start_Time = micros();
  }
}

void Receiver_Init(void)
{ 
  attachInterrupt(digitalPinToInterrupt(CHANNEL_1_PIN), ISR_CH1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHANNEL_2_PIN), ISR_CH2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHANNEL_3_PIN), ISR_CH3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHANNEL_4_PIN), ISR_CH4, CHANGE); 
}

Channel_Typedef Receiver_Read_Value(void)
{
  static Channel_Typedef Channel_Value;

  if(CH1_PPM >= PPM_MIN && CH1_PPM <= PPM_MAX) //CH1
  {
    Channel_Value.CH1 = map(CH1_PPM,PPM_MIN,PPM_MAX,ZERO,CHANNEL_1_MAX);
  } 

  if(CH2_PPM >= PPM_MIN && CH2_PPM <= PPM_MAX) //CH2
  {
    Channel_Value.CH2 = map(CH2_PPM,PPM_MIN,PPM_MAX,ZERO,CHANNEL_2_MAX);
  } 

  if(CH3_PPM >= PPM_MIN && CH3_PPM <= PPM_MAX) //CH3
  {
    Channel_Value.CH3 = map(CH3_PPM,PPM_MIN,PPM_MAX,ZERO,CHANNEL_3_MAX);
  } 

  if(CH4_PPM >= PPM_MIN && CH4_PPM <= PPM_MAX) //CH4
  {
    Channel_Value.CH4 = map(CH4_PPM,PPM_MIN,PPM_MAX,ZERO,CHANNEL_4_MAX);
  } 

  return Channel_Value;
}
