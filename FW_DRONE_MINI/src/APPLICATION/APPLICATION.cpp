
#include "APPLICATION.h"
#ifdef FIRMWARE_VERSION

HardwareSerial Serial1(RXD_1, TXD_1);
static IMU_DATA_TYPEDEF Read_IMU;
static Channel_Typedef Read_Channel;
static PID_VALUE_typedef PID_Set_Value;

PID_OBJECT_typedef PITCH_PID;
PID_OBJECT_typedef ROLL_PID;
PID_OBJECT_typedef YAW_PID;

static uint32_t LastTimeLCD=0;
static uint32_t LastTimePID=0;

static void EEP_SAVE_PID_Value(PID_VALUE_typedef *PIDValue);
static void RESET_MACHINE(void);

static void Hardware_Driver_Init(void)
{
  Serial1.begin(SPEED_UART);
  EEPROM.begin();
  Wire.setSDA(SDA_1);
  Wire.setSCL(SCL_1);
  Wire.setClock(SPEED_I2C);  
  Wire.begin();  

  pinMode(BUTTON_SELECT_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PLUS_PIN, INPUT_PULLUP);
  pinMode(BUTTON_MINUS_PIN, INPUT_PULLUP);
  pinMode(CHANNEL_1_PIN, INPUT);
  pinMode(CHANNEL_2_PIN, INPUT);
  pinMode(CHANNEL_3_PIN, INPUT);
  pinMode(CHANNEL_4_PIN, INPUT);   
}

static void PID_SET_VALUE(PID_typedef SelectPID, float *PIDValue)
{
  if( READ_BUTTON_PLUS==false && READ_BUTTON_MINUS==true )
  {
    if(SelectPID==KP) *PIDValue = *PIDValue + 0.01;
    else if(SelectPID==KI) *PIDValue = *PIDValue + 0.001;
    else if(SelectPID==KD)  *PIDValue = *PIDValue + 0.01;
    
    else if(SelectPID==KP_YAW)  *PIDValue = *PIDValue + 0.01;
    else if(SelectPID==KI_YAW) *PIDValue = *PIDValue + 0.001;
    else *PIDValue = *PIDValue + 0.01;

    if(*PIDValue>=1000) *PIDValue=1000;
  }

  if( READ_BUTTON_MINUS==false && READ_BUTTON_PLUS==true )
  {
    if(SelectPID==KP) *PIDValue = *PIDValue - 0.01;
    else if(SelectPID==KI) *PIDValue = *PIDValue - 0.001;
    else if(SelectPID==KD)  *PIDValue = *PIDValue - 0.01;   

    else if(SelectPID==KP_YAW) *PIDValue = *PIDValue - 0.01;
    else if(SelectPID==KI_YAW) *PIDValue = *PIDValue - 0.001;
    else *PIDValue = *PIDValue - 0.01;

    if(*PIDValue<=0) *PIDValue=0;
  }  
}

static void Menu_Select(void)
{
  static uint32_t LastTimeTick=0;
  static uint8_t STEP=0;  
  static uint8_t Count_Menu=0;

  if(STEP==0)
  {
    if(READ_BUTTON_SELECT==false)
    {
      LastTimeTick=MILLIS;
      STEP=1;
    }
  }
  else if(STEP==1)
  {
    /* Select menu */
    if( ((uint32_t)(MILLIS-LastTimeTick)<=500) && READ_BUTTON_SELECT==true && \
        READ_BUTTON_PLUS==true && READ_BUTTON_MINUS==true )
    {
      PAUSE_MILLIS(200);
      Count_Menu+=1;
      
      #if ( FIRMWARE_VERSION_CHECK == 14 || \
            FIRMWARE_VERSION_CHECK == 17)
      if(Count_Menu>6) Count_Menu=0;
      #endif /* FIRMWARE_VERSION_CHECK */

      #if (FIRMWARE_VERSION_CHECK == 15)
      if(Count_Menu>3) Count_Menu=0;
      #endif /* FIRMWARE_VERSION_CHECK */
      
      STEP=0;
    }   

    /* Save new PID value */
    if( ((uint32_t)(MILLIS-LastTimeTick)>=3000) && READ_BUTTON_SELECT==true && \
      READ_BUTTON_PLUS==true && READ_BUTTON_MINUS==true )
    {
      LCD_Menu_SAVE();
      EEP_SAVE_PID_Value(&PID_Set_Value);  
      STEP=0;
      PAUSE_MILLIS(3000);
    }
  }  

  if( (uint32_t)(MILLIS - LastTimeLCD)>=100 )
  {
    switch (Count_Menu)
    {
      case 0:
        RESET_MACHINE();
        LCD_Menu_IMU(Read_IMU.Pitch, Read_IMU.Roll, Read_IMU.Yaw);
        break;

      case 1: /* KP */
        PID_SET_VALUE(KP, &PID_Set_Value.KP);
        LCD_Menu_PID(KP, PID_Set_Value.KP);
        break;
      case 2: /* KI */
        PID_SET_VALUE(KI, &PID_Set_Value.KI);
        LCD_Menu_PID(KI, PID_Set_Value.KI);
        break;
      case 3: /* KD */
        PID_SET_VALUE(KD, &PID_Set_Value.KD);
        LCD_Menu_PID(KD, PID_Set_Value.KD);
        break;

      #if ( FIRMWARE_VERSION_CHECK == 14 || \
            FIRMWARE_VERSION_CHECK == 17)
        case 4: /* KP_YAW */
          PID_SET_VALUE(KP_YAW, &PID_Set_Value.KP_YAW);
          LCD_Menu_PID(KP_YAW, PID_Set_Value.KP_YAW);
          break;
        case 5: /* KI_YAW */
          PID_SET_VALUE(KI_YAW, &PID_Set_Value.KI_YAW);
          LCD_Menu_PID(KI_YAW, PID_Set_Value.KI_YAW);
          break;
        case 6: /* KD_YAW */
          PID_SET_VALUE(KD_YAW, &PID_Set_Value.KD_YAW);
          LCD_Menu_PID(KD_YAW, PID_Set_Value.KD_YAW);
          break; 
      #endif /* FIRMWARE_VERSION_CHECK */                   
      
      default:
        break;
    }

    LastTimeLCD=MILLIS;
  }
}

static PID_VALUE_typedef EEP_Read_PID_Value(void)
{
  PID_VALUE_typedef ReadPID;
  EEPROM.get(10, ReadPID);
  return ReadPID;
}

static void EEP_SAVE_PID_Value(PID_VALUE_typedef *PIDValue)
{
  EEPROM.put(10, *PIDValue);
  PAUSE_MILLIS(1);
}

static void RESET_MACHINE(void)
{
  static uint8_t STEP=0;
  static uint32_t LastTimeTick=0;

  if(STEP==0)
  {
    if( READ_BUTTON_SELECT==0 && READ_BUTTON_PLUS==0 \
        && READ_BUTTON_MINUS==0 )
    {
      LastTimeTick=MILLIS;
      STEP=1;
    }
  }
  else if(STEP==1)
  {
    if( (uint32_t)(MILLIS-LastTimeTick)>=3000 && READ_BUTTON_SELECT==0\
        && READ_BUTTON_PLUS==0 && READ_BUTTON_MINUS==0)
    {
      LCD_Menu_RESET();
      
      PID_Set_Value.KP=0.2;
      PID_Set_Value.KI=0.05;
      PID_Set_Value.KD=0.1;
      PID_Set_Value.KP_YAW=0.5;
      PID_Set_Value.KI_YAW=0.05;
      PID_Set_Value.KD_YAW=0.1;
      EEP_SAVE_PID_Value(&PID_Set_Value);  
      PAUSE_MILLIS(3000);

      NVIC_SystemReset();
    }

    if( READ_BUTTON_SELECT==1 && READ_BUTTON_PLUS==1 \
        && READ_BUTTON_MINUS==1 )
    {
      STEP=0; LastTimeTick=0;
    }

  }
}

static void PID_MOTOR_CONTROL(void)
{
  static MOTOR_Speed_typedef Speed_Motor;
  static float PITCH_CONTROL = 0;
  static float ROLL_CONTROL = 0;
  static float YAW_CONTROL = 0;
  
  PITCH_PID.ReadValue = Read_IMU.Pitch;
  // if(PITCH_PID.ReadValue>=PITCH_ROLL_DEGREE_LIMIT) PITCH_PID.ReadValue==PITCH_ROLL_DEGREE_LIMIT;
  // else if(PITCH_PID.ReadValue<=-PITCH_ROLL_DEGREE_LIMIT) PITCH_PID.ReadValue==-PITCH_ROLL_DEGREE_LIMIT;

  ROLL_PID.ReadValue = Read_IMU.Roll;
  // if(ROLL_PID.ReadValue>=PITCH_ROLL_DEGREE_LIMIT) ROLL_PID.ReadValue==PITCH_ROLL_DEGREE_LIMIT;
  // else if(ROLL_PID.ReadValue<=-PITCH_ROLL_DEGREE_LIMIT) ROLL_PID.ReadValue==-PITCH_ROLL_DEGREE_LIMIT;

  #if ( FIRMWARE_VERSION_CHECK == 14 || \
        FIRMWARE_VERSION_CHECK == 17)
  YAW_PID.ReadValue = Read_IMU.Yaw;
  #endif /* FIRMWARE_VERSION_CHECK */

  if( (uint32_t)(MILLIS-LastTimePID) >= TimeCalPID )
  {
    PITCH_CONTROL = map(Read_Channel.CH2,ZERO,CHANNEL_2_MAX, PITCH_CONTROL_LIMIT, -PITCH_CONTROL_LIMIT);
    ROLL_CONTROL = map(Read_Channel.CH1,ZERO,CHANNEL_1_MAX, ROLL_CONTROL_LIMIT, -ROLL_CONTROL_LIMIT);
    
    #if ( FIRMWARE_VERSION_CHECK == 14 || \
          FIRMWARE_VERSION_CHECK == 17)
    YAW_PID.SetValue = map(Read_Channel.CH4,ZERO,CHANNEL_4_MAX, YAW_CONTROL_LIMIT, -YAW_CONTROL_LIMIT);
    #endif /* FIRMWARE_VERSION_CHECK */

    #if (FIRMWARE_VERSION_CHECK == 15)
    YAW_CONTROL = map(Read_Channel.CH4,ZERO,CHANNEL_4_MAX, YAW_CONTROL_LIMIT, -YAW_CONTROL_LIMIT);
    #endif /* FIRMWARE_VERSION_CHECK */

    #if (FIRMWARE_VERSION_CHECK == 16)
    YAW_PID.SetValue += map(Read_Channel.CH4,ZERO,CHANNEL_4_MAX, YAW_CONTROL_LIMIT, -YAW_CONTROL_LIMIT);
    if(YAW_PID.SetValue>=360) YAW_PID.SetValue = 360;
    else if(YAW_PID.SetValue<=-360) YAW_PID.SetValue = -360;
    #endif /* FIRMWARE_VERSION_CHECK */    

    PID_CALCULATOR(&PITCH_PID);
    PITCH_PID.Output = constrain(PITCH_PID.Output, -PID_CONTROL_VALUE_LIMIT, PID_CONTROL_VALUE_LIMIT);

    PID_CALCULATOR(&ROLL_PID);
    ROLL_PID.Output = constrain(ROLL_PID.Output, -PID_CONTROL_VALUE_LIMIT, PID_CONTROL_VALUE_LIMIT);
    
    #if ( FIRMWARE_VERSION_CHECK == 14 || \
          FIRMWARE_VERSION_CHECK == 17)
    PID_CAL_YAW(&YAW_PID);
    YAW_PID.Output = constrain(YAW_PID.Output, -PID_CONTROL_VALUE_LIMIT, PID_CONTROL_VALUE_LIMIT);
    YAW_PID.Output = -YAW_PID.Output;
    #endif /* FIRMWARE_VERSION_CHECK */

    /*PITCH control: OK*/
    // Speed_Motor.Motor_1 = round(Read_Channel.CH3 + PITCH_PID.Output);
    // Speed_Motor.Motor_2 = round(Read_Channel.CH3 + PITCH_PID.Output);
    // Speed_Motor.Motor_3 = round(Read_Channel.CH3 - PITCH_PID.Output);
    // Speed_Motor.Motor_4 = round(Read_Channel.CH3 - PITCH_PID.Output);

    /* ROLL control: OK */
    // Speed_Motor.Motor_1 = round(Read_Channel.CH3 - ROLL_PID.Output);
    // Speed_Motor.Motor_2 = round(Read_Channel.CH3 + ROLL_PID.Output);
    // Speed_Motor.Motor_3 = round(Read_Channel.CH3 + ROLL_PID.Output);
    // Speed_Motor.Motor_4 = round(Read_Channel.CH3 - ROLL_PID.Output);

    /* CH3 + PID control */
    Speed_Motor.Motor_1 = round(Read_Channel.CH3 + PITCH_PID.Output - ROLL_PID.Output);
    Speed_Motor.Motor_2 = round(Read_Channel.CH3 + PITCH_PID.Output + ROLL_PID.Output);
    Speed_Motor.Motor_3 = round(Read_Channel.CH3 - PITCH_PID.Output + ROLL_PID.Output);
    Speed_Motor.Motor_4 = round(Read_Channel.CH3 - PITCH_PID.Output - ROLL_PID.Output);

    /* PITCH + ROLL */
    Speed_Motor.Motor_1 = round(Speed_Motor.Motor_1 - ROLL_CONTROL + PITCH_CONTROL);
    Speed_Motor.Motor_2 = round(Speed_Motor.Motor_2 + ROLL_CONTROL + PITCH_CONTROL);
    Speed_Motor.Motor_3 = round(Speed_Motor.Motor_3 + ROLL_CONTROL - PITCH_CONTROL);
    Speed_Motor.Motor_4 = round(Speed_Motor.Motor_4 - ROLL_CONTROL - PITCH_CONTROL);    

    /* PITCH + ROLL + YAW */
    #ifdef ENABLE_YAW_CONTROL
      #if ( FIRMWARE_VERSION_CHECK == 14 || \
            FIRMWARE_VERSION_CHECK == 17)
      Speed_Motor.Motor_1 = round(Speed_Motor.Motor_1 + YAW_PID.Output);
      Speed_Motor.Motor_2 = round(Speed_Motor.Motor_2 - YAW_PID.Output);
      Speed_Motor.Motor_3 = round(Speed_Motor.Motor_3 + YAW_PID.Output);
      Speed_Motor.Motor_4 = round(Speed_Motor.Motor_4 - YAW_PID.Output);  
      #endif /* FIRMWARE_VERSION_CHECK */  

      #if (FIRMWARE_VERSION_CHECK == 15)
      Speed_Motor.Motor_1 = round(Speed_Motor.Motor_1 + YAW_CONTROL);
      Speed_Motor.Motor_2 = round(Speed_Motor.Motor_2 - YAW_CONTROL);
      Speed_Motor.Motor_3 = round(Speed_Motor.Motor_3 + YAW_CONTROL);
      Speed_Motor.Motor_4 = round(Speed_Motor.Motor_4 - YAW_CONTROL);  
      #endif /* FIRMWARE_VERSION_CHECK */      
    #endif /* ENABLE_YAW_CONTROL */    
    
    Speed_Motor.Motor_1 = constrain(Speed_Motor.Motor_1,ZERO,SERVO_MAX);
    Speed_Motor.Motor_2 = constrain(Speed_Motor.Motor_2,ZERO,SERVO_MAX);
    Speed_Motor.Motor_3 = constrain(Speed_Motor.Motor_3,ZERO,SERVO_MAX);
    Speed_Motor.Motor_4 = constrain(Speed_Motor.Motor_4,ZERO,SERVO_MAX);

    if(Read_Channel.CH3<=10)
    {
      Speed_Motor.Motor_1 = 0;
      Speed_Motor.Motor_2 = 0;
      Speed_Motor.Motor_3 = 0;
      Speed_Motor.Motor_4 = 0;
    }
    
    #ifndef STOP_FOR_TEST
    Motor_Control(MOTOR_1, Speed_Motor.Motor_1);
    Motor_Control(MOTOR_2, Speed_Motor.Motor_2);
    Motor_Control(MOTOR_3, Speed_Motor.Motor_3);
    Motor_Control(MOTOR_4, Speed_Motor.Motor_4);
    #endif /*STOP_FOR_TEST*/

    PITCH_PID.LastTimeCalPID=MILLIS;
    ROLL_PID.LastTimeCalPID=MILLIS;

    #if ( FIRMWARE_VERSION_CHECK == 14 || \
          FIRMWARE_VERSION_CHECK == 17)
    YAW_PID.LastTimeCalPID=MILLIS;
    #endif /* FIRMWARE_VERSION_CHECK */

    LastTimePID = MILLIS;
  }
}

static void Check_Reset_PID(void)
{
  if( (Read_IMU.Pitch>=60 || Read_IMU.Pitch<=-60) ||\
      (Read_IMU.Roll>=60 || Read_IMU.Roll<=-60) )
  {
    Motor_Stop();
    PID_RESET_DATA(&PITCH_PID);
    PID_RESET_DATA(&ROLL_PID);

    #if ( FIRMWARE_VERSION_CHECK == 14 || \
          FIRMWARE_VERSION_CHECK == 17)
    PID_RESET_DATA(&YAW_PID);
    #endif /* FIRMWARE_VERSION_CHECK */
  }
}

void APP_INIT(void)
{  
  /*Hardware init*/
  Hardware_Driver_Init();

  /*Receiver init*/
  Receiver_Init();

  /*Motor init*/
  Motor_Init();

  /*LCD init*/
  LCD_Init();
  LCD_Menu_Main();

  /*IMU init*/
  setupMPU();

  PID_Set_Value=EEP_Read_PID_Value();
  // PID_Set_Value.KP=0.2;
  // PID_Set_Value.KI=0.1;
  // PID_Set_Value.KD=0.16;
  // PID_Set_Value.KP_YAW=0.5;
  // PID_Set_Value.KI_YAW=0.05;
  // PID_Set_Value.KD_YAW=0.1;  
  // EEP_SAVE_PID_Value(&PID_Set_Value);

  PITCH_PID.SetValue=0.0;
  PITCH_PID.PIDValue=&PID_Set_Value;
  
  ROLL_PID.SetValue=0.0;
  ROLL_PID.PIDValue=&PID_Set_Value;

  #if ( FIRMWARE_VERSION_CHECK == 14 || \
        FIRMWARE_VERSION_CHECK == 17)
  YAW_PID.SetValue=0.0;
  YAW_PID.PIDValue=&PID_Set_Value;
  #endif /* FIRMWARE_VERSION_CHECK */

  LastTimeLCD=MILLIS;
  LastTimePID=MILLIS;
}

void APP_MAIN(void)
{
  // static uint64_t start=MICROS;

  Read_IMU = GetIMUValue();
  Read_Channel = Receiver_Read_Value();
  Check_Reset_PID();  
  Menu_Select();

  if(Read_Channel.CH3<=10)
  {
    Motor_Stop();
    PID_RESET_DATA(&PITCH_PID);
    PID_RESET_DATA(&ROLL_PID);

    #if ( FIRMWARE_VERSION_CHECK == 14 || \
          FIRMWARE_VERSION_CHECK == 17)
    PID_RESET_DATA(&YAW_PID);
    #endif /* FIRMWARE_VERSION_CHECK */
  }
  else 
  {
    PID_MOTOR_CONTROL();
    // Motor_Control(ALL_MOTOR, Read_Channel.CH3);
  }

  // static uint64_t stop=MICROS;
  // Serial1.print(" - Stop Time: "); Serial1.println(stop-start);

  // Serial1.print("CH1: "); Serial1.print(Read_Channel.CH1);
  // Serial1.print(" - CH2: "); Serial1.println(Read_Channel.CH2);

  // Serial1.println();
  PAUSE_MICROS(100);
}
#endif /* FIRMWARE_VERSION */
