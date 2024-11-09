
#include "APPLICATION.h"

#define TimeCalPID  (float(5.0)) //ms

HardwareSerial Serial1(RXD_1, TXD_1);
static IMU_DATA_TYPEDEF Read_IMU;
static Channel_Typedef Read_Channel;
static PID_VALUE_typedef PID_Set_Value;

PID_OBJECT_typedef PITCH_PID;
PID_OBJECT_typedef ROLL_PID;

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

static void PID_SET_VALUE(float *PIDValue)
{
  if( digitalRead(BUTTON_PLUS_PIN)==false && digitalRead(BUTTON_MINUS_PIN)==true )
  {
    delay(30);
    *PIDValue = *PIDValue + 0.0001;
    if(*PIDValue>=1000) *PIDValue=1000;
  }

  if( digitalRead(BUTTON_MINUS_PIN)==false && digitalRead(BUTTON_PLUS_PIN)==true )
  {
    delay(30);
    *PIDValue = *PIDValue - 0.0001;
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
    if(digitalRead(BUTTON_SELECT_PIN)==false)
    {
      LastTimeTick=MILLIS;
      STEP=1;
    }
  }
  else if(STEP==1)
  {
    /* Select menu */
    if( ((uint32_t)(MILLIS-LastTimeTick)<=200) && digitalRead(BUTTON_SELECT_PIN)==true && \
        digitalRead(BUTTON_PLUS_PIN)==true && digitalRead(BUTTON_MINUS_PIN)==true )
    {
      delay(200);
      Count_Menu+=1;
      if(Count_Menu>3) Count_Menu=0;
      STEP=0;
    }   

    /* Save new PID value */
    if( ((uint32_t)(MILLIS-LastTimeTick)>=3000) && digitalRead(BUTTON_SELECT_PIN)==true && \
      digitalRead(BUTTON_PLUS_PIN)==true && digitalRead(BUTTON_MINUS_PIN)==true )
    {
      LCD_Menu_SAVE();
      EEP_SAVE_PID_Value(&PID_Set_Value);  
      STEP=0;
      delay(3000);
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
      case 1:
        PID_SET_VALUE(&PID_Set_Value.KP);
        LCD_Menu_PID(KP, PID_Set_Value.KP);
        break;
      case 2:
        PID_SET_VALUE(&PID_Set_Value.KI);
        LCD_Menu_PID(KI, PID_Set_Value.KI);
        break;
      case 3:
        PID_SET_VALUE(&PID_Set_Value.KD);
        LCD_Menu_PID(KD, PID_Set_Value.KD);
        break;                  
      
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
  delay(1);
}

static void RESET_MACHINE(void)
{
  static uint8_t STEP=0;
  static uint32_t LastTimeTick=0;

  if(STEP==0)
  {
    if( digitalRead(BUTTON_SELECT_PIN)==0 && digitalRead(BUTTON_PLUS_PIN)==0 \
        && digitalRead(BUTTON_MINUS_PIN)==0 )
    {
      LastTimeTick=MILLIS;
      STEP=1;
    }
  }
  else if(STEP==1)
  {
    if( (uint32_t)(MILLIS-LastTimeTick)>=3000 && digitalRead(BUTTON_SELECT_PIN)==0\
        && digitalRead(BUTTON_PLUS_PIN)==0 && digitalRead(BUTTON_MINUS_PIN)==0)
    {
      LCD_Menu_RESET();
      
      PID_Set_Value.KP=0.2;
      PID_Set_Value.KI=0.05;
      PID_Set_Value.KD=0.1;
      EEP_SAVE_PID_Value(&PID_Set_Value);  
      delay(3000);

      NVIC_SystemReset();
    }

    if( digitalRead(BUTTON_SELECT_PIN)==1 && digitalRead(BUTTON_PLUS_PIN)==1 \
        && digitalRead(BUTTON_MINUS_PIN)==1 )
    {
      STEP=0; LastTimeTick=0;
    }

  }
}

static void PID_MOTOR_CONTROL(void)
{
  PITCH_PID.ReadValue = Read_IMU.Pitch;
  ROLL_PID.ReadValue = Read_IMU.Roll;

  if( (uint32_t)(MILLIS-LastTimePID) >= TimeCalPID )
  {
    PID_CALCULATOR(&PITCH_PID);
    PID_CALCULATOR(&ROLL_PID);

    PITCH_PID.Output = constrain(PITCH_PID.Output, -PID_CONTROL_VALUE_LIMIT, PID_CONTROL_VALUE_LIMIT);
    ROLL_PID.Output = constrain(ROLL_PID.Output, -PID_CONTROL_VALUE_LIMIT, PID_CONTROL_VALUE_LIMIT);

    MOTOR_Speed_typedef Speed_Motor;
    Speed_Motor.Motor_1 = round(Read_Channel.CH3 + PITCH_PID.Output + ROLL_PID.Output);
    Speed_Motor.Motor_2 = round(Read_Channel.CH3 + PITCH_PID.Output - ROLL_PID.Output);
    Speed_Motor.Motor_3 = round(Read_Channel.CH3 - PITCH_PID.Output + ROLL_PID.Output);
    Speed_Motor.Motor_4 = round(Read_Channel.CH3 - PITCH_PID.Output - ROLL_PID.Output);

    Motor_Control(MOTOR_1, Speed_Motor.Motor_1);
    Motor_Control(MOTOR_2, Speed_Motor.Motor_2);
    Motor_Control(MOTOR_3, Speed_Motor.Motor_3);
    Motor_Control(MOTOR_4, Speed_Motor.Motor_4);

    PITCH_PID.LastTimeCalPID=MILLIS;
    ROLL_PID.LastTimeCalPID=MILLIS;
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

  PITCH_PID.SetValue=0.0;
  PITCH_PID.PIDValue=&PID_Set_Value;
  
  ROLL_PID.SetValue=0.0;
  ROLL_PID.PIDValue=&PID_Set_Value;

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
  }
  else 
  {
    PID_MOTOR_CONTROL();
    // Motor_Control(ALL_MOTOR, Read_Channel.CH3);
  }

  // static uint64_t stop=MICROS;
  // Serial1.print(" - Stop Time: "); Serial1.println(stop-start);

  // Serial1.println();
  delay(1);
}
