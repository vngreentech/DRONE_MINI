
#include "APPLICATION.h"

HardwareSerial Serial1(RXD_1, TXD_1);

IMU_DATA_TYPEDEF Read_IMU;
Channel_Typedef Read_Channel;

static void Hardware_Driver_Init(void)
{
  Serial1.begin(SPEED_UART);
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

static void Menu_Select(void)
{
  static uint8_t Count_Menu=0;

  if( digitalRead(BUTTON_SELECT_PIN)==0 )
  {
    delay(200);
    Count_Menu+=1;
    if(Count_Menu>3) Count_Menu=0;
  }

  switch (Count_Menu)
  {
    case 0:
      LCD_Menu_IMU(Read_IMU.Pitch, Read_IMU.Roll, Read_IMU.Yaw);
      break;
    case 1:
      LCD_Menu_PID(KP, 0.0004);
      break;
    case 2:
      LCD_Menu_PID(KI, 0.0004);
      break;
    case 3:
      LCD_Menu_PID(KD, 0.0004);
      break;                  
    
    default:
      break;
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
}

void APP_MAIN(void)
{
  Read_IMU = GetIMUValue();
  Read_Channel = Receiver_Read_Value();  

  Menu_Select();

  // Serial1.print("CH1: "); Serial1.print(Read_Channel.CH1);
  // Serial1.print(" - CH2: "); Serial1.print(Read_Channel.CH2);
  // Serial1.print(" - CH3: "); Serial1.print(Read_Channel.CH3);
  // Serial1.print(" - CH4: "); Serial1.print(Read_Channel.CH4);

  // Serial1.println();
  delay(2);
}
