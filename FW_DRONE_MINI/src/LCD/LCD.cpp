
#include "LCD.h"

#ifdef FIRMWARE_VERSION
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0);

void LCD_Init(void)
{
  u8g2.begin();

  u8g2.setFont(u8g2_font_helvB08_tr);

  u8g2.clearBuffer();
  u8g2.setCursor(8, 10);
  u8g2.print("GREEN TECHNOLOGY");
  u8g2.setCursor(15, 30);
  u8g2.print("MADE IN VIETNAM");  
  u8g2.sendBuffer(); 

  delay(2000);

  u8g2.clearBuffer();
  u8g2.setCursor(10, 10);
  u8g2.print("VNGREENTECH.COM");
  u8g2.setCursor(8, 30);
  u8g2.print("Design by NhanNguyen");  
  u8g2.sendBuffer();   

  u8g2.setFont(u8g2_font_helvB14_tr);

  delay(2000);
}

void LCD_Menu_IMU(float PITCH, float ROLL, float YAW)
{
  u8g2.setFont(u8g2_font_helvB08_tr);
  u8g2.clearBuffer();

  u8g2.setCursor(1, 8);
  u8g2.print("Pitch:");
  u8g2.setCursor(35, 8);
  u8g2.print(PITCH);

  u8g2.setCursor(1, 19);
  u8g2.print("Roll:");
  u8g2.setCursor(35, 19);
  u8g2.print(ROLL);  

  u8g2.setCursor(1, 32);
  u8g2.print("Yaw:");
  u8g2.setCursor(35, 32);
  u8g2.print(YAW);   

  u8g2.setCursor(80, 32);
  u8g2.print(FIRMWARE_VERSION);   

  u8g2.sendBuffer();  
}

void LCD_Menu_PID(PID_typedef PID_Select, float PID_Value)
{
  u8g2.setFont(u8g2_font_helvB14_tr);
  u8g2.clearBuffer();
  u8g2.setCursor(1, 22);

  switch (PID_Select)
  {
    case KP:
      u8g2.print("KP:");
      u8g2.setCursor(40, 22);
      u8g2.print(PID_Value,2);
      break;
    case KI:
      u8g2.print("KI:");
      u8g2.setCursor(35, 22);
      u8g2.print(PID_Value,3);
      break;
    case KD:
      u8g2.print("KD:");
      u8g2.setCursor(40, 22);
      u8g2.print(PID_Value,2);
      break;   

    case KP_YAW:
      u8g2.print("KP_YAW:");
      u8g2.setCursor(90, 22);
      u8g2.print(PID_Value,2);
      break;
    case KI_YAW:
      u8g2.print("KI_YAW:");
      u8g2.setCursor(82, 22);
      u8g2.print(PID_Value,3);
      break;
    case KD_YAW:
      u8g2.print("KD_YAW:");
      u8g2.setCursor(90, 22);
      u8g2.print(PID_Value,2);
      break;                           
    
    default:
      break;
  }
  
  u8g2.sendBuffer(); 
}

void LCD_Menu_RESET(void)
{
  u8g2.setFont(u8g2_font_helvB14_tr);
  u8g2.clearBuffer();
  u8g2.setCursor(1, 22);
  u8g2.print("RESET...!!!");
  u8g2.sendBuffer();
}

void LCD_Menu_SET_DEFAULT_PID_VALUE(uint8_t Step)
{
  switch (Step)
  {
    case 1:
      u8g2.setFont(u8g2_font_helvB08_tr);
      u8g2.clearBuffer();
      u8g2.setCursor(1, 20);
      u8g2.print("SAVE DEFAULT SETTING");
      u8g2.setCursor(1, 30);
      u8g2.print("...........");  
      u8g2.sendBuffer();
      break;

    case 2:
      u8g2.clearBuffer();
      u8g2.setCursor(1, 10);
      u8g2.print("PLEASE !!!");
      u8g2.setCursor(1, 30);
      u8g2.print("RESET BOARD....");  
      u8g2.sendBuffer();
      break;      
    
    default:
      break;
  }

}

void LCD_Menu_SAVE(void)
{
  u8g2.setFont(u8g2_font_helvB14_tr);
  u8g2.clearBuffer();
  u8g2.setCursor(1, 22);
  u8g2.print("SAVE PID...");
  u8g2.sendBuffer();
}
#endif /* FIRMWARE_VERSION */

