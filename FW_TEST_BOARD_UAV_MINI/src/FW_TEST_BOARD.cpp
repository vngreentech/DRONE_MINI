#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include "U8g2lib.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// #define SET_MOTOR
#define TXD_1 (PA9)
#define RXD_1 (PA10)
#define SDA_1 (PB7)
#define SCL_1 (PB6)

#define BUTTON_SELECT_PIN (PB12)
#define BUTTON_PLUS_PIN   (PB13)
#define BUTTON_MINUS_PIN  (PB14)
#define CHANNEL_1_PIN     (PA0)
#define CHANNEL_2_PIN     (PA1)
#define CHANNEL_3_PIN     (PA2)
#define CHANNEL_4_PIN     (PA3)
#define ESC_1_PIN         (PA4)
#define ESC_2_PIN         (PA5)
#define ESC_3_PIN         (PA6)
#define ESC_4_PIN         (PA7)

Servo Motor_1;  // ESC 1
Servo Motor_2;  // ESC 2
Servo Motor_3;  // ESC 3
Servo Motor_4;  // ESC 4

HardwareSerial Serial1(RXD_1, TXD_1);

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0); 

volatile uint32_t CH1_Start_Time = 0, CH2_Start_Time = 0, CH3_Start_Time = 0, CH4_Start_Time = 0 ;
volatile uint32_t CH1_VALUE, CH2_VALUE=0, CH3_VALUE=0, CH4_VALUE=0;
volatile uint32_t CH1_PPM=0, CH2_PPM=0, CH3_PPM=0, CH4_PPM=0;

MPU6050 mpu;
struct Orientation 
{
  float Yaw;
  float Pitch;
  float Roll;
  bool Error;
};
struct Orientation prevOrientation;
struct Orientation Angle;

Quaternion q;           
VectorFloat gravity;    
float ypr[3];  
bool dmpReady = false; 
uint8_t mpuIntStatus=0;   
uint16_t packetSize=0;
uint8_t fifoBuffer[64];

struct Orientation getIMUOrientation() 
{
  if (!dmpReady || !mpu.testConnection()) 
  {
    struct Orientation o;
    o.Yaw = 0;
    o.Pitch = 0;
    o.Roll = 0;
    o.Error = true;
    return o;
  }

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
  {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    struct Orientation o;
    o.Yaw = double( float(ypr[0] * float(180 / M_PI)) );
    o.Pitch = double( float(ypr[2] * float(180 / M_PI)) );
    o.Roll = double( float(ypr[1] * float(180 / M_PI)) );
    o.Error = false;

    prevOrientation.Yaw = o.Yaw;
    prevOrientation.Pitch = o.Pitch;
    prevOrientation.Roll = o.Roll;

    return o;
  } 
  else {return prevOrientation;}
}

void setupMPU() 
{
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  mpuIntStatus = mpu.getIntStatus();

  Serial1.println(mpuIntStatus);

  dmpReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();  

  /*
  MPU6050_GYRO_FS_250  
  MPU6050_GYRO_FS_500  
  MPU6050_GYRO_FS_1000 
  MPU6050_GYRO_FS_2000 
  */
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

  /*
  MPU6050_ACCEL_FS_2  
  MPU6050_ACCEL_FS_4  
  MPU6050_ACCEL_FS_8  
  MPU6050_ACCEL_FS_16 
  */
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
  
  /*
  MPU6050_DLPF_BW_256
  MPU6050_DLPF_BW_188
  MPU6050_DLPF_BW_98 
  MPU6050_DLPF_BW_42 
  MPU6050_DLPF_BW_20 
  MPU6050_DLPF_BW_10 
  MPU6050_DLPF_BW_5  
  */
  mpu.setDLPFMode(MPU6050_DLPF_BW_188);
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6); 
}

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

void STOP_MOTOR(void)
{
  Motor_1.write(0);
  Motor_2.write(0);
  Motor_3.write(0);
  Motor_4.write(0);
}

void CONTROL_ALL_MOTOR(uint32_t INPUT_VALUE)
{
  Motor_1.write(INPUT_VALUE);
  Motor_2.write(INPUT_VALUE);
  Motor_3.write(INPUT_VALUE);
  Motor_4.write(INPUT_VALUE);
}

void setup(void) 
{
  Serial1.begin(115200);
  Wire.setSDA(SDA_1);
  Wire.setSCL(SCL_1);
  Wire.setClock(400000);  
  Wire.begin();    

  pinMode(BUTTON_SELECT_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PLUS_PIN, INPUT_PULLUP);
  pinMode(BUTTON_MINUS_PIN, INPUT_PULLUP);
  pinMode(CHANNEL_1_PIN, INPUT);
  pinMode(CHANNEL_2_PIN, INPUT);
  pinMode(CHANNEL_3_PIN, INPUT);
  pinMode(CHANNEL_4_PIN, INPUT);  

  attachInterrupt(digitalPinToInterrupt(CHANNEL_1_PIN), ISR_CH1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHANNEL_2_PIN), ISR_CH2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHANNEL_3_PIN), ISR_CH3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHANNEL_4_PIN), ISR_CH4, CHANGE);    
  
  Motor_1.attach(ESC_1_PIN,1000,2000);
  Motor_2.attach(ESC_2_PIN,1000,2000);
  Motor_3.attach(ESC_3_PIN,1000,2000);
  Motor_4.attach(ESC_4_PIN,1000,2000);  
  STOP_MOTOR();    

  setupMPU();
  u8g2.begin();
  u8g2.setFont( u8g2_font_helvB14_tr);
}

#ifdef SET_MOTOR
void loop(void)
{
  if(CH3_PPM >= 1000 && CH3_PPM <= 2000) //CH3
  {
    CH3_VALUE = map(CH3_PPM,1000,2000,0,180);
  } 

  CONTROL_ALL_MOTOR(CH3_VALUE);  
}
#endif //SET_MOTOR  

#ifndef SET_MOTOR
void loop(void) 
{
  static int count=0;
  Angle = getIMUOrientation();

  if(CH1_PPM >= 1000 && CH1_PPM <= 2000) //CH1
  {
    CH1_VALUE = map(CH1_PPM,1000,2000,0,180);
  } 

  if(CH2_PPM >= 1000 && CH2_PPM <= 2000) //CH2
  {
    CH2_VALUE = map(CH2_PPM,1000,2000,0,180);
  } 

  if(CH3_PPM >= 1000 && CH3_PPM <= 2000) //CH3
  {
    CH3_VALUE = map(CH3_PPM,1000,2000,0,180);
  } 

  if(CH4_PPM >= 1000 && CH4_PPM <= 2000) //CH4
  {
    CH4_VALUE = map(CH4_PPM,1000,2000,0,180);
  }   

  if(CH3_VALUE<=10) STOP_MOTOR();
  else CONTROL_ALL_MOTOR(CH3_VALUE);    

  if( digitalRead(BUTTON_SELECT_PIN)==0 || digitalRead(BUTTON_PLUS_PIN)==0 || digitalRead(BUTTON_MINUS_PIN)==0)
  {
    delay(200);
    count+=1;
    if(count>6) count=0;
  }

  if(count==0) //Pitch
  {
    u8g2.clearBuffer();
    u8g2.setCursor(1, 22);
    u8g2.print("P:");
    u8g2.setCursor(30, 22);
    u8g2.print(Angle.Pitch);
    u8g2.sendBuffer();  
  }
  else if(count==1) //ROLL
  {
    u8g2.clearBuffer();
    u8g2.setCursor(1, 22);
    u8g2.print("R:");
    u8g2.setCursor(30, 22);
    u8g2.print(Angle.Roll);
    u8g2.sendBuffer();  
  } 
  else if(count==2) //YAW
  {
    u8g2.clearBuffer();
    u8g2.setCursor(1, 22);
    u8g2.print("Y:");
    u8g2.setCursor(30, 22);
    u8g2.print(Angle.Yaw);
    u8g2.sendBuffer();  
  }  
  else if(count==3) //CH1
  {
    u8g2.clearBuffer();
    u8g2.setCursor(1, 22);
    u8g2.print("CH1:");
    u8g2.setCursor(50, 22);
    u8g2.print(CH1_VALUE);
    u8g2.sendBuffer();  
  }  
  else if(count==4) //CH2
  {
    u8g2.clearBuffer();
    u8g2.setCursor(1, 22);
    u8g2.print("CH2:");
    u8g2.setCursor(50, 22);
    u8g2.print(CH2_VALUE);
    u8g2.sendBuffer();  
  }  
  else if(count==5) //CH3
  {
    u8g2.clearBuffer();
    u8g2.setCursor(1, 22);
    u8g2.print("CH3:");
    u8g2.setCursor(50, 22);
    u8g2.print(CH3_VALUE);
    u8g2.sendBuffer();  
  }  
  else if(count==6) //CH4
  {
    u8g2.clearBuffer();
    u8g2.setCursor(1, 22);
    u8g2.print("CH4:");
    u8g2.setCursor(50, 22);
    u8g2.print(CH4_VALUE);
    u8g2.sendBuffer();  
  }            

  delay(10);
}
#endif //SET_MOTOR
