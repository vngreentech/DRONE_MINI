
#ifndef __GLOBAL__
#define __GLOBAL__

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include "SimpleKalmanFilter.h"
#include "U8g2lib.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "VERSION.h"

/*================================
        Hardware map PIN
================================*/
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
/*==============================*/

/*================================
        General Define
================================*/
#define READ_BUTTON_PLUS    (digitalRead(BUTTON_PLUS_PIN))
#define READ_BUTTON_MINUS   (digitalRead(BUTTON_MINUS_PIN))
#define READ_BUTTON_SELECT  (digitalRead(BUTTON_SELECT_PIN))
#define MILLIS (millis())
#define MICROS (micros())
#define PAUSE_MILLIS(Time) (delay(Time))
#define PAUSE_MICROS(Time) (delayMicroseconds(Time))
#define SPEED_UART (115200UL)
#define SPEED_I2C (400000UL)
#define ZERO (0UL)
/*==============================*/

/*================================
        PPM value define
================================*/
#define PPM_MIN (1000UL)
#define PPM_MAX (2000UL)
/*==============================*/

/*================================
            TESTER
================================*/
// #define STOP_FOR_TEST
/*==============================*/

/*================================
        PID Default Value
================================*/
#define PID_CHECK_SAVE ((float)0x2F)

#define PID_KP_DEFAULT ((float)0.2)
#define PID_KI_DEFAULT ((float)0.02)
#define PID_KD_DEFAULT ((float)0.08)

#define PID_KP_YAW_DEFAULT ((float)0.15)
#define PID_KI_YAW_DEFAULT ((float)0.004)
#define PID_KD_YAW_DEFAULT ((float)0.04)
/*==============================*/

/*================================
        Servo value define
================================*/
#define MOTOR_1_MAX (180UL)
#define MOTOR_2_MAX (180UL)
#define MOTOR_3_MAX (180UL)
#define MOTOR_4_MAX (180UL)

#define CHANNEL_1_MAX (180UL)
#define CHANNEL_2_MAX (180UL)
#define CHANNEL_3_MAX (180UL)
#define CHANNEL_4_MAX (180UL)

#define SERVO_MIN (0UL)
#define SERVO_MAX (180UL)

#define PID_CONTROL_VALUE_LIMIT (150)

/* PITCH control limit */
#define PITCH_CONTROL_LIMIT (20)
/* ROLL control limit */
#define ROLL_CONTROL_LIMIT  (20)
/* YAW CONTROL limit (Degree) */
#define YAW_CONTROL_LIMIT   (50)

#define YAW_GAINS_VALUE (1)

/* Enable yaw control */
#define ENABLE_YAW_CONTROL

/*==============================*/

/*================================
        GLOBAL ENUMS
================================*/
typedef enum 
{
  PITCH,
  ROLL,
  YAW
}IMU_typedef;

typedef enum 
{
  KP,
  KI,
  KD,
  KP_YAW,
  KI_YAW,
  KD_YAW
}PID_typedef;
/*==============================*/

/*================================
        GLOBAL VARIABLES
================================*/
/*==============================*/

/*================================
        GLOBAL FUNCTIONS
================================*/
/*==============================*/

#endif /*__GLOBAL__*/
