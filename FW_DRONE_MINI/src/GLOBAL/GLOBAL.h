
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
#define MILLIS (millis())
#define MICROS (micros())
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

#define PID_CONTROL_VALUE_LIMIT (50)

#define PITCH_CONTROL_LIMIT (20)
#define ROLL_CONTROL_LIMIT  (20)

/* Degree */
#define YAW_CONTROL_LIMIT   (30)

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
