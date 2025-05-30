
#ifndef __VERSION_FIRMWARE__
#define __VERSION_FIRMWARE__

/*================================================
                VERSION FIRMWARE
================================================*/

  /***********************************************
   * Project: Drone Mini    
   * Version: DMV.2.1
   * Date: 15,Mar,2025
   * Dev: NhanNguyen
   * HW-TX: V24OL, Flysky I6
   * HW-RX: V24.2, Flysky IA6B
   * Drone-HW: UAV MINI
   * Drone-HW-MCU: STM32F401CCU6
   * Debug tool: JLINK-SEGGER
   * 
   * *** Decription ***
   * Add feature: check save default PID value 
   * Update version check
   * Update somes API
   * Release new version to customer
   * 
   * - Fix ERR: 
   * 
  ************************************************/
 #ifndef FIRMWARE_VERSION
 #define FIRMWARE_VERSION            ("DMV.2.1")
 #endif /* FIRMWARE_VERSION */
 #ifndef FIRMWARE_VERSION_CHECK
 #define FIRMWARE_VERSION_CHECK      (21)
 #endif /* FIRMWARE_VERSION_CHECK */  
 #ifndef RELEASES_DATE
 #define RELEASES_DATE               ("15,Mar,2025")
 #endif /* RELEASES_DATE */

  /***********************************************
   * Project: Drone Mini    
   * Version: DMV.2.0
   * Date: 8,Feb,2025
   * Dev: NhanNguyen
   * HW-TX: V24OL, Flysky I6
   * HW-RX: V24.2, Flysky IA6B
   * Drone-HW: UAV MINI
   * Drone-HW-MCU: STM32F401CCU6
   * Debug tool: JLINK-SEGGER
   * 
   * *** Decription ***
   * Update: PID controller
   * keep PID for balance
   * Test with PITCH + ROLL + YAW
   * 
   * - Fix ERR: 
   * 
  ************************************************/
  #ifndef FIRMWARE_VERSION
  #define FIRMWARE_VERSION            ("DMV.2.0")
  #endif /* FIRMWARE_VERSION */
  #ifndef FIRMWARE_VERSION_CHECK
  #define FIRMWARE_VERSION_CHECK      (20)
  #endif /* FIRMWARE_VERSION_CHECK */  
  #ifndef RELEASES_DATE
  #define RELEASES_DATE               ("8,Feb,2025")
  #endif /* RELEASES_DATE */

  /***********************************************
   * Project: Drone Mini    
   * Version: DMV.1.9
   * Date: 1,Jan,2025
   * Dev: NhanNguyen
   * HW-TX: V24OL, Flysky I6
   * HW-RX: V24.2, Flysky IA6B
   * Drone-HW: UAV MINI
   * Drone-HW-MCU: STM32F401CCU6
   * Debug tool: JLINK-SEGGER
   * 
   * *** Decription ***
   * Yaw always use PID controller
   * Update: method control yaw use moment from motor
   * but still keep PID for balance
   * Test with PITCH + ROLL + YAW
   * 
   * - Fix ERR: 
   * 
  ************************************************/
  #ifndef FIRMWARE_VERSION
  #define FIRMWARE_VERSION            ("DMV.1.9")
  #endif /* FIRMWARE_VERSION */
  #ifndef FIRMWARE_VERSION_CHECK
  #define FIRMWARE_VERSION_CHECK      (19)
  #endif /* FIRMWARE_VERSION_CHECK */  
  #ifndef RELEASES_DATE
  #define RELEASES_DATE               ("1,Jan,2025")
  #endif /* RELEASES_DATE */

  /***********************************************
   * Project: Drone Mini    
   * Version: DMV.1.8
   * Date: 1,Jan,2025
   * Dev: NhanNguyen
   * HW-TX: V24OL, Flysky I6
   * HW-RX: V24.2, Flysky IA6B
   * Drone-HW: UAV MINI
   * Drone-HW-MCU: STM32F401CCU6
   * Debug tool: JLINK-SEGGER
   * 
   * *** Decription ***
   * Integral limit -500 -> 500
   * Add macro: YAW_GAINS_VALUE
   * Clear Pitch/Roll degree limit
   * Test flysky i6
   * Pitch, Roll ok, fix yaw controller
   * Update: method for convert CH4 value
   * to Yaw set value
   * 
   * - Fix ERR: 
   * 
   * 
  ************************************************/
  #ifndef FIRMWARE_VERSION
  #define FIRMWARE_VERSION            ("DMV.1.8")
  #endif /* FIRMWARE_VERSION */
  #ifndef FIRMWARE_VERSION_CHECK
  #define FIRMWARE_VERSION_CHECK      (18)
  #endif /* FIRMWARE_VERSION_CHECK */  
  #ifndef RELEASES_DATE
  #define RELEASES_DATE               ("1,Jan,2025")
  #endif /* RELEASES_DATE */

  /***********************************************
   * Project: Drone Mini    
   * Version: DMV.1.7
   * Date: 28,Dec,2024
   * Dev: NhanNguyen
   * HW-TX: V24OL, Flysky I6
   * HW-RX: V24.2, Flysky IA6B
   * Drone-HW: UAV MINI
   * Drone-HW-MCU: STM32F401CCU6
   * Debug tool: JLINK-SEGGER
   * 
   * *** Decription ***
   * Test PID for PITCH, ROLL, YAW
   * YAW control use PID
   * YAW fix PID controller
   * Update Yaw control value, test pass 
   * for pitch and roll
   * add macro STOP_FOR_TEST
   * 
   * - Fix ERR: 
   * 
   * 
  ************************************************/
  #ifndef FIRMWARE_VERSION
  #define FIRMWARE_VERSION            ("DMV.1.7")
  #endif /* FIRMWARE_VERSION */
  #ifndef FIRMWARE_VERSION_CHECK
  #define FIRMWARE_VERSION_CHECK      (17)
  #endif /* FIRMWARE_VERSION_CHECK */  
  #ifndef RELEASES_DATE
  #define RELEASES_DATE               ("28,Dec,2024")
  #endif /* RELEASES_DATE */

  /***********************************************
   * Project: Drone Mini    
   * Version: DMV.1.6
   * Date: 22,Nov,2024
   * Dev: NhanNguyen
   * HW-TX: V24OL, Flysky I6
   * HW-RX: V24.2, Flysky IA6B
   * Drone-HW: UAV MINI
   * Drone-HW-MCU: STM32F401CCU6
   * Debug tool: JLINK-SEGGER
   * 
   * *** Decription ***
   * Test PID for PITCH, ROLL, YAW
   * YAW control use PID
   * YAW fix PID controller
   * 
   * - Fix ERR: 
   * 
   * 
  ************************************************/
  #ifndef FIRMWARE_VERSION
  #define FIRMWARE_VERSION            ("DMV.1.6")
  #endif /* FIRMWARE_VERSION */
  #ifndef FIRMWARE_VERSION_CHECK
  #define FIRMWARE_VERSION_CHECK      (16)
  #endif /* FIRMWARE_VERSION_CHECK */  
  #ifndef RELEASES_DATE
  #define RELEASES_DATE               ("22,Nov,2024")
  #endif /* RELEASES_DATE */

  /***********************************************
   * Project: Drone Mini    
   * Version: DMV.1.5
   * Date: 12,Nov,2024
   * Dev: NhanNguyen
   * HW-TX: V24OL, Flysky I6
   * HW-RX: V24.2, Flysky IA6B
   * Drone-HW: UAV MINI
   * Drone-HW-MCU: STM32F401CCU6
   * Debug tool: JLINK-SEGGER
   * 
   * *** Decription ***
   * Test PID for PITCH and ROLL
   * YAW control use motor torque
   * 
   * - Fix ERR: 
   * 
   * 
  ************************************************/
  #ifndef FIRMWARE_VERSION
  #define FIRMWARE_VERSION            ("DMV.1.5")
  #endif /* FIRMWARE_VERSION */
  #ifndef FIRMWARE_VERSION_CHECK
  #define FIRMWARE_VERSION_CHECK      (15)
  #endif /* FIRMWARE_VERSION_CHECK */  
  #ifndef RELEASES_DATE
  #define RELEASES_DATE               ("12,Nov,2024")
  #endif /* RELEASES_DATE */

  /***********************************************
   * Project: Drone Mini    
   * Version: DMV.1.4
   * Date: 12,Nov,2024
   * Dev: NhanNguyen
   * HW-TX: V24OL, Flysky I6
   * HW-RX: V24.2, Flysky IA6B
   * Drone-HW: UAV MINI
   * Drone-HW-MCU: STM32F401CCU6
   * Debug tool: JLINK-SEGGER
   * 
   * *** Decription ***
   * Test PID for PITCH and ROLL
   * YAW control use PID
   * 
   * - Fix ERR: 
   * 
   * 
  ************************************************/
  #ifndef FIRMWARE_VERSION
  #define FIRMWARE_VERSION            ("DMV.1.4")
  #endif /* FIRMWARE_VERSION */
  #ifndef FIRMWARE_VERSION_CHECK
  #define FIRMWARE_VERSION_CHECK      (14)
  #endif /* FIRMWARE_VERSION_CHECK */  
  #ifndef RELEASES_DATE
  #define RELEASES_DATE               ("12,Nov,2024")
  #endif /* RELEASES_DATE */

  /***********************************************
   * Project: Drone Mini    
   * Version: DMV.1.3
   * Date: 10,Nov,2024
   * Dev: NhanNguyen
   * HW-TX: V24OL, Flysky I6
   * HW-RX: V24.2, Flysky IA6B
   * Drone-HW: UAV MINI
   * Drone-HW-MCU: STM32F401CCU6
   * Debug tool: JLINK-SEGGER
   * 
   * *** Decription ***
   * Dev PID control: PITCH and ROLL
   * Test control: PITCH and ROLL
   * IMU add function check YAW value
   * Dev PID for YAW
   * Update version type
   * 
   * - Fix ERR: 
   * 
   * 
  ************************************************/
  #ifndef FIRMWARE_VERSION
  #define FIRMWARE_VERSION            ("DMV.1.3")
  #endif /* FIRMWARE_VERSION */
  #ifndef FIRMWARE_VERSION_CHECK
  #define FIRMWARE_VERSION_CHECK      (13)
  #endif /* FIRMWARE_VERSION_CHECK */  
  #ifndef RELEASES_DATE
  #define RELEASES_DATE               ("10,Nov,2024")
  #endif /* RELEASES_DATE */

  /***********************************************
   * Project: Drone Mini    
   * Version: DMV.0.2
   * Date: 9,Nov,2024
   * Dev: NhanNguyen
   * HW-TX: V24OL, Flysky I6
   * HW-RX: V24.2, Flysky IA6B
   * Drone-HW: UAV MINI
   * Drone-HW-MCU: STM32F401CCU6
   * Debug tool: JLINK-SEGGER
   * 
   * *** Decription ***
   * Update lib kalman filter
   * Add folder PID
   * Dev PID control: pitch and roll
   * Release menu
   * Fix day release in version DMV.0.1
   * 
   * - Fix ERR: 
   * 
   * 
  ************************************************/
  #ifndef FIRMWARE_VERSION
  #define FIRMWARE_VERSION      ("DMV.0.2")
  #endif /* FIRMWARE_VERSION */
  #ifndef RELEASES_DATE
  #define RELEASES_DATE         ("9,Nov,2024")
  #endif /* RELEASES_DATE */

  /***********************************************
   * Project: Drone Mini    
   * Version: DMV.0.1
   * Date: 9,Nov,2024
   * Dev: NhanNguyen
   * HW-TX: V24OL, Flysky I6
   * HW-RX: V24.2, Flysky IA6B
   * Drone-HW: UAV MINI
   * Drone-HW-MCU: STM32F401CCU6
   * Debug tool: JLINK-SEGGER
   * 
   * *** Decription ***
   * First version release
   * Create licence for user
   * Define folder structure
   * Test hardware
   * Read data from receiver
   * Read MPU6050
   * Control LCD OLED 0.91inch I2C
   * Control 4 ESC for 4 MOTOR
   * Dev menu basic
   * 
   * - Fix ERR: 
   * 
   * 
  ************************************************/
  #ifndef FIRMWARE_VERSION
  #define FIRMWARE_VERSION      ("DMV.0.1")
  #endif /* FIRMWARE_VERSION */
  #ifndef RELEASES_DATE
  #define RELEASES_DATE         ("9,Nov,2024")
  #endif /* RELEASES_DATE */

/*==============================================*/

#endif /*__VERSION_FIRMWARE__*/
