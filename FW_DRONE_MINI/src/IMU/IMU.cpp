
#include "IMU.h"

static MPU6050 mpu;
static IMU_DATA_TYPEDEF prevOrientation;
static IMU_DATA_TYPEDEF Angle;
static Quaternion q;           
static VectorFloat gravity;    
static float ypr[3];  
static bool dmpReady = false; 
static uint8_t mpuIntStatus=0;   
static uint16_t packetSize=0;
static uint8_t fifoBuffer[64];

static IMU_DATA_TYPEDEF getIMUOrientation(void) 
{
  if (!dmpReady || !mpu.testConnection()) 
  {
    IMU_DATA_TYPEDEF o;
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

    IMU_DATA_TYPEDEF o;
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

void setupMPU(void) 
{
  mpu.initialize();
  Serial1.println(mpu.testConnection());

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

IMU_DATA_TYPEDEF GetIMUValue(void)
{
  return getIMUOrientation();
}

float fix360degrees(float val) 
{
  if (val > 180) 
  {
    return val - 360;
  } 
  else if (val < -180) 
  {
    return val + 360;
  } 
  else 
  {
    return val;
  }
}
