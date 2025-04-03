#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include <stdint.h>
#include <signal.h>
#include <sys/shm.h>
#include <sys/stat.h>

//gcc -o week1 week_1_student.cpp -lwiringPi  -lm


int setup_imu();
void calibrate_imu();      
void read_imu();    
void update_filter();

//global variables
int accel_address,gyro_address;
float x_gyro_calibration=0;
float y_gyro_calibration=0;
float z_gyro_calibration=0;
float roll_calibration=0;
float pitch_calibration=0;
float accel_z_calibration=0;
float imu_data[6]; //accel xyz,  gyro xyz, 
long time_curr;
long time_prev;
struct timespec te;
float yaw=0;
float pitch_angle=0;
float roll_angle=0;

 
int main (int argc, char *argv[])
{

    setup_imu();
    calibrate_imu();    
    
    while(1)
    {
      read_imu();   
      //printf("%10.5f %10.5f %10.5f %10.5f %10.5f %10.5f \n", imu_data[0], imu_data[1], imu_data[2], imu_data[3], imu_data[4], imu_data[5]);
      printf("gyro x: %10.5f, gyro y: %10.5f, gyro z: %10.5f, roll: %10.5f, pitch: %10.5f \n", imu_data[3], imu_data[4], imu_data[5], roll_angle, pitch_angle);
      //printf("%f %f %f %f %f %f \n", imu_data[0], imu_data[1], imu_data[2], imu_data[3], imu_data[4], imu_data[5]);
      
    }
  
}

void calibrate_imu()
{
  float x_run = 0;
  float y_run = 0;
  float z_run = 0;
  float r_run = 0;
  float p_run = 0;

  for (int i=0; i<1000; i++) {
    read_imu();
    x_run = x_run + imu_data[3];
    y_run = y_run + imu_data[4];
    z_run = z_run + imu_data[5];
    p_run = p_run + pitch_angle;
    r_run = r_run + roll_angle;
  }
  x_gyro_calibration=x_run / 1000;
  y_gyro_calibration=y_run / 1000;
  z_gyro_calibration=z_run / 1000;
  roll_calibration= r_run / 1000;
  pitch_calibration= p_run / 1000;
  //accel_z_calibration=??
  
printf("calibration complete, %f %f %f %f %f %f\n\r",x_gyro_calibration,y_gyro_calibration,z_gyro_calibration,roll_calibration,pitch_calibration,accel_z_calibration);


}

void read_imu()
{
  uint8_t address=0;//todo: set address value for accel x value 
  float ax=0;
  float az=0;
  float ay=0; 
  int vh=0;
  int vl=0;
  int vw=0;


  //accel reads

  address=0x12;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(accel_address,address);  
   
  //convert from 2's complement
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  // printf("%d, %f, %f \n", vw, (float) vw, (float) vw / 32767 * 3);

  imu_data[0]=(float) vw / 32767 * 3;//convert to g's  
  
  address=0x14;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(accel_address,address);  
  //convert from 2's complement
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[1]=(float) vw / 32767 * 3;//convert to g's  
  
  address=0x16;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(accel_address,address);   
  //convert from 2's complement     
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[2]=(float) vw / 32767 * 3;//convert to g's  
  
  
     

  //gyro reads

  address=0x02;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(gyro_address,address);   
  //convert from 2's complement          
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[3]=(float) vw / 32767 * 1000 - x_gyro_calibration;//convert to degrees/sec
  
  address=0x04;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(gyro_address,address);    
  //convert from 2's complement              
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[4]=(float) vw / 32767 * 1000 - y_gyro_calibration;//convert to degrees/sec
  
  address=0x06;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(gyro_address,address);   
  //convert from 2's complement               
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[5]=-((float) vw / 32767 * 1000) - z_gyro_calibration;//convert to degrees/sec  
  
  roll_angle = atan2(imu_data[2], imu_data[0]) / M_PI * 180 - roll_calibration;
  pitch_angle = atan2(imu_data[1], imu_data[0]) / M_PI * 180 - pitch_calibration;


}


int setup_imu()
{
  wiringPiSetup ();
  
  //setup imu on I2C
  accel_address=wiringPiI2CSetup (0x19) ; 
  
  
  gyro_address=wiringPiI2CSetup (0x69) ; 
  
  if(accel_address==-1)
  {
    printf("-----cant connect to accel I2C device %d --------\n",accel_address);
    return -1;
  }
  else if(gyro_address==-1)
  {
    printf("-----cant connect to gyro I2C device %d --------\n",gyro_address);
    return -1;
  }
  else
  {
    printf("all i2c devices detected\n");
    sleep(1);
    wiringPiI2CWriteReg8(accel_address, 0x7d, 0x04); //power on accel    
    wiringPiI2CWriteReg8(accel_address, 0x41, 0x00); //accel range to +_3g    
    wiringPiI2CWriteReg8(accel_address, 0x40, 0x89); //high speed filtered accel
    
    wiringPiI2CWriteReg8(gyro_address, 0x11, 0x00);//power on gyro
    wiringPiI2CWriteReg8(gyro_address, 0x0F, 0x01);//set gyro to +-1000dps
    wiringPiI2CWriteReg8(gyro_address, 0x10, 0x02);//set data rate and bandwith
    
    
    sleep(1);
  }
  return 0;
}


