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

#define GYRO_RATE_MAX 300 // deg/s
#define ROLL_ANGLE_MAX 45 // deg
#define PITCH_ANGLE_MAX 45 // deg
#define JOYSTICK_TIMEOUT 0.35 // s

//gcc -o week1 week_1_student.cpp -lwiringPi  -lm


int setup_imu();
void calibrate_imu();      
void read_imu();    
void update_filter();
void setup_joystick();
void trap(int signal);
void safety_check();

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
long time_curr_2;
long time_prev_2;
struct timespec te;
float yaw=0;
float pitch_angle=0;
float roll_angle=0;

float A = 0.02;
float roll_filtered=0;
float pitch_filtered=0;

float roll_gyro_integral = 0;
float pitch_gyro_integral = 0;

float sequence_num_last = 0;
float time_since_last = 0;

struct Joystick
{
  int key0;
  int key1;
  int key2;
  int key3;
  int pitch;
  int roll;
  int yaw;
  int thrust;
  int sequence_num;
};

Joystick* shared_memory;
Joystick joystick_data; 


int run_program=1;

void safety_check() 
{
  joystick_data = *shared_memory;
  
  float time_diff=time_curr-time_prev; 
  
  //get current time in nanoseconds
  timespec_get(&te,TIME_UTC);
  time_curr_2=te.tv_nsec; 
  time_diff = time_curr_2-time_prev_2;  
  
  time_prev_2=time_curr_2;       
  
  // check for rollover
  if(time_diff<=0)
  {
    time_diff+=1000000000;
  }
  // convert to seconds
  time_diff=time_diff/1000000000;
  
  // determine if timeout
  if (joystick_data.sequence_num != sequence_num_last) 
  {
    time_since_last = 0;
    sequence_num_last = joystick_data.sequence_num;
  }
  else {
    time_since_last += time_diff;
  }
  
  if (time_since_last > JOYSTICK_TIMEOUT) {
  
    run_program = 0;
    printf("Timeout. \n");
    
  }
  
  //check joystick b
  
  printf("%d %d %d %d %d %d %d %d %d \n", joystick_data.key0, joystick_data.key1, joystick_data.key2, joystick_data.key3, joystick_data.pitch, joystick_data.roll, joystick_data.yaw, joystick_data.thrust, joystick_data.sequence_num);  
  
  if (joystick_data.key1 == 1) {
    run_program = 0;
    printf("B pressed. \n");
  }
  

  // rate and angle maxes
  if (imu_data[3] > GYRO_RATE_MAX || imu_data[4] > GYRO_RATE_MAX || imu_data[5] > GYRO_RATE_MAX) 
  {
    run_program = 0;
    printf("Gyro rate max exceeded.\n");
  }
  
  if (roll_angle > ROLL_ANGLE_MAX || roll_angle < -ROLL_ANGLE_MAX) 
  {
    run_program = 0;
    printf("Roll angle max exceeded. \n");
  }
  
  if (pitch_angle > PITCH_ANGLE_MAX || pitch_angle < - PITCH_ANGLE_MAX) 
  {
    run_program = 0;
    printf("Pitch angle max exceeded. \n");
  }
  
}
 
int main (int argc, char *argv[])
{

    setup_imu();
    calibrate_imu(); 
    setup_joystick();
    signal(SIGINT, &trap);
      
    
    while(run_program == 1)
    {
      read_imu();   
      update_filter(); 
      safety_check();
      
      //printf("%10.5f %10.5f %10.5f %10.5f %10.5f %10.5f \n", imu_data[0], imu_data[1], imu_data[2], imu_data[3], imu_data[4], imu_data[5]);
      
      //printf("gyro x: %10.5f, gyro y: %10.5f, gyro z: %10.5f, roll: %10.5f, pitch: %10.5f \n", imu_data[3], imu_data[4], imu_data[5], roll_angle, pitch_angle);
      
      //printf("%f %f %f %f %f %f \n", imu_data[0], imu_data[1], imu_data[2], imu_data[3], imu_data[4], imu_data[5]);
      
      //printf("accel roll: %10.5f, filtered roll: %10.5f, gyro y: %10.5f  \n", pitch_angle, pitch_filtered, imu_data[5]); 
      
      //printf("%10.5f, %10.5f, %10.5f\n", pitch_angle, pitch_gyro_integral, pitch_filtered);
      
    }
    
    return 0;
  
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
  /*if (roll_angle > 180) {
    roll_angle = roll_angle - 360;
  }
  if (roll_angle < -180) {
    roll_angle = roll_angle + 360;
  }
  */
  pitch_angle = atan2(imu_data[1], imu_data[0]) / M_PI * 180 - pitch_calibration;
 

}

void update_filter()
{

  //get current time in nanoseconds
  timespec_get(&te,TIME_UTC);
  time_curr=te.tv_nsec;
  //compute time since last execution
  float imu_diff=time_curr-time_prev;           
  
  //check for rollover
  if(imu_diff<=0)
  {
    imu_diff+=1000000000;
  }
  //convert to seconds
  imu_diff=imu_diff/1000000000;
  time_prev=time_curr;
  
  roll_filtered = roll_angle * A + (1-A)*(imu_data[4]*imu_diff+roll_filtered);
  roll_gyro_integral += imu_data[4]*imu_diff;
  
  pitch_filtered = pitch_angle * A + (1-A)*(imu_data[5]*imu_diff+pitch_filtered);
  pitch_gyro_integral += imu_data[5]*imu_diff;
   
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

void setup_joystick()
{

  int segment_id;   
  struct shmid_ds shmbuffer; 
  int segment_size; 
  const int shared_segment_size = 0x6400; 
  int smhkey=33222;
  
  /* Allocate a shared memory segment.  */ 
  segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666); 
  /* Attach the shared memory segment.  */ 
  shared_memory = (Joystick*) shmat (segment_id, 0, 0); 
  printf ("shared memory attached at address %p\n", shared_memory); 
  /* Determine the segment's size. */ 
  shmctl (segment_id, IPC_STAT, &shmbuffer); 
  segment_size  =               shmbuffer.shm_segsz; 
  printf ("segment size: %d\n", segment_size); 
  /* Write a string to the shared memory segment.  */ 
  //sprintf (shared_memory, "test!!!!."); 

}

// catch ctrl-c
void trap(int signal)
{
   printf("\nending program\n\r");
   run_program=0;
}
