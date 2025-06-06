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

#define GYRO_RATE_MAX 1200 // deg/s //600
#define ROLL_ANGLE_MAX 45 // deg
#define PITCH_ANGLE_MAX 45 // deg
#define JOYSTICK_TIMEOUT 0.7 // s

//gcc -o week1 week_1_student.cpp -lwiringPi  -lm


int setup_imu();
void calibrate_imu();   
void calibrate_tag();   
void read_imu();    
void update_filter();
void setup_joystick();
void trap(int signal);
void safety_check();
void motor_enable();
void set_motors(int motor0, int motor1, int motor2, int motor3);
void compute_motors();
void setup_camera();

//global variables
int accel_address,gyro_address, motor_address;
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


int motor_commands[4];
float thrust_neutral = 1450; //1450 for ground effect
float thrust_amplitude = 500; //200 for ground effect
float thrust_min = 250;
float thrust_max = 2000;
float pitch_amplitude = 6; //was 4
float pitch_desired = 0;
float roll_amplitude = 6; //was 4
float roll_desired = 0;
float thrust = 100;
float thrust_joystick = 0;

float yaw_amplitude = 80;
float yaw_desired = 0; //this is a rate
float yaw_cam_desired = 0;

float p_p_gain = 20;   // 12
float p_d_gain = 2;    // 2
float p_i_gain = 0.3;  // 0.3
float int_p_error = 0;
float int_p_error_sat = 500;
float r_p_gain = 20;
float r_d_gain = 2; 
float r_i_gain = 0.3;
float int_r_error = 0;
float int_r_error_sat = 500;
float y_p_gain = 6;
float y_cam_p_gain = 1.5;

float auto_thrust = 0;
float thrust_weight = 0;
float thrust_p_gain = 10;
float thrust_d_gain = 2;
float thrust_i_gain = 0.3;
float thrust_int_error = 0;
float thrust_int_error_sat = 100;

float camera_z_desired = -1500;
float camera_x_desired = 0;
float camera_y_desired = 0;
float camera_z_last = 0;
float camera_x_last = 0;
float camera_y_last = 0;
float camera_x_ema = 0;
float camera_y_ema = 0;
float camera_z_ema = 0;

float auto_pitch = 0;
float auto_pitch_p_gain = 0.0095;  // 0.01
float auto_pitch_d_gain = 0.002; // 0.002


float auto_roll = 0;
float auto_roll_p_gain = 0.0075;  // 0.008
float auto_roll_d_gain = 0.002; // 0.0015

float auto_yaw = 0;

int camera_sequence_last = 0;
long camera_time_last = 0;

float p_bias = 0; //deg
float r_bias = 0; //deg
float y_bias = 0; //deg/s

bool flag_print = false;
bool is_paused = true;
bool autonomy_flag = false;
int previous_x;


struct Joystick
{
  int key0;
  int key1;     
  int key2;
  int key3;
  int pitch;  // joystick down is positive, up is negative
  int roll;   // joystick left is negative, right is positive
  int yaw;    // joystick left is positive, right is negative
  int thrust;
  int sequence_num;
};

Joystick* shared_memory;
Joystick joystick_data; 

int run_program=1;

struct Camera
{
  int x;
  int y;
  int z;
  int yaw;
  int sequence_num;
};

Camera* camera_memory; 
Camera camera_data;


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
    flag_print = true;
    sequence_num_last = joystick_data.sequence_num;
  }
  else {
    time_since_last += time_diff;
    flag_print = false;
  }
  
  if (time_since_last > JOYSTICK_TIMEOUT) {
  
    run_program = 0;
    printf("Timeout. \n");
    
  }
  
  //check joystick b
  
  
  if (joystick_data.key1 == 1) {
    run_program = 0;
    //printf("B pressed. \n");
  }
  

  // rate and angle maxes
  if (imu_data[3] > GYRO_RATE_MAX || imu_data[4] > GYRO_RATE_MAX || imu_data[5] > GYRO_RATE_MAX) 
  {
    run_program = 0;
    printf("Gyro rate max exceeded. \n");
  }
  
  if (roll_angle > ROLL_ANGLE_MAX) 
  {
    run_program = 0;
    printf("Roll angle max exceeded. \n");
  }
  
  if (roll_angle < -ROLL_ANGLE_MAX) {
    run_program = 0;
    printf("Roll angle min exceeded. \n");
  }
  
  if (pitch_angle > PITCH_ANGLE_MAX) 
  {
    run_program = 0;
    printf("Pitch angle max exceeded. \n");
  }
  
  if (pitch_angle < - PITCH_ANGLE_MAX) {
    run_program = 0;
    printf("Pitch angle min exceeded. \n");
  }
  
}

void calibrate_tag() {
  camera_x_desired = camera_x_ema;
  camera_y_desired = camera_y_ema;
  printf("calibrated tag center to x: %10.5f y: %10.5f \n", camera_x_desired, camera_y_desired);
}


// motor0 is bottom right, motor1 is top right, motor2 is bottom left, motor 3 is top left
void set_motors(int motor0, int motor1, int motor2, int motor3)
{

    if(motor0<0)
      motor0=thrust_min;
    if(motor0>thrust_max)
      motor0=thrust_max;
    if(motor1<0)
      motor1=thrust_min;
    if(motor1>thrust_max)
      motor1=thrust_max;
    if(motor2<0)
      motor2=thrust_min;
    if(motor2>thrust_max)
      motor2=thrust_max;
    if(motor3<0)
      motor3=thrust_min;
    if(motor3>thrust_max)
      motor3=thrust_max;
      
    
    
    uint8_t motor_id=0;
    uint8_t special_command=0;
    uint16_t commanded_speed_0=1000;    
    uint16_t commanded_speed_1=0;
    uint16_t commanded_speed=0;
    uint8_t data[2]; 
    
   // wiringPiI2CWriteReg8(motor_address, 0x00,data[0] );
    //wiringPiI2CWrite (motor_address,data[0]) ;
    int com_delay=500;
   
    motor_id=0;
    commanded_speed=motor0;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
    data[1]=commanded_speed&0x7f;    
    wiringPiI2CWrite(motor_address,data[0]);     
    usleep(com_delay);    
    wiringPiI2CWrite(motor_address,data[1]);  
 
    
    usleep(com_delay);   
    motor_id=1;
    commanded_speed=motor1;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
    data[1]=commanded_speed&0x7f;    
    wiringPiI2CWrite(motor_address,data[0]);     
    usleep(com_delay);    
    wiringPiI2CWrite(motor_address,data[1]);      
  
    usleep(com_delay); 
    motor_id=2;
    commanded_speed=motor2;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
    data[1]=commanded_speed&0x7f;    
    wiringPiI2CWrite(motor_address,data[0]);     
    usleep(com_delay);    
    wiringPiI2CWrite(motor_address,data[1]);   

    
    usleep(com_delay);   
    motor_id=3;
    commanded_speed=motor3;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
    data[1]=commanded_speed&0x7f;    
    wiringPiI2CWrite(motor_address,data[0]);     
    usleep(com_delay);    
    wiringPiI2CWrite(motor_address,data[1]);    
    usleep(com_delay);


}

void compute_motors() {


  //auto thrust, pitch, roll computation happens here
  if (camera_data.sequence_num != camera_sequence_last) {
    // box in front means x is negative, box to the right means y is positive
  
    //get current time in nanoseconds
    timespec_get(&te,TIME_UTC);
    float camera_time_curr=te.tv_nsec;
    float camera_time_diff=camera_time_curr-camera_time_last;
    if (camera_time_diff<=0)
    {
      camera_time_diff+=1000000000;
    }
    camera_time_diff=camera_time_diff/1000000000;
    camera_time_last = camera_time_curr;
    
    //computation of auto thrust
    thrust_int_error += ((float) camera_z_desired - (float) camera_data.z) * thrust_i_gain;
    thrust_int_error = std::min(std::max(thrust_int_error, (float) - thrust_int_error_sat), (float) thrust_int_error_sat);
    
    camera_z_ema = 0.9*camera_z_ema + 0.1*camera_data.z;
    camera_x_ema = 0.9*camera_x_ema + 0.1*camera_data.x;
    camera_y_ema = 0.9*camera_y_ema + 0.1*camera_data.y;  
    auto_thrust = (camera_z_desired - camera_z_ema)*thrust_p_gain - (camera_z_ema - camera_z_last)/camera_time_diff*thrust_d_gain + thrust_int_error;
    
    
    //computation of auto pitch
    auto_pitch = -(camera_x_desired - camera_x_ema)*auto_pitch_p_gain + (camera_x_ema - camera_x_last)/camera_time_diff*auto_pitch_d_gain;
    
    //computation of auto roll
    auto_roll = -(camera_y_desired - camera_y_ema)*auto_roll_p_gain + (camera_y_ema - camera_y_last)/camera_time_diff*auto_roll_d_gain;
    
    auto_yaw = -(camera_data.yaw * y_cam_p_gain);
    
    camera_x_last = camera_x_ema;
    camera_y_last = camera_y_ema;
    camera_z_last = camera_z_ema;
    
  }  
  
  thrust_joystick = thrust_neutral + (128 - (float) joystick_data.thrust) / 128 * thrust_amplitude;  
  
  if (thrust_joystick < 1450.0) {
      auto_pitch = 0;
      auto_roll = 0;
      auto_yaw = 0;
      auto_thrust = 0;
    }

  if (autonomy_flag) {
    yaw_desired = auto_yaw;
    pitch_desired = auto_pitch;
    roll_desired = auto_roll;
    thrust = (1-thrust_weight)*thrust_joystick + thrust_weight*auto_thrust;
  }
  else {
    yaw_desired = (128 - (float) joystick_data.yaw) / 128 * yaw_amplitude + y_bias;
    pitch_desired = - (128 - (float) joystick_data.pitch) / 128 * pitch_amplitude + p_bias;
    roll_desired = - (128 - (float) joystick_data.roll) / 128 * roll_amplitude + r_bias;
    thrust = thrust_joystick;
  }
  // yaw_desired = (128 - (float) joystick_data.yaw) / 128 * yaw_amplitude + y_bias; // uncomment this to force manual yaw
  
  float p_error = pitch_desired - pitch_filtered;
  int_p_error = int_p_error + p_i_gain * p_error;
  int_p_error = std::min(std::max(int_p_error, (float) -int_p_error_sat), (float) int_p_error_sat);
  float y_error = yaw_desired - imu_data[3];
  float r_error = roll_desired - roll_filtered;
  int_r_error = int_r_error + r_i_gain * r_error;
  int_r_error = std::min(std::max(int_r_error, (float) -int_r_error_sat), (float) int_r_error_sat);

  
  motor_commands[3] = (int) thrust + (int) (p_p_gain * p_error) - (int) (p_d_gain * imu_data[5]) + (int) int_p_error - y_p_gain*y_error
    + (int) (r_p_gain * r_error) - (int) (r_d_gain * imu_data[4]) + (int) int_r_error;
  motor_commands[2] = (int) thrust - (int) (p_p_gain * p_error) + (int) (p_d_gain * imu_data[5]) - (int) int_p_error + y_p_gain*y_error
    + (int) (r_p_gain * r_error) - (int) (r_d_gain * imu_data[4]) + (int) int_r_error;
  motor_commands[1] = (int) thrust + (int) (p_p_gain * p_error) - (int) (p_d_gain * imu_data[5]) + (int) int_p_error + y_p_gain*y_error
    - (int) (r_p_gain * r_error) + (int) (r_d_gain * imu_data[4]) - (int) int_r_error;
  motor_commands[0] = (int) thrust - (int) (p_p_gain * p_error) + (int) (p_d_gain * imu_data[5]) - (int) int_p_error - y_p_gain*y_error
    - (int) (r_p_gain * r_error) + (int) (r_d_gain * imu_data[4]) - (int) int_r_error;
  
  // printf("int_p_error %10.5f, thrust %10.5f, front %d, back %d \n", int_p_error, thrust, motor_commands[0], motor_commands[1]);
  
}
 
int main (int argc, char *argv[])
{

    setup_imu();
    setup_camera();
    motor_address=wiringPiI2CSetup(0x56); 
    calibrate_imu(); 
    motor_enable();
    setup_joystick();
    signal(SIGINT, &trap);
    
    int cycle = 3;
    int counter = 0;

    
    while(run_program == 1)
    {
      camera_data=*camera_memory;
      // printf("cam=%d %d %d %d %d, joy=%10.5f, auto=%10.5f, pitch=%8.5f, roll=%8.5f,  %d, %d, %d, %d \n",camera_data.x,camera_data.y,camera_data.z,camera_data.yaw,camera_data.sequence_num, thrust_joystick, auto_thrust, auto_pitch, auto_roll, motor_commands[0], motor_commands[1], motor_commands[2], motor_commands[3]);
      // printf("x raw: %d, x ema: %10.5f, y raw: %d, y ema: %10.5f, z raw: %d, z ema: %10.5f, seq: %d \n", camera_data.x, camera_x_ema, camera_data.y, camera_y_ema, camera_data.z, camera_z_ema, camera_data.sequence_num);
      printf("pitch e: %8.5f, c: %8.5f deg, roll e: %8.5f, c: %8.5f deg, yaw e: %d, z e: %8.5f, c: %8.5f\n", (camera_x_desired - camera_x_ema), auto_pitch, (camera_y_desired - camera_y_ema), auto_roll, -camera_data.yaw, (camera_z_desired - camera_z_ema), auto_thrust);

      
      if (joystick_data.key0 == 1) {  // press Y
        is_paused = true;
      }
      if (joystick_data.key3 == 1) {  // press A
        is_paused = false;
      }
      if (joystick_data.key2 == 1) {  // press X
        if (previous_x== 0) {
          autonomy_flag = !autonomy_flag;
          printf("flipped autonomy flag to %b\n", autonomy_flag); 
          if (autonomy_flag) {
            calibrate_tag();
          }
        }
        previous_x = 1;
      }
      else {
      
        previous_x = 0;
        
      }
      // printf("%d, %d, %d, %d\n", motor_commands[0], motor_commands[1], motor_commands[2], motor_commands[3]);

      read_imu();   
      update_filter(); 
      safety_check();
      compute_motors();
      if (is_paused) {
        set_motors(250,250,250,250);
      }
      else {
        set_motors(motor_commands[0],motor_commands[1],motor_commands[2],motor_commands[3]); 
      }
     
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

void setup_camera()
{

  int segment_id;   
  struct shmid_ds shmbuffer; 
  int segment_size; 
  const int shared_segment_size = sizeof(struct Camera);
  int smhkey=123456;
  
  /* Allocate a shared memory segment.  */ 
  segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666); 
  /* Attach the shared memory segment.  */ 
  camera_memory = (Camera*) shmat (segment_id, 0, 0); 
  printf ("shared memory attached at address %p\n", camera_memory); 
  /* Determine the segment's size. */ 
  shmctl (segment_id, IPC_STAT, &shmbuffer); 
  segment_size  =               shmbuffer.shm_segsz; 
  printf ("segment size: %d\n", segment_size); 
  /* Write a string to the shared memory segment.  */ 
  // sprintf (shared_memory, "test!!!!."); 

}

void motor_enable()
{
  
    uint8_t motor_id=0;
    uint8_t special_command=0;
    uint16_t commanded_speed_0=1000;    
    uint16_t commanded_speed_1=0;
    uint16_t commanded_speed=0;
    uint8_t data[2]; 
    
    int cal_delay=50;
    
    for(int i=0;i<1000;i++)
    {
    
      motor_id=0;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]); 
      
      
      usleep(cal_delay);   
      motor_id=1;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);      
      
      usleep(cal_delay); 
      motor_id=2;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);   
   
      
      usleep(cal_delay);   
      motor_id=3;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);       
      usleep(cal_delay);

    }
     
    for(int i=0;i<2000;i++)
    {
    
      motor_id=0;
      commanded_speed=50;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]); 
      
      
      usleep(cal_delay);   
      motor_id=1;
      commanded_speed=50;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);      
      
      usleep(cal_delay); 
      motor_id=2;
      commanded_speed=50;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);   
   
      
      usleep(cal_delay);   
      motor_id=3;
      commanded_speed=50;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);       
      usleep(cal_delay);

    }
    
     
    for(int i=0;i<500;i++)
    {
    
      motor_id=0;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]); 
      
      
      usleep(cal_delay);   
      motor_id=1;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);      
      
      usleep(cal_delay); 
      motor_id=2;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);   
   
      
      usleep(cal_delay);   
      motor_id=3;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);       
      usleep(cal_delay);

    }

}

// catch ctrl-c
void trap(int signal)
{
   printf("\nending program\n\r");
   run_program=0;
}
