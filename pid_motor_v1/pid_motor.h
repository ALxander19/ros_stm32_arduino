#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#define ENCODER_PUBLISH_FREQUENCY        100   //hz
#define FILTER_PUBLISH_FREQUENCY         100   //hz

#define encaR  19   // Yellow cable
#define encbR  18   // White cable
#define inaR   6
#define inbR   7

#define kp_R   60
#define kd_R   12
#define ki_R   1.5


volatile long encoderRPos=0;
float newpositionR;
float oldpositionR = 0;
float newtimeR;
float oldtimeR = 0;
float wR_round;
float wR;

int wR_control;
float error_R = 0;
float prev_errorR = 0;
float integral_R = 0;
float derivative_R = 0;
int old_timeR = 0;
int new_timeR;

float velfil = 0;

static uint32_t tTime[4];


/********* Subscribers *********/

void w_right_cb(const std_msgs::Float32& w_right_msg);
ros::Subscriber<std_msgs::Float32> w_right("w_right", w_right_cb);

/********* Publishers *********/

std_msgs::Float32 vel_right_msg;
ros::Publisher vel_motor_right("vel_motor_right", &vel_right_msg);

std_msgs::Float32 vel_fil_msg;
ros::Publisher vel_fil_right("vel_fil_right", &vel_fil_msg);
