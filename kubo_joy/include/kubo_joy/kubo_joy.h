#include "ros/ros.h"  //ROS 기본 헤더파일
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include "iostream"

#define MAX_F_FLIPPER 130
#define MIN_F_FLIPPER -90
#define MAX_B_FLIPPER 130
#define MIN_B_FLIPPER -90

#define wheel_diameter 225 //mm
#define wheels_x_distance_ 304
#define wheels_y_distance_ 495

#define PI 3.1415926535

#define MAX_VEL 0.3
#define MAX_ANG 0.5
#define FLIPPER_SPEED_GAIN 0.01

typedef struct RPM
{
    float L_RPM;
    float R_RPM;
}RPM;

RPM vel2rpm(const geometry_msgs::Twist &msg);

sensor_msgs::Joy joy_mani;

void msgCallback(const sensor_msgs::Joy::ConstPtr &joy);
void f_init_Callback(const std_msgs::Bool::ConstPtr &msg);
void pub_mobile();
void pub_mani(sensor_msgs::Joy joy_mani);
void pub_zero_vel();
void pub_zero_joy();
void flipper_control();
void connect_cb(const std_msgs::Bool::ConstPtr &msg);
void flipper_auto_control();

ros::Publisher pub_cmd_vel;
ros::Publisher pub_flipper;

ros::Publisher FL_pub;
ros::Publisher FR_pub;
ros::Publisher BL_pub;
ros::Publisher BR_pub;
ros::Publisher RPM_pub;

ros::Subscriber robot_state_sub;
ros::Subscriber sub;
ros::Subscriber f_init_trigger_sub;
ros::Subscriber connect_sub;

std_msgs::Float64 FL;
std_msgs::Float64 FR;
std_msgs::Float64 BL;
std_msgs::Float64 BR;

std_msgs::Float64MultiArray rpm_input_msg;

float target_FL;
float target_FR;
float target_BL;
float target_BR;

float now_FL;
float now_FR;
float now_BL;
float now_BR;



bool init_done=false;

/*joystick data_flipper*/
float _flipper_pos[4]={130,130,90,90};
bool flipper_auto=false;

/*joystick data_mani*/

std::string connect;
bool joy_mode=false;
bool pub_auto_flipper=false;

bool in_once=true;
bool waypoint_btn_once = true;

bool x_btn_once = true;
bool o_btn_once = true;

bool flipper_init_trigger=false;

float
leftStickY=0,
leftStickX=0,
rightStickY=0,
rightStickX=0,
l2,
r2;

float
ori_X = 0.0,
ori_Y = 90.0,
ori_Z = 0.0;

float
joy_X = 0.0,
joy_Y = 0.0,
joy_Z = 0.0;

int
arrowsX=0,
arrowsY=0,
buttonSq=0,
buttonX=0,
buttonO=0,
buttonTr=0,
l1=0,
r1=0,
buttonShare=0;

int buttonOption=0;
int buttonTouch=0;
int yawturn=0;
int buttonCenter=0;
int buttonLeftJoy=0;
int buttonRightJoy=0;

float rightStickX_, rightStickY_;
float axisL2,axisR2;

int buttonModeChange=0;
int ModeChange=0;

float data_sum=0;
