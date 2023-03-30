#include "../include/kubo_joy/kubo_joy.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32.h"



int main(int argc, char **argv) //c++ 의 기본 함수형태
{
    ros::init(argc, argv, "kubo_joy"); //노드명 초기화
    ros::NodeHandle nh; //ROS시스템과 통신을 위한 노드핸들 선언
    ros::NodeHandle nh_;


    pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel_operator",1);
//    pub_flipper = nh.advertise<kubo_msgs::RobotState>("flipper",1);

    FL_pub = nh.advertise<std_msgs::Float64>("/flipper_joint_FL_position_controller/command",10);
    FR_pub = nh.advertise<std_msgs::Float64>("/flipper_joint_FR_position_controller/command",10);
    BL_pub = nh.advertise<std_msgs::Float64>("/flipper_joint_BL_position_controller/command",10);
    BR_pub = nh.advertise<std_msgs::Float64>("/flipper_joint_BR_position_controller/command",10);

    RPM_pub = nh.advertise<std_msgs::Float64MultiArray>("/sprocket_velocity_controller/command",10);

    sub = nh.subscribe("joy", 100, msgCallback);

    ros::Rate loop_rate(100);
//    ros::spin();
    while(ros::ok()) //ros 가 활성화되면
    {
      flipper_control();
      pub_mobile();

      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}

void msgCallback(const sensor_msgs::Joy::ConstPtr &joy)
{

          leftStickX = joy->axes[0];
          leftStickY = joy->axes[1];
          rightStickX = joy->axes[3];
          rightStickY = joy->axes[4];
          arrowsX = joy->axes[6];
          arrowsY = joy->axes[7];
    //      axisL2 = joy->axes[2];
    //      axisR2 = joy->axes[5];


          buttonSq = joy->buttons[3];
          buttonX = joy->buttons[0];
          buttonO = joy->buttons[1];
          buttonTr= joy->buttons[2];
          l1 = joy->buttons[4];
          r1 = joy->buttons[5];
          l2 = joy->buttons[6];
          r2 = joy->buttons[7];
          buttonShare = joy->buttons[8];
          buttonOption= joy->buttons[9];
          buttonLeftJoy = joy->buttons[11];
          buttonRightJoy= joy->buttons[12];
          buttonCenter = joy->buttons[10];



}



void pub_mobile()
{

    geometry_msgs::Twist msg;
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;

    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;

    msg.linear.x= (leftStickY*MAX_VEL);
    msg.angular.z=-(rightStickX*MAX_ANG);

    RPM rpm;
    rpm = vel2rpm(msg);

    std_msgs::Float64MultiArray input_rpm;
    input_rpm.data = {rpm.L_RPM,rpm.R_RPM,rpm.R_RPM,rpm.L_RPM,rpm.R_RPM,rpm.L_RPM};
    rpm_input_msg = input_rpm;


    RPM_pub.publish(rpm_input_msg);
}

RPM vel2rpm(const geometry_msgs::Twist &msg)
{
  float wheel_circumference_ = wheel_diameter*3.1415926535;
  float linear_vel_x_mins;
  float linear_vel_y_mins;
  float angular_vel_z_mins;
  float tangential_vel;
  float x_rpm;
  float tan_rpm;

  //convert m/s to m/min
  linear_vel_x_mins = msg.linear.x * 6;

  //convert rad/s to rad/min
  angular_vel_z_mins = msg.angular.z * 6;


  tangential_vel = angular_vel_z_mins * (((wheels_x_distance_/(float)1000) / 2) + ((wheels_y_distance_/(float)1000) / 2));

  x_rpm = linear_vel_x_mins / (wheel_circumference_/(float)1000);
  tan_rpm = tangential_vel / (wheel_circumference_/(float)1000);

  //calculate for the target motor RPM and direction
  RPM rpm;
  rpm.R_RPM = x_rpm - tan_rpm;
  rpm.L_RPM = x_rpm + tan_rpm;


  return rpm;
}



void flipper_control()
{

  _flipper_pos[0] += l1*arrowsY*0.3;
  _flipper_pos[1] += r1*arrowsY*0.3;
  _flipper_pos[2] += l2*arrowsY*0.3;
  _flipper_pos[3] += r2*arrowsY*0.3;

  if(buttonSq==1)
  {
      _flipper_pos[0] = 45;
      _flipper_pos[1] = 45;
      _flipper_pos[2] = 30;
      _flipper_pos[3] = 30;
  }

  if(buttonX==1&&x_btn_once==true)
  {
      std_msgs::Bool msg;
      msg.data=true;
      x_btn_once=false;
  }
  else if(buttonX==0)
  {
    x_btn_once=true;

  }

//  if(buttonTr==1)
//  {
//      _flipper_pos[0] = 90;
//      _flipper_pos[1] = 90;
//      _flipper_pos[2] = 90;
//      _flipper_pos[3] = 90;
//  }

//  if(buttonX==1)
//  {
//      std_msgs::Bool msg;
//      msg.data=false;
//      pub_obj.publish(msg);
//  }

  if(buttonOption==1)
  {
      _flipper_pos[0]=130;
      _flipper_pos[1]=130;
      _flipper_pos[2]=90;
      _flipper_pos[3]=90;
  }


  for(int i=0;i<2;i++)
  {
    if(_flipper_pos[i] > MAX_F_FLIPPER) _flipper_pos[i] = MAX_F_FLIPPER;
    if(_flipper_pos[i] < MIN_F_FLIPPER) _flipper_pos[i] = MIN_F_FLIPPER;
  }

  for(int i=2;i<4;i++)
  {
    if(_flipper_pos[i] > MAX_B_FLIPPER) _flipper_pos[i] = MAX_B_FLIPPER;
    if(_flipper_pos[i] < MIN_B_FLIPPER) _flipper_pos[i] = MIN_B_FLIPPER;
  }             

  target_FL=-(float)_flipper_pos[0]*PI/180;
  target_FR=(float)_flipper_pos[1]*PI/180;
  target_BL=(float)_flipper_pos[2]*PI/180;
  target_BR=-(float)_flipper_pos[3]*PI/180;

  if(now_FL > target_FL + 0.01)
    now_FL -= FLIPPER_SPEED_GAIN;
  else if(now_FL < target_FL - 0.01)
    now_FL += FLIPPER_SPEED_GAIN;
  else if((now_FL > target_FL -0.01)&&(now_FL<target_FL+0.01))
    now_FL=target_FL;

  if(now_FR > target_FR + 0.01)
    now_FR -= FLIPPER_SPEED_GAIN;
  else if(now_FR < target_FR - 0.01)
    now_FR += FLIPPER_SPEED_GAIN;
  else if((now_FR > target_FR -0.01)&&(now_FR<target_FR+0.01))
    now_FR=target_FR;

  if(now_BL > target_BL + 0.01)
    now_BL -= FLIPPER_SPEED_GAIN;
  else if(now_BL < target_BL - 0.01)
    now_BL += FLIPPER_SPEED_GAIN;
  else if((now_BL > target_BL -0.01)&&(now_BL<target_BL+0.01))
    now_BL=target_BL;

  if(now_BR > target_BR + 0.01)
    now_BR -= FLIPPER_SPEED_GAIN;
  else if(now_BR < target_BR - 0.01)
    now_BR += FLIPPER_SPEED_GAIN;
  else if((now_BR > target_BR -0.01)&&(now_BR<target_BR+0.01))
    now_BR=target_BR;






  FL.data=now_FL;
  FR.data=now_FR;
  BL.data=now_BL;
  BR.data=now_BR;

  FL_pub.publish(FL);
  FR_pub.publish(FR);
  BL_pub.publish(BL);
  BR_pub.publish(BR);
}
