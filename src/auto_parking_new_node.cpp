#include "ros/ros.h"
#include "std_msgs/String.h"

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include <math.h>

#define MAX_RANGE 7

#define LEFT_ANG 90
#define RIGHT_ANG 270

#define DEG2RAD 0.0174533
#define RAD2DEG 180.0/3.141592

//#define SIDE_DEG 45
int SIDE_DEG = 58; //(int)(RAD2DEG*atan(1.2/0.75)); = 57.99

#define TRIANGLE 0.34

#define PRESENT_PAST_RATIO 0.6

#define MAX_LINEAR_VEL 1.5

#define Robot_Width 0.55


/*********************
 * S = 0.5 * (r*r) *theta
 * ********************/

int phase = 0;

float laserscan_arr[180]={0};

float pre_linear_x = 0.0;
float pre_angular_z = 0.0;
float linear_x=0;
float angular_z=0;

float Kp = 15;

void scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg){
  int i=0;
  float left_triangle=0;
  float right_triangle=0;

  for(i=0;i<180;i++){
    if(isinf(msg->ranges[i+LEFT_ANG])){
      laserscan_arr[i] = MAX_RANGE;
    }
    laserscan_arr[i] = msg->ranges[i+LEFT_ANG];
  }

  for(i=0;i<SIDE_DEG;i++){
    left_triangle += (laserscan_arr[i]*laserscan_arr[i]);
  }
  for(i=180-SIDE_DEG-1;i<180;i++){
    right_triangle += (laserscan_arr[i]*laserscan_arr[i]);
  }

  left_triangle *= (0.5*DEG2RAD);
  right_triangle *=(0.5*DEG2RAD);

  ROS_INFO("left_triangle : %f, right_triangle : %f",left_triangle,right_triangle);

 // float triangle_error = left_triangle-right_triangle;
  float triangle_error = TRIANGLE-right_triangle;

  float left_square = left_triangle - TRIANGLE;

  if(phase == 0){
/*
    if((1.2*1.2 - left_square) < 0.35){
      phase++;
    }
*/
    if((laserscan_arr[0]+laserscan_arr[1]+laserscan_arr[2])/3 > 1.2){
      static int phase2_count = 0;
      phase2_count ++;
      if(phase2_count >=5){
        phase++;
      }
    }
    linear_x =  Kp*0.02*((1.2*1.2) - left_square); //+ MAX_LINEAR_VEL*PRESENT_PAST_RATIO + pre_linear_x*(1 - PRESENT_PAST_RATIO) - left_square * Kp*0.01;
    angular_z = PRESENT_PAST_RATIO * triangle_error * Kp + pre_linear_x * (1 - PRESENT_PAST_RATIO);
  }
  if(phase == 1){

    float r_vel,l_vel;
    l_vel = 0.4;
    r_vel = l_vel * ((((1.5 - Robot_Width)*0.5)+Robot_Width)/((1.5-Robot_Width)*0.5));
    float ease_curve = 0.1;

    ROS_INFO("r_vel = %f",r_vel);
    float ang_vel = (r_vel-l_vel)/Robot_Width - ease_curve;
    float lin_vel = (r_vel+l_vel)/2;

    linear_x = lin_vel;
    angular_z = ang_vel;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "auto_parking_new_node");
  ros::NodeHandle nh;

  ros::Publisher cmdvel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber cmd_vel_sub = nh.subscribe("/scan", 1000, scan_Callback);

  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    ROS_INFO("%d",SIDE_DEG);
    geometry_msgs::Twist msg;
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;

    cmdvel_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
