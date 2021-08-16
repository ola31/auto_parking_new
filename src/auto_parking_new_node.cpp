#include "ros/ros.h"
#include "std_msgs/String.h"

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include <math.h>

#define MAX_RANGE 10

#define LEFT_ANG 90
#define RIGHT_ANG 270

#define DEG2RAD 0.0174533
#define RAD2DEG 180.0/3.141592

//#define SIDE_DEG 45
int SIDE_DEG = 58; //(int)(RAD2DEG*atan(1.2/0.75)); = 57.99

#define TRIANGLE 0.34

#define PRESENT_PAST_RATIO 0.7

#define MAX_LINEAR_VEL 1.0 //2.0

#define Robot_Width 0.55

#define LOOP_RATE 30

#define PARKING_AREA_TIME 3

#define PARKING_AREA_TRIAGNLE (0.6)*(1.2)*0.5


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
float phase2_Kp = 0.005;
float phase2_Kp_angular = 10.0;

float sum_all_scan = 0;
float stop_threshold = 70;

int wait =0;
int wait_threshold = (float)PARKING_AREA_TIME/((float)1.0/(float)LOOP_RATE);

int phase2_count = 0;
int phase4_count =0;
int phase5_count=0;

void scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg){
  int i=0;
  float left_triangle=0;
  float right_triangle=0;

  for(i=0;i<180;i++){
    if(isinf(msg->ranges[i+LEFT_ANG])){
      laserscan_arr[i] = MAX_RANGE;
    }
    else laserscan_arr[i] = msg->ranges[i+LEFT_ANG];
  }

  for(i=0;i<SIDE_DEG;i++){
    left_triangle += (laserscan_arr[i]*laserscan_arr[i]);
  }
  for(i=180-SIDE_DEG-1;i<180;i++){
    right_triangle += (laserscan_arr[i]*laserscan_arr[i]);
  }

  left_triangle *= (0.5*DEG2RAD);
  right_triangle *=(0.5*DEG2RAD);

  //ROS_INFO("left_triangle : %f, right_triangle : %f",left_triangle,right_triangle);

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
      phase2_count ++;
      if(phase2_count >=5){
        phase++;
      }
    }
    linear_x =  Kp*0.015*((1.25*1.25) - left_square); //+ MAX_LINEAR_VEL*PRESENT_PAST_RATIO + pre_linear_x*(1 - PRESENT_PAST_RATIO) - left_square * Kp*0.01;
    angular_z = PRESENT_PAST_RATIO * triangle_error * Kp + pre_angular_z * (1 - PRESENT_PAST_RATIO);
  }
  if(phase == 1){



    float r_vel,l_vel;
    l_vel = 0.2;//0.4;
    r_vel = l_vel * ((((1.5 - Robot_Width)*0.5)+Robot_Width)/((1.5-Robot_Width)*0.5));
    float ease_curve = -0.1; //0.07;

    //ROS_INFO("r_vel = %f",r_vel);
    float ang_vel = (r_vel-l_vel)/Robot_Width - ease_curve;
    float lin_vel = (r_vel+l_vel)/2;

    linear_x = lin_vel;
    angular_z = ang_vel;

    if(left_triangle < TRIANGLE*0.5){
      phase ++;
    }
  }
  if(phase == 2){
    ROS_INFO("phase 2");
    for(i=0;i<180;i++){
      sum_all_scan += laserscan_arr[i];
    }
    //ROS_INFO("sum_all_scan : %f",sum_all_scan);
    if(sum_all_scan < stop_threshold){
      phase++;
    }

    float triangle_error_rl = left_triangle-right_triangle;
    linear_x = 0.4*phase2_Kp * sum_all_scan;
    angular_z = 0.1*phase2_Kp_angular * triangle_error_rl;

    sum_all_scan = 0.0;

  }
  if(phase == 3){
    ROS_INFO("phase3");
    linear_x = 0.0;
    angular_z = 0.0;
    if((wait++) > wait_threshold){
      phase ++;
    }

  }
  if(phase == 4){
    //ROS_INFO("wait_theashold : %d",wait_threshold);
    ROS_INFO("phase 4");
/*
    if(left_triangle > PARKING_AREA_TRIAGNLE){
      phase ++;
    }
    */

/*
    for(i=0;i<180;i++){
      sum_all_scan += laserscan_arr[i];
    }
*/
    //ROS_INFO("sum_all_scan : %f",sum_all_scan);

    float triangle_error_rl = left_triangle-right_triangle;
    linear_x = -0.4;
    angular_z = -1 * 0.5*phase2_Kp_angular * triangle_error_rl;

    sum_all_scan = 0.0;

    /*if(left_triangle > PARKING_AREA_TRIAGNLE){
      phase ++;
    }*/
    if(laserscan_arr[90]>0.7){
      phase4_count++;
      if(phase4_count>5){
        phase++;
      }
    }
  }
  if(phase == 5){

    ROS_INFO("phase 5");

    float r_vel,l_vel;
    l_vel = -0.2; //-0.4;
    r_vel = l_vel * ((((1.5 - Robot_Width)*0.5)+Robot_Width)/((1.5-Robot_Width)*0.5));
    float ease_curve = 0.0;//0.08;

    //ROS_INFO("r_vel = %f",r_vel);
    float ang_vel = (r_vel-l_vel)/Robot_Width - ease_curve;
    float lin_vel = (r_vel+l_vel)/2;

    linear_x = lin_vel;
    angular_z = ang_vel;

    //ang_vel *=-1;
    //lin_vel *=-1;

    if(laserscan_arr[81]>(MAX_RANGE-2) ||
       laserscan_arr[82]>(MAX_RANGE-2) ||
       laserscan_arr[83]>(MAX_RANGE-2) ||
       laserscan_arr[84]>(MAX_RANGE-2) ||
       laserscan_arr[85]>(MAX_RANGE-2) ||
       laserscan_arr[86]>(MAX_RANGE-2) ||
       laserscan_arr[88]>(MAX_RANGE-2) ||
       laserscan_arr[89]>(MAX_RANGE-2) ||
       laserscan_arr[90]>(MAX_RANGE-2) ||
       laserscan_arr[91]>(MAX_RANGE-2) ||
       laserscan_arr[92]>(MAX_RANGE-2) ||
       laserscan_arr[93]>(MAX_RANGE-2) ||
       laserscan_arr[94]>(MAX_RANGE-2) ||
       laserscan_arr[95]>(MAX_RANGE-2) ||
       laserscan_arr[96]>(MAX_RANGE-2) ||
       laserscan_arr[97]>(MAX_RANGE-2) ||
       laserscan_arr[98]>(MAX_RANGE-2) ||
       laserscan_arr[99]>(MAX_RANGE-2)
       ){
        phase5_count++;
        ROS_WARN("isinf");
        if(phase5_count >= 5){
          phase++;
          pre_angular_z = 0.0;
        }
    }
  }
  if(phase == 6){
    ROS_INFO("phase6");
    float right_error = 0.6*1.414- laserscan_arr[135];
    linear_x =  0.4;//+ MAX_LINEAR_VEL*PRESENT_PAST_RATIO + pre_linear_x*(1 - PRESENT_PAST_RATIO) - left_square * Kp*0.01;
    angular_z = PRESENT_PAST_RATIO * right_error * Kp*0.5 + pre_angular_z * (1 - PRESENT_PAST_RATIO);
    int i=0;

    if(laserscan_arr[170]>(MAX_RANGE-2)){
      phase++;
    }
    if(laserscan_arr[120]>(MAX_RANGE-2)){
      linear_x = 0.2;
      right_error = 0.6 - laserscan_arr[179];
      angular_z = PRESENT_PAST_RATIO * right_error * Kp*0.5 + pre_angular_z * (1 - PRESENT_PAST_RATIO);

    }
    if(laserscan_arr[160]>(MAX_RANGE-2)){
      linear_x = 0.1;
      right_error = 0.6 - laserscan_arr[179];
      angular_z = PRESENT_PAST_RATIO * right_error * Kp*0.5 + pre_angular_z * (1 - PRESENT_PAST_RATIO);
    }
  }
  if(phase == 7){
    linear_x = 0.0;
    angular_z = 0.0;

  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "auto_parking_new_node");
  ros::NodeHandle nh;

  ros::Publisher cmdvel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber cmd_vel_sub = nh.subscribe("/scan", 1000, scan_Callback);

  ros::Rate loop_rate(LOOP_RATE);
  while (ros::ok())
  {
    //ROS_INFO("%d",SIDE_DEG);
    geometry_msgs::Twist msg;
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;

    cmdvel_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
