#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Bool.h"

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include <unistd.h>

#include "rt_thread/r_theta.h"

#include <math.h>

float R,Theta = 0.0;

int operating_mode=5;           //start mode = cmd_vel mode


#define MAX_RANGE 3

#define LEFT_ANG 90
#define RIGHT_ANG 270

#define DEG2RAD 0.0174533
#define RAD2DEG 180.0/3.141592

//#define SIDE_DEG 45
int SIDE_DEG = 58; //(int)(RAD2DEG*atan(1.2/0.75)); = 57.99

#define TRIANGLE 0.36// 0.31//0.45//0.31 //0.34

#define PRESENT_PAST_RATIO 0.9

#define MAX_LINEAR_VEL 0.2//0.4 //2.0
#define MAX_ANGULAR_Z 1.0  //2.0

#define Robot_Width 0.48//0.55

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
int wait_threshold = PARKING_AREA_TIME*LOOP_RATE;

int phase2_count = 0;
int phase4_count =0;
int phase5_count=0;

float yd_laserscan_arr[720]={0};
float pre_yd_laserscan_arr[720]={0};

bool is_posi_mode_b = false;
int service_result = 1;

void modeCallback(const std_msgs::Int8::ConstPtr& msg);

void scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg){
  int i=0;
  float left_triangle=0;
  float right_triangle=0;
/*
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
*/
yd_laserscan_arr[0]=msg->ranges[1];
  for(i=1;i<719;i++){
    if(msg->ranges[i]==0){
        yd_laserscan_arr[i]=yd_laserscan_arr[i-1];
    }
    else{
      yd_laserscan_arr[i]=msg->ranges[i];
    }
  }

  for(i=0;i<719;i++){
    pre_yd_laserscan_arr[i]=yd_laserscan_arr[i];
  }


  for(i=0;i<90;i++){
    if(isinf(msg->ranges[i*2])){
      laserscan_arr[89-i] = MAX_RANGE;
    }
    else laserscan_arr[89-i]=yd_laserscan_arr[i*2];
  }
  for(i=269;i<359;i++){
    if(isinf(msg->ranges[i*2])){
      laserscan_arr[89+(359-i)] = MAX_RANGE;
    }
    laserscan_arr[89+(359-i)]=yd_laserscan_arr[i*2];
  }

  for(i=0;i<SIDE_DEG;i++){
    left_triangle += (laserscan_arr[i]*laserscan_arr[i]);
  }
  for(i=180-SIDE_DEG-1;i<180;i++){
    right_triangle += (laserscan_arr[i]*laserscan_arr[i]);
  }
/*
  printf("[");
  for(i=0;i<180;i++){
    printf("%d : %f\n",i,laserscan_arr[i]);
  }
  printf("]");

  ROS_INFO("1 deg : %f",laserscan_arr[1]);
*/
  left_triangle *= (0.5*DEG2RAD);
  right_triangle *=(0.5*DEG2RAD);

  //ROS_INFO("left_triangle : %f, right_triangle : %f",left_triangle,right_triangle);

 // float triangle_error = left_triangle-right_triangle;
  float triangle_error = TRIANGLE-right_triangle;
  ROS_INFO("triangle : %f",right_triangle);

  float left_square = left_triangle - TRIANGLE;

  if(phase == 0){
    //ROS_INFO("size : %d",sizeof(arrarrarr)/sizeof(float));
/*
    if((1.2*1.2 - left_square) < 0.35){
      phase++;
    }
*/
    if((laserscan_arr[2]+laserscan_arr[3]+laserscan_arr[4])/3 > 1.2){
      phase2_count ++;
      if(phase2_count >=5){
        //phase++;
        is_posi_mode_b = true;
        linear_x = 0.0;
        angular_z = 0.0;
        if(phase2_count >=6){
          phase++;
        }
      }
    }
    linear_x = MAX_LINEAR_VEL*(((1.25*1.25) - left_square)/(1.25*1.25)); //Kp*0.015*((1.25*1.25) - left_square); //+ MAX_LINEAR_VEL*PRESENT_PAST_RATIO + pre_linear_x*(1 - PRESENT_PAST_RATIO) - left_square * Kp*0.01;
    angular_z = MAX_ANGULAR_Z*(triangle_error/TRIANGLE);//PRESENT_PAST_RATIO * triangle_error * Kp + pre_angular_z * (1 - PRESENT_PAST_RATIO);
    if(angular_z > MAX_ANGULAR_Z){
      angular_z = MAX_ANGULAR_Z;
    }
    if(angular_z < -1*MAX_ANGULAR_Z){
      angular_z = -1*MAX_ANGULAR_Z;
    }
    angular_z = PRESENT_PAST_RATIO*angular_z + (1-PRESENT_PAST_RATIO)*pre_angular_z;
    pre_angular_z = angular_z;
    ROS_INFO("phase0");
    ROS_WARN("angle 0 : %f angle 90 : %f angle 180 : %f ",laserscan_arr[0],laserscan_arr[90],laserscan_arr[179]);
  }
  if(phase == 1){

    is_posi_mode_b = true;

/*

    float r_vel,l_vel;
    l_vel = 0.1;//0.2;//0.4;                                                                  //tuning 1-1
    r_vel = l_vel * ((((1.5 - Robot_Width)*0.5)+Robot_Width)/((1.5-Robot_Width)*0.5));
    //r_vel = l_vel*2.1579;
    float ease_curve = 0.0;//-0.1; //0.07;                                                    //tuning 1-2

    //ROS_INFO("r_vel = %f",r_vel);
    float ang_vel = (r_vel-l_vel)/Robot_Width - ease_curve;
    float lin_vel = (r_vel+l_vel)/2;

    linear_x = lin_vel;
    angular_z = ang_vel;

    if(left_triangle < TRIANGLE*0.8){                                                   //tuning 1-3
      phase ++;
    }

    */
ROS_INFO("phase1");
    R = 0.20;
    Theta = 0.17; //0.16

    if(left_triangle < TRIANGLE){                                                   //tuning 1-3
      phase ++;
      is_posi_mode_b = false;
      R = 0.0;
     Theta = 0.0;
    }
  }
  if(phase == 2){
    is_posi_mode_b = false;
    ROS_INFO("phase 2");
    for(i=0;i<180;i++){
      sum_all_scan += laserscan_arr[i];
    }
    //ROS_INFO("sum_all_scan : %f",sum_all_scan);
    if(sum_all_scan < stop_threshold){
      phase++;
    }

    float triangle_error_rl = left_triangle-right_triangle;
    linear_x = 0.15*phase2_Kp * sum_all_scan;                                            //tuning 2-1
    angular_z = 0.15*phase2_Kp_angular * triangle_error_rl;                              //tuning 2-2

    sum_all_scan = 0.0;

  }
  if(phase == 3){


    ROS_INFO("phase3");
    linear_x = 0.0;
    angular_z = 0.0;
    /*
    if((wait++) > wait_threshold){
      phase ++;
    }
    */
    sleep(4);            //sleep 3+1 sec
    phase++;

  }
  if(phase == 4){
    //ROS_INFO("wait_theashold : %d",wait_threshold);
    ROS_INFO("phase 4");
    pre_angular_z = 0.0;
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
    linear_x = -0.1;                                                                    //tuning 3-1
    angular_z = -1 * 0.05*phase2_Kp_angular * triangle_error_rl;                         //tuning 3-2
    angular_z = PRESENT_PAST_RATIO*angular_z + (1-PRESENT_PAST_RATIO)*pre_angular_z;

    //sum_all_scan = 0.0;

    /*if(left_triangle > PARKING_AREA_TRIAGNLE){
      phase ++;
    }*/

    if(laserscan_arr[88]>0.65 ||
       laserscan_arr[89]>0.65 ||
       laserscan_arr[90]>0.65 ||
       laserscan_arr[91]>0.65 ||
       laserscan_arr[92]>0.65 ){    //0.7                                                      //tuning 4-1
      phase4_count++;
      if(phase4_count>=5){
        linear_x = 0.0;
        angular_z = 0.0;
        is_posi_mode_b = true;
        if(phase4_count>=6){
          phase++;
        }
      }
    }
  }
  if(phase == 5){

    ROS_INFO("phase 5");
/*
    float r_vel,l_vel;
    l_vel = -0.2; //-0.4;                                                                //tuning 5-1
    r_vel = l_vel * ((((1.5 - Robot_Width)*0.5)+Robot_Width)/((1.5-Robot_Width)*0.5));
    float ease_curve = 0.0;//0.08;                                                       //tuning 5-2

    //ROS_INFO("r_vel = %f",r_vel);
    float ang_vel = (r_vel-l_vel)/Robot_Width - ease_curve;
    float lin_vel = (r_vel+l_vel)/2;

    linear_x = lin_vel;
    angular_z = ang_vel;
*/
    //ang_vel *=-1;
    //lin_vel *=-1;

    is_posi_mode_b = true;

    R = -0.20;
    Theta = -0.20;


    if(laserscan_arr[81]>(MAX_RANGE) ||
       laserscan_arr[82]>(MAX_RANGE) ||
       laserscan_arr[83]>(MAX_RANGE) ||
       laserscan_arr[84]>(MAX_RANGE) ||
       laserscan_arr[85]>(MAX_RANGE) ||
       laserscan_arr[86]>(MAX_RANGE) ||
       laserscan_arr[88]>(MAX_RANGE) ||
       laserscan_arr[89]>(MAX_RANGE) ||
       laserscan_arr[90]>(MAX_RANGE) ||
       laserscan_arr[91]>(MAX_RANGE) ||
       laserscan_arr[92]>(MAX_RANGE) ||
       laserscan_arr[93]>(MAX_RANGE) ||
       laserscan_arr[94]>(MAX_RANGE) ||
       laserscan_arr[95]>(MAX_RANGE) ||
       laserscan_arr[96]>(MAX_RANGE) ||
       laserscan_arr[97]>(MAX_RANGE) ||
       laserscan_arr[98]>(MAX_RANGE) ||
       laserscan_arr[99]>(MAX_RANGE)
       ){
        phase5_count++;
        ROS_WARN("isinf");
        ROS_INFO("laser90 : %f",laserscan_arr[90]);
        if(phase5_count >= 5){

          pre_angular_z = 0.0;
          is_posi_mode_b = false;
          R = 0.0;
          Theta = 0.0;
          ROS_INFO("r,theta = 0");
          if(phase5_count>=6){
           phase++;
          }
         
        }
    }
  }
  if(phase == 6){
    is_posi_mode_b = false;
    ROS_INFO("phase6");
    float right_error = 0.5* 0.7*1.414- laserscan_arr[135];                                                                        //tuning 5-0

  //  if(laserscan_arr[170]>(MAX_RANGE-1)){
   //   phase++;
   // }
   if(laserscan_arr[170]<0.85){
     phase++;
   }
/*
    else if(laserscan_arr[120]>(MAX_RANGE)){
     ROS_INFO("6-2");
      linear_x = 0.1;
      right_error = 0.6 - laserscan_arr[179];
      angular_z = PRESENT_PAST_RATIO * right_error * Kp*0.5 + pre_angular_z * (1 - PRESENT_PAST_RATIO);
      pre_angular_z = angular_z;

    }
    else if(laserscan_arr[160]>(MAX_RANGE)){
     ROS_INFO("6-3");
      linear_x = 0.05;
      right_error = 0.7 - laserscan_arr[179];
      angular_z = PRESENT_PAST_RATIO * right_error * Kp*0.5 + pre_angular_z * (1 - PRESENT_PAST_RATIO);
      pre_angular_z = angular_z;
    }
*/
    else{
      ROS_INFO("6-1");

     // float right_error = 0.7*1.414- laserscan_arr[170];//135                                                                       //tuning 5-0
     float right_error = 0.7 - laserscan_arr[170];
      linear_x =  0.2;//+ MAX_LINEAR_VEL*PRESENT_PAST_RATIO + pre_linear_x*(1 - PRESENT_PAST_RATIO) - left_square * Kp*0.01;    //tuning 5-1
      //angular_z = PRESENT_PAST_RATIO * right_error * Kp*0.5 + pre_angular_z * (1 - PRESENT_PAST_RATIO);                         //tuning 5-2
      angular_z =0.5* MAX_ANGULAR_Z*(right_error/0.7);
      angular_z = PRESENT_PAST_RATIO * angular_z + (1-PRESENT_PAST_RATIO)*pre_angular_z;
      pre_angular_z = angular_z;
    }
  }
  if(phase == 7){
  
  if(laserscan_arr[170]>1.0){
    phase++;
  }
  else{
   float right_error_2 = 0.7-laserscan_arr[170];
   linear_x = 0.15;
  angular_z=0.5*MAX_ANGULAR_Z*(right_error_2/0.7);

  angular_z = PRESENT_PAST_RATIO * angular_z + (1-PRESENT_PAST_RATIO)*pre_angular_z;
  pre_angular_z = angular_z;
  }
}  
  if(phase == 8){
   ROS_INFO("phase7");
    linear_x = 0.0;
    angular_z = 0.0;
    ROS_INFO("=====================================================");
    ROS_INFO("*****************************************************");
    ROS_INFO("=====================================================");
    ROS_INFO("*****************************************************");

  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "auto_parking_new_node");
  ros::NodeHandle nh;

  ros::Publisher cmdvel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Publisher status_pub = nh.advertise<std_msgs::UInt16>("/parking_state", 1000);
  ros::Publisher is_posi_mode_pub = nh.advertise<std_msgs::Bool>("/is_posi_mode", 1000);

  ros::Subscriber scan_sub = nh.subscribe("/scan", 20, scan_Callback);
  ros::Subscriber mode_sub = nh.subscribe("/mode", 10, modeCallback);

  ros::ServiceClient r_theta_client = nh.serviceClient<rt_thread::r_theta>("r_theta_go");



  ros::Rate loop_rate(LOOP_RATE);
  while (ros::ok())
  {
    //ROS_INFO("%d",SIDE_DEG);
    geometry_msgs::Twist msg;
    std_msgs::UInt16 status_msg;
    status_msg.data = 1;
    status_pub.publish(status_msg);

    std_msgs::Bool is_posi_mode_msg;
    is_posi_mode_msg.data = is_posi_mode_b;

    msg.linear.x = linear_x;
    msg.angular.z = angular_z;

    is_posi_mode_pub.publish(is_posi_mode_msg);

    if(operating_mode == 5){  // 5 = JoyNotUse
      cmdvel_pub.publish(msg);
     // ROS_INFO("operating mode : %d",operating_mode);
    }
    else{
      ROS_WARN("Operating_Mode is not JoyNotUse(5)mode, Robot will not move");

    }

    rt_thread::r_theta srv;
    srv.request.r = R;
    srv.request.theta = Theta;

//   ROS_INFO("test");
//   ROS_INFO("is_posi_mode: %d, R : %f, Theta : %f",is_posi_mode_b,R,Theta);
    if(is_posi_mode_b == true && (fabs(R)>0.000001 || fabs(Theta)>0.00001)){
     ROS_INFO("service_enter");
      if(service_result == 1){
        service_result = 0;
       ROS_INFO("service may start");
      if (r_theta_client.call(srv)) {
          //ROS_INFO("send srv: %d + %d", (long int)srv.request.a, (long int)srv.request.b);
          ROS_INFO("receive srv: %d", (float)srv.response.result);
          service_result = srv.response.result;

          R=0.0;
          Theta = 0.0;
      }
      else {
          ROS_ERROR("Failed to call service ros_tutorial_srv");
      }
      }
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}





void modeCallback(const std_msgs::Int8::ConstPtr& msg){

  if(operating_mode != msg->data){

    //ROS_INFO("modecallback1");
    operating_mode = msg->data;
  }

  //ROS_INFO("modecallback2");
}

