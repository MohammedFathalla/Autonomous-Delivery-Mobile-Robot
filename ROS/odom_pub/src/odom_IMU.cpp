/*
   This node is dedicated for Publishing odometry and broadcasting transformations
*/

// Header files
#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Point.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/Imu.h>

// Math constant
const double pi = 3.14;

// Constants for My mobile robot
#define R 0.125
#define L 0.58
#define N 25000

// Variables for my Robot
double Dc=0.0;            // incremental instanteneous distance for robot
double Dr=0.0;            // incremental instanteneius distance for right wheel
double Dl=0.0;            // incremental instanteneous distance for left wheel
double Rtick_new=0.0;     // new right wheel ticks
double Rtick_old=0.0;     // old right wheel ticks
double Ltick_new=0.0;     // new left wheel ticks
double Ltick_old=0.0;     // old left wheel ticks
double x=0.0;             // overall distance in x
double y=0.0;             // overall distance in y

double th=0.0;            // overall angle of robot From IMU
double delta_th=0.0;
double vx=0.0;            // velocity of robot in x
double vy=0.0;            // velocity of robot in y
double vth=0.0;
double orientation=0.0;
double orientation_new=0.0;
double orientation_old=0.0;


// Callback function that receives imu data
void callback1(const geometry_msgs::Point &point)
{
  orientation_new = point.z;
}

// Callback function that recieves encoder tics 
void callback2( const geometry_msgs::Point &msg)
{
 // Receiving new ticks from wheel encoders
 Ltick_new = msg.x;
 Rtick_new = msg.y;

 // Calculating the moved distance of right and left wheel and robot itself
 Dr=(2*pi*R)*((Rtick_new-Rtick_old)/N);
 Dl=(2*pi*R)*((Ltick_new-Ltick_old)/N);
 Dc=((Dr+Dl)/2);

 // Computing the robot position in map
 x+=Dc*cos(orientation_new);
 y+=Dc*sin(orientation_new);

 // giving the old odometry a new value of odom
 Rtick_old=Rtick_new;
 Ltick_old=Ltick_new;

}

int main( int argc , char **argv )
{
 ros::init(argc, argv, "odom");
  
 // Initializing our node
 ros::NodeHandle nh;
  
 // Reciving IMU Yaw angle from IMU 
 ros::Subscriber subs = nh.subscribe("/imu" ,100 , &callback1); // I changed this from 1000 to 100

 // Reciving encoder ticks from arduino
 ros::Subscriber sub = nh.subscribe("/encoder" ,100 , &callback2); // I changed this from 1000 to 100
  
 // Publishing odometry on odom topic 
 ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("/odom" , 100) ; // I changed this from 1000 to 100
  
 // Broadcast transformation For Rviz
 tf::TransformBroadcaster transform;
  
 ros::Time current_time , last_time;
 //current_time = ros::Time::now();
 last_time = ros::Time::now();
  
 ros::Rate loop(100);
 // was 100 and no error shows now i set it to 10
  
 while(ros::ok())
 {
  current_time = ros::Time::now();
  
  double dt = ( current_time - last_time ).toSec();
  vx=Dc*cos(orientation_new)/dt;
  vy=Dc*sin(orientation_new)/dt;
  vth=((orientation_new-orientation_old)/dt);

  // We Converted The angle from yaw in " roll pitch yaw form " to Quaternion to be more accurate and roboust
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(orientation_new);
    
  // Tranformation Broadcasting
  geometry_msgs::TransformStamped trans;
  
  trans.header.stamp=current_time;
  trans.header.frame_id="odom";
  trans.child_frame_id="base_link";
  
  // Translation value  ( For TF )
  trans.transform.translation.x=x;
  trans.transform.translation.y=y;
  trans.transform.translation.z=0;
  trans.transform.rotation = odom_quat;
  
  transform.sendTransform(trans);
  

  // Odometry calculation
  nav_msgs::Odometry odom;
  
  odom.header.stamp=current_time;
  odom.header.frame_id="odom";  
  odom.child_frame_id="base_link";  
  
  // translation values ( For Odometry )
  odom.pose.pose.position.x=x;
  odom.pose.pose.position.y=y;
  odom.pose.pose.position.z=0;
  odom.pose.pose.orientation = odom_quat;
  
  // Linear Velocity values ( For odometry )
  odom.twist.twist.linear.x=vx;
  odom.twist.twist.linear.y=vy;
  
  // Angular Velocity values ( For odometry )
  odom.twist.twist.angular.z=vth;

  pub.publish(odom);
  last_time = current_time;
  orientation_old=orientation_new;

    ros::spinOnce();
    loop.sleep();
 }
  
    return 0;
}