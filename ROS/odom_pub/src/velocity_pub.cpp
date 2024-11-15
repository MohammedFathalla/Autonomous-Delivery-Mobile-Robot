/*
  This node is dedicated for sending velocities to low level arduino ( PWM and direction to motors )
*/

#include<ros/ros.h>
#include<geometry_msgs/Point.h>
#include<geometry_msgs/Twist.h>
// #include <cmath>

double linear_vel_R; // linear velocity from move_base
double angular_vel_R; // angilar velocity from move_base
double linear_vel_L;
double angular_vel_L;

ros::Publisher pub1;
ros::Publisher pub2;

geometry_msgs::Point point_L;
geometry_msgs::Point point_R;

void callback1 ( const geometry_msgs::Twist &twist )
{
    point_L.x =(float)twist.linear.x;
    point_L.z =(float)twist.angular.z;

    point_R.x =(float)twist.linear.x;
    point_R.z =(float)twist.angular.z;
}

int main( int argc , char **argv)
{
    ros::init(argc,argv,"Velocity");
    
    // initializing our node
    ros::NodeHandle nh;
    
    // Receiving velocity that is send on cmd_vel topic by move_base
    ros::Subscriber sub1 = nh.subscribe("/cmd_vel", 100 , &callback1);  // Was 1000
    
    // Publishing left motor velocity 
    pub1 = nh.advertise<geometry_msgs::Point>("/velocity_L", 100);  // Was 1000
    
    // Publishing left motor velocity 
    pub2 = nh.advertise<geometry_msgs::Point>("/velocity_R", 100); // Was 1000
    
    ros::Rate loop(100);  // That`s frequency of publishing (Hz) not time of publishing  10 means 1/10 = 0.1 seconds
    // was 100 and things started to become slow

    while(ros::ok())
    {

    pub1.publish(point_L);

    pub2.publish(point_R);

    ros::spinOnce();

    loop.sleep();
    }

    return 0;
}