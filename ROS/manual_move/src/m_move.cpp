/*
  This node is dedicated for moving the robot with joystick ( Manually )
*/

#include<ros/ros.h>
#include<sensor_msgs/Joy.h>
#include<geometry_msgs/Twist.h>

sensor_msgs::Joy joy;

float x=0, y=0 , z=0 , l=0; // Global variables 

// This callback Funcion receives messages from joystick and assign it to a variable

void callback ( const sensor_msgs::Joy::ConstPtr &msg)
{
  x=msg->axes[7];  // Receiving message from upper and lower arrow in the joystick and give it to a variable called " X "
  y=msg->axes[6];  // Receiving message from Righ and left arrow in the joystick and give it to a variable called " Y "
  z=msg->buttons[4];
  l=msg->buttons[5];
}

int main(int argc ,char** argv )
{
    ros::init(argc,argv,"mover");
    ros::NodeHandle nh; 
    ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("/cmd_vel",10); 
    ros::Subscriber sub = nh.subscribe("/joy",10,&callback); 
    ros::Rate loop(10);

  while(ros::ok())
  {

   geometry_msgs::Twist twist; // creating object from geoemtry messages

   twist.linear.x=x; // Receiving joystick message of moving forward or backward

   twist.angular.z=y;  // Receiving joystick message of moving Right or left

   twist.linear.y=z;

   twist.angular.y=l;

   pub.publish(twist); // Publishing messages assigned from joy node

   ros::spinOnce();

   loop.sleep();

  }
  return 0;
}