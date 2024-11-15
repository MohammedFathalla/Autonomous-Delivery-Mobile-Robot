/*
 * This code is dedicated for handeling 
 * 1 - Encoder data 
 *     Reading the encoder data and send it through topic for odometery and tf calculation  ( odom and tf code in ROS files )
 *       
 * 2 - Driving the motors
 *     Receiving PWM values after PID calculation of for driving right and left wheels ( PID code in ROS files )
 * 
 */


#include <ros.h>
#include <Encoder.h>
#include<geometry_msgs/Point.h>

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder knobLeft( 3 , 2 );  // Left encoder pins
Encoder knobRight ( 20 , 21 );  // Right encoder pins


// Left motor Pins 
#define pwm1 6
#define dir1 7


// Right motor Pins
#define pwm2 5
#define dir2 4


// Global variables for Receiving PWM values
double PWM_VR;  
double PWM_VL;


// Global variables for Receiving Motor Direction values
double dir_L;
double dir_R;


long positionLeft  = -999;
long positionRight = -999;


// Object from geoemtry_msgs/Point32 to send encoder messages
geometry_msgs::Point point;


// Encoder Publisher
ros::Publisher pub("encoder", &point );


// Instantiate the node handle which allow our program to create publisher and subscriber , it also takes care of serial port communications
ros::NodeHandle nh;


// First callback function that takes right vel and dir to drive motors
void callback1 ( const geometry_msgs::Point &point1 )
{
  dir_R = point1.x;
  PWM_VR = point1.z;
  digitalWrite(dir2 , dir_R );
  analogWrite(pwm2 , PWM_VR);
}

ros::Subscriber<geometry_msgs::Point> sub("/right_vel", &callback1 );


// Second callback function that takes right vel and dir to drive motors
void callback2 ( const geometry_msgs::Point &point2 )
{
   dir_L = point2.x;
   PWM_VL = point2.z;
   digitalWrite(dir1 , dir_L );
   analogWrite(pwm1 , PWM_VL);
}

ros::Subscriber<geometry_msgs::Point> sub1("/left_vel", &callback2 );


void setup()
{  
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub1);
  nh.advertise(pub);
  Serial.begin(115200);

  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
}


void loop()
{
  nh.spinOnce();

  long newLeft, newRight;    // Variables for recieving encoder values
  newLeft = knobLeft.read(); // Recieving left encoder values
  newRight = knobRight.read(); // Recieving right encoder values
  
  if (newLeft != positionLeft || newRight != positionRight)
  { 
    
    Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.print(", Right = ");
    Serial.print(newRight);
    Serial.println();
    
    point.x=newLeft;
    point.y=newRight;

    positionLeft = newLeft;
    positionRight = newRight;

  }
  
   // Publishing encoder values
   pub.publish(&point);

}
