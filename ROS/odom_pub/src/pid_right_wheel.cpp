/*
    This node is dedicated for converting cmd_vel to PWM values for Right motors 
       By using the PI-controller with Anti-windup technique
*/

#include<ros/ros.h>
#include<geometry_msgs/Point.h>
#include<geometry_msgs/Twist.h>
#include<math.h>

#define encoder_ticks_per_revolution 25000   // Encoder ticks per wheel revolution
#define radius 0.125        // Robot wheel radius
#define pi 3.14             
#define base_length 0.58    // Distance between center of two wheels
#define max_control 255     // maximum_pwm_signal
#define min_control -255    // minimum_pwm_signal

// PID parameters
double Kp = 1.0;
double Ki = 21.0;
double Kd = 0.0;

double encoder_ticks; // Encoder ticks of left wheel
double linear_vel;    // linear velocity from move_base
double angular_vel;   // angilar velocity from move_base
double control_signal;

double right_ticks_new;  
double right_ticks_old;

double error_prev = 0;
double eintegral = 0;  

double Required_Right_RPM;  // The Required RPM ( That the motor rquired to rotate with )
double Current_Right_RPM;   // The Calculated current RPM ( That the motor actually rotating with )
double VR;                  // Required left wheel velocity in rad/s

int error_check_sign = 0;  
double saturation_check = 0;

int dir;           // Direction of required wheel rotation ( forward or backward )
float T = 0.01;    // Sampling time in second

ros::Time last_time ;

void callback ( const geometry_msgs::Point &point )
{
   right_ticks_new = point.y;
}

void callback1 ( const geometry_msgs::Twist &twist )
{
   linear_vel = twist.linear.x;
   angular_vel = twist.angular.z;
}

/* 
PID_Right_Calc function aims to :    1 - Calculate required right wheel velocity 
                                     2 -  Using PI controller for velocity control with Anti-windup technique
                                     3 - Generating PWM values for low level control
*/                        
void PID_Right_Calc()
{ 

   ros::Time current_time = ros::Time::now();

   // Time difference 
   double dt = (( current_time - last_time ).toNSec())/(1000000000.0);  // Converting time from NSecond to S
   
    if ( dt >= T )
    {

  // Required left wheel velocity in rad/s
  VR = ( ( 2 * linear_vel ) + ( base_length * angular_vel) ) / ( 2 * radius );
  
  // Converting from angular velocity to RPM
  Required_Right_RPM = ( VR * 60 )/( 2 * pi );
  
  // Encoder difference
  double delta_encoder = ( right_ticks_new - right_ticks_old );

  // MY current RPM
  Current_Right_RPM = ( delta_encoder * 60.0 ) / ( encoder_ticks_per_revolution * dt  );
  
  // Error calculation ( Setpoint − Process Variable )
  double error = (Required_Right_RPM - Current_Right_RPM);

  // Derivative 
  double dedt = ( error - error_prev) / ( dt );

  // control signal
   control_signal = Kp*error + Ki*eintegral + Kd*dedt;


  /* Anti-Windup Technique */

   saturation_check = 0;

   error_check_sign = 0;
   if (control_signal >= 0)
   {
    saturation_check = ( control_signal > max_control) ? max_control : control_signal;
   }
   else if ( control_signal < 0 )
   {
    error_check_sign = 1;
    saturation_check = ( control_signal < min_control) ? min_control : control_signal;
   }

   int error_sign = ( error >= 0 ) ? 1 : 0;

   if ( ( error_sign == error_check_sign ) && ( control_signal != saturation_check ) )
   {
      eintegral=0;
   }
   else
   {
      eintegral += (error * dt);
   }

error_prev = error;
right_ticks_old = right_ticks_new;
last_time = current_time;
   }

 }

int main( int argc , char **argv)
{   
    ros::init(argc,argv,"right_wheel_pid");
    
    // initializing our node
    ros::NodeHandle nh;
    
    // Receiving left ticks from encoder
    ros::Subscriber sub = nh.subscribe("/encoder",100 , &callback);
    
    // Receiving velocity that is send on cmd_vel topic by move_base
    ros::Subscriber sub1 = nh.subscribe("/cmd_vel",100 , &callback1);
    
    // Publishing left motor velocity 
    ros::Publisher pub = nh.advertise<geometry_msgs::Point>("/right_vel", 100);
    
    ros::Rate loop(100);  // That`s frequency of publishing (Hz) not time of publishing  10 means 1/10 = 0.1 seconds

    while(ros::ok())
    {
        PID_Right_Calc();
        
        geometry_msgs::Point point;
        
        point.x=error_check_sign;
        
        point.z=abs(saturation_check);

        pub.publish(point);
           
        ros::spinOnce();

        loop.sleep();
    }

    return 0;
}

/////////////////////////////////////////////// Just Laws ///////////////////////////////////////////////

//   Convert encoder ticks to RPM
//    RPM = ( encoder_ticks *60.0 ) / ( encoder_ticks_per_revolution );

//   Convert RPMs to angular speed
//     angular_speed = ( RPM * 2 * pi)/( 60 );

//   Convert angular speed to linear speed
//     linear_speed = ( radius * angular_speed );
