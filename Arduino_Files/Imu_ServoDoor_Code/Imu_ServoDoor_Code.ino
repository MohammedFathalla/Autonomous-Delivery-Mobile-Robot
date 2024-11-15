/*
 * This Code is for 
 * 
 *  1- Reading and sending imu values to imu topic for odometry and tf calculation
 *     
 *  2 - Controlling Servo motors that opens and closes the Door mechanism
 * 
 */

// Right motor pin 8 
// left motor pin 9

#include <ros.h>
//#include <ReefwingMPU6050.h>
#include <geometry_msgs/Point.h>
#include <ServoEasing.hpp> // Use ServoEasing for smoother control
#include <std_msgs/Int32.h>
#include <MPU6050_tockn.h>
#include <Servo.h>

const double pi = 3.14159265358;   // Pi constant

MPU6050 mpu6050(Wire);

// Create two Servo objects for servo1 and servo2
ServoEasing myServo1,myServo2; 

// Servo related global variables
const int servoPin1 = 8; // Pin connected to the servo signal ( Right Servo motor )
const int servoPin2 = 9; // Pin connected to the servo signal ( Left Servo motor )
int motor_state;         // The should be state of servos ( Whether to open or close doors ) 
int targetAngle_R = 20;   // Default start for Right Servo motor
int targetAngle_L = 180; // Default start angle for Left Servo motor

// Taking object of geometry point message for receiving then publishing imu messages
geometry_msgs::Point twist1;  // IMU message

// Publisher for Publishing IMU messages
ros::Publisher pub("/imu", &twist1);

// ROS node handle for publishers/subscribers
ros::NodeHandle nh;

// Callback function for subscribing to door topic
void messageCb(const std_msgs::Int32& msg) 
{
/* 
 *  You may notice later that both motors take different angles at close state because they are placed reverse to each others 
 *   
*/
  
  motor_state = msg.data;  // receiving the should be state of door ( 0 door to be closed , 1 door to be opened )

  // Update target angle based on the motor state
  if (motor_state == 0) // Close door state
  {
    targetAngle_R = 0;  // Right motor angle to be set to angle 0 
    targetAngle_L = 180; // Left motor angle to be set to angle 180 ( if you wander why 180 not 0 look at above for the notice ) 
  } 
  else if (motor_state == 1) // Open door state
  {
    targetAngle_R = 80;   // Right motor angle to be set to angle 90
    targetAngle_L = 85; // Left motor angle to be set to angle 90
  }

  // Set the speed (degrees per second) for smooth movement
  myServo1.setSpeed(23); // Adjust the speed as needed ( right ) 
  myServo2.setSpeed(20); // Adjust the speed as needed ( left  )

  // Start easing to the new target position
  myServo1.startEaseTo(targetAngle_L);
  myServo2.startEaseTo(targetAngle_R);

}

// Subscriber for door control
ros::Subscriber<std_msgs::Int32> sub("/door", &messageCb);


void updateIMU() 
{
  // This IMU drive provides orientation ( angular position ) in degrees
  mpu6050.update();
  
  double angle=0.0;
  angle= ((mpu6050.getAngleZ()*pi)/180);  // Converting orientation value from degrees to radians 

  // Normalizing orientation to be in range from -3.14 to 3.14 ( As convention in ROS )
  while (angle > pi) angle -= 2.0 * pi;
  while (angle < -pi) angle += 2.0 * pi;
  twist1.z=angle;
  Serial.println(angle);
  pub.publish(&twist1);  // Publishing the imu value
}

void setup() 
{
  nh.initNode();
  nh.subscribe(sub);   // Door subscriber
  nh.advertise(pub);   // IMU publisher 
  Serial.begin(115200);  // 115200


  Wire.begin();
  mpu6050.begin();
  //  mpu6050.calcGyroOffsets(true);
  mpu6050.setGyroOffsets(-1.21, 0.10,0.13);   // Differ accoriding to IMU placed position in robot ( set calcGyroOffsets true then take the values from serial and place it here )
                                              // If you changed IMU place don`t forget for re-calibration

  // Initialize the servo and attach it to the pin
  myServo1.attach(servoPin1);
  myServo2.attach(servoPin2);
  
  myServo1.write(180);  // Default Start at angle 0 ( Door closed )
  myServo2.write(0);  // Default Start at angle 0   ( Door closed )
}


void loop() 
{
  myServo1.update();  // Ensure smooth movement of the servo
  nh.spinOnce();  // Process ROS messages 
  myServo2.update();  // Ensure smooth movement of the servo
  nh.spinOnce();  // Process ROS messages
  updateIMU();  // Update IMU readings
  
}
