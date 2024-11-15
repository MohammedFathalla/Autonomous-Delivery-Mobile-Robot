#include <ros/ros.h>
#include <std_msgs/Int32.h>

void callback(const std_msgs::Int32::ConstPtr& msg) {
  ROS_INFO_STREAM("Received Room number: " << msg->data);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_subscriber");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/room_number", 1000, callback);

  ros::spin();  // Keeps the node running and processing callbacks

  return 0;
}
