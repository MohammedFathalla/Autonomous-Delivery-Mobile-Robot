#include <ros/ros.h>
#include <std_msgs/Int32.h> // Import the message type used for room_number

// Callback function to handle incoming messages
void roomNumberCallback(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("Received room number: %d", msg->data);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "room_number_subscriber"); // Initialize the ROS node
    ros::NodeHandle nh; // Create a NodeHandle

    // Create a subscriber to the "room_number" topic
    ros::Subscriber sub = nh.subscribe("room_number", 1000, roomNumberCallback);

    ros::spin(); // Enter a loop to process callbacks
    return 0;
}
