#include <ros/ros.h>
#include <std_msgs/Int64.h>

ros::Time startTime;

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "time_publisher");
    ros::NodeHandle nh;

    // Publisher for the elapsed time in microseconds
    ros::Publisher time_pub = nh.advertise<std_msgs::Int64>("elapsed_time", 10);

    // startTime = ros::Time::now(); // Store the start time

    ros::Rate loop_rate(100); // Publish at 10 Hz

    while (ros::ok()) 
    {
        std_msgs::Int64 time_msg;
        ros::Time currentTime = ros::Time::now();
        ros::Duration elapsedTime = currentTime - startTime; // Calculate the elapsed time

        time_msg.data = elapsedTime.toNSec() / 1000000; // Convert nanoseconds to microseconds
        ROS_INFO_STREAM("delta time = " << time_msg.data);
        time_pub.publish(time_msg);

        startTime  = currentTime;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
