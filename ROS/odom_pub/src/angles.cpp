/*
  This node to aims to convert quaternion values to roll pitch yaw values to make sure of the yaw angle limits
*/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

/*

// Publisher topic is /tf

Message type : tf2_msgs/TFMessage

*/

void callback(const tf2_msgs::TFMessage::ConstPtr& msg) {
    // Loop through all transforms in the TF message
    for (const auto& transform : msg->transforms) {
        // Check the frame you are interested in
        if (transform.header.frame_id == "odom" && transform.child_frame_id == "base_link") {
            // Extract quaternion from the transform
            geometry_msgs::Quaternion q = transform.transform.rotation;
            
            // Convert quaternion to tf::Quaternion
            tf::Quaternion quaternion(q.x, q.y, q.z, q.w);

            // Convert quaternion to yaw
            double yaw = tf::getYaw(quaternion);

            // Print the yaw value
            ROS_INFO("Yaw: %f", yaw);
        }
    }
}

int main( int argc , char **argv )
{
    ros::init(argc, argv, "angle");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/tf", 100 , &callback);
    
    ros::spin();
    return 0;
}