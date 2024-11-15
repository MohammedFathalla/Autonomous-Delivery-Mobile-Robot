/*
    This node is dedicated for Receiving room number and send it`s location to move base ( Way Points )
*/

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int32.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Subscriber sub;
ros::Publisher goal_reach_pub;
MoveBaseClient* ac;
move_base_msgs::MoveBaseGoal goal;
int last_room_number = -1;

void callback(const std_msgs::Int32::ConstPtr& msg) {
    int current_room_number = msg->data;
    if (current_room_number != last_room_number) {
        ROS_INFO_STREAM("Received Room number: " << current_room_number);
        last_room_number = current_room_number;
        
        // Set goal based on received room number
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        if (current_room_number == 1) {

            goal.target_pose.pose.position.x = -2.474172353744507;
            goal.target_pose.pose.position.y = -81.7481689453125;
            goal.target_pose.pose.orientation.z = -0.8320657014846802;
            goal.target_pose.pose.orientation.w = 0.5209760936971669;

            ROS_INFO("Setting goal for room 1");
        } else if (current_room_number == 4) {
            goal.target_pose.pose.position.x = 0.0;
            goal.target_pose.pose.position.y =  0.0;
            goal.target_pose.pose.orientation.z = 0.4981655884133998;
            goal.target_pose.pose.orientation.w = 0.8670819145390654;
            ROS_INFO("Setting goal for room 2");
        } else {
            ROS_WARN("Received invalid room number: %d", current_room_number);
            return;
        }

        // Send goal to move_base
        ac->sendGoal(goal);
        ac->waitForResult();

        if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Successfully reached the goal for room %d", current_room_number);
            
            // Publish to goal_reach topic
            std_msgs::Int32 msg;
            msg.data = 1; // Indicates goal reached
            goal_reach_pub.publish(msg);
            ROS_INFO("Published goal reached message.");
        } else {
            ROS_ERROR("Failed to reach the goal for room %d", current_room_number);
        }
    } else {
        ROS_INFO("Received the same room number: %d (No action taken)", current_room_number);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "room_navigation_node");
    ros::NodeHandle nh;

    // Initialize the MoveBaseClient
    ac = new MoveBaseClient("move_base", true);

    // Wait for the action server to come up
    ROS_INFO("Waiting for move_base action server to come up...");
    ac->waitForServer(ros::Duration(5.0));
    ROS_INFO("Connected to move_base action server.");

    // Subscribe to the /room_number topic
    sub = nh.subscribe("/room", 1000, callback);

    // Advertise the goal_reach topic
    goal_reach_pub = nh.advertise<std_msgs::Int32>("goal_reach", 1000);

    ros::spin(); // Keeps the node running and processing callbacks

    // Clean up
    delete ac;

    return 0;
}
