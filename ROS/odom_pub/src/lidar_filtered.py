## This node is dedicated for Filtering lidar scan by computing only 180 degress from lidar scan 

#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def filter_scan(data):
    filtered_data = LaserScan()
    filtered_data.header = data.header
    filtered_data.angle_min = -1.5  # -90 degrees  was -1.57
    filtered_data.angle_max = 1.5   # 90 degrees   was 1.57
    filtered_data.angle_increment = data.angle_increment
    filtered_data.time_increment = data.time_increment
    filtered_data.scan_time = data.scan_time
    filtered_data.range_min = data.range_min
    filtered_data.range_max = data.range_max
    
    # Calculate indices for the 180 degree range
    min_index = int((filtered_data.angle_min - data.angle_min) / data.angle_increment)
    max_index = int((filtered_data.angle_max - data.angle_min) / data.angle_increment)
    
    filtered_data.ranges = data.ranges[min_index:max_index]
    filtered_data.intensities = data.intensities[min_index:max_index]
    
    pub.publish(filtered_data)

rospy.init_node('laser_filter')
sub = rospy.Subscriber('/scan', LaserScan, filter_scan)
pub = rospy.Publisher('/filtered_scan', LaserScan, queue_size=10)

rospy.spin()
