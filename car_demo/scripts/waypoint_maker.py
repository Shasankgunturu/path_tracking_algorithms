#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

def odom_callback(msg):
    global waypoint_file
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    waypoint_file.write(f"{x}, {y}\n")

def listener():
    global waypoint_file
    rospy.init_node('waypoint_logger', anonymous=True)
    rospy.Subscriber("/base_pose_ground_truth", Odometry, odom_callback)
    waypoint_file = open("/home/shasankgunturu/tracking_ws/src/car_demo/waypoints/waypoints2.txt", "w")
    rospy.spin()
    waypoint_file.close()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
