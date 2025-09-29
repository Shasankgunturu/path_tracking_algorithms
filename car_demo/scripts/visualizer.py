#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
import os

global x_points, y_points
x_points=np.array([])
y_points=np.array([])
expanded_path = os.path.expanduser('/home/shasankgunturu/tracking_ws/src/car_demo/waypoints/waypoints1.txt')
waypoints = []
with open(expanded_path, 'r') as file:
    lines = file.readlines()
    for line in lines:
        parts = line.strip().split(",")
        x = float(parts[0])
        y = float(parts[1])
        waypoints.append((x,y))
waypoints.append((9999,9999))
waypoints = np.array(waypoints)
plot_waypoints = waypoints[:-1]

def __callback__(msg):
    global x_points, y_points
    x_points=np.append(x_points,msg.pose.pose.position.x)
    y_points=np.append(y_points,msg.pose.pose.position.y)

if __name__ == "__main__":
    rospy.init_node("visualize")
    rospy.Subscriber("/base_pose_ground_truth", Odometry, __callback__)
    rospy.spin()
    plt.plot(plot_waypoints[:,0], plot_waypoints[:,1], label="Original Path")
    plt.plot(x_points, y_points, linestyle="--", label="Pure Pursuit Algorithm")
    plt.xlabel("x_coordinate")
    plt.ylabel("y_coordinate")
    plt.show()
