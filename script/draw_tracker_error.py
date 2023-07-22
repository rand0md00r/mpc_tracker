#!/usr/bin/env python
# -*- coding: utf-8 -*-
#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math

def calculate_tracking_error(global_path, odom_poses):
    # Assuming both global_path and odom_poses are lists of PoseStamped messages
    if not global_path or not odom_poses:
        return []

    path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in global_path]
    odom_points = [(pose.pose.position.x, pose.pose.position.y) for pose in odom_poses]

    # Find the closest point in the path to the current position in the odometry
    closest_index = 0
    min_distance = float('inf')
    for i, (odom_x, odom_y) in enumerate(odom_points):
        for j, (path_x, path_y) in enumerate(path_points):
            distance = math.sqrt((odom_x - path_x)**2 + (odom_y - path_y)**2)
            if distance < min_distance:
                min_distance = distance
                closest_index = j

    # Calculate tracking errors (distance between odom and closest path point)
    tracking_errors = []
    for i, (odom_x, odom_y) in enumerate(odom_points):
        path_x, path_y = path_points[min(len(path_points)-1, closest_index + i)]
        tracking_errors.append(math.sqrt((odom_x - path_x)**2 + (odom_y - path_y)**2))

    return tracking_errors

def global_path_callback(data):
    global global_path
    global_path = data.poses

def odom_callback(data):
    global odom_poses
    odom_poses.append(data.pose.pose)

def path_tracking_error():
    global global_path, odom_poses

    global_path = []
    odom_poses = []

    rospy.init_node('path_tracking_error', anonymous=True)
    rospy.Subscriber('/global_path', Path, global_path_callback)
    rospy.Subscriber('/odom_gazebo', Odometry, odom_callback)

    rospy.spin()

    # Calculate tracking errors
    tracking_errors = calculate_tracking_error(global_path, odom_poses)

    # Plot the data
    plt.plot(tracking_errors)
    plt.xlabel('Time (samples)')
    plt.ylabel('Tracking Error')
    plt.title('Path Tracking Error')
    plt.show()

if __name__ == '__main__':
    path_tracking_error()
