#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist


error_pose = None
cmd_vel = None

def cmd_vel_callback(msg):
    global cmd_vel
    cmd_vel = msg

def error_callback(msg):
    global error_pose
    error_pose = msg


def plot_position_error():
    # rospy.init_node('position_error_plot')

    # Create empty lists to store position error, time, linear velocity, and angular velocity
    time = []
    start_time = rospy.Time.now().to_sec()

    cte_list = []
    etheta_list = []
    linear_velocity = []
    angular_velocity = []


    while not rospy.is_shutdown():

        if error_pose is not None and cmd_vel is not None:
            # Append position error, time, linear velocity, and angular velocity to the lists
            time.append(rospy.Time.now().to_sec() - start_time)
            cte_list.append(error_pose.position.y)
            etheta_list.append(error_pose.orientation.z)
            linear_velocity.append(cmd_vel.linear.x)
            angular_velocity.append(cmd_vel.angular.z)

            # Clear the current plot
            plt.clf()

            plt.subplot(2, 2, 1)
            plt.cla()
            plt.plot(time, cte_list, color='#66AAEE',label='Cross-Track Error')
            plt.xlabel('Time')
            plt.ylabel('Cross-Track Error')
            plt.grid(True)
            plt.legend()

            plt.subplot(2, 2, 2)
            plt.cla()
            plt.plot(time, etheta_list, color='#EE66AA',label='Heading Error')
            plt.xlabel('Time')
            plt.ylabel('Heading Error')
            plt.grid(True)
            plt.legend()

            plt.subplot(2, 2, 3)
            plt.cla()
            plt.plot(time, linear_velocity, color='#66AAEE',label='Linear Velocity')
            plt.xlabel('Time')
            plt.ylabel('Linear Velocity')
            plt.grid(True)
            plt.legend()

            plt.subplot(2, 2, 4)
            plt.cla()
            plt.plot(time, angular_velocity, color='#EE66AA',label='Angular Velocity')
            plt.xlabel('Time')
            plt.ylabel('Angular Velocity')
            plt.grid(True)
            plt.legend()


            # Adjust plot layout
            plt.tight_layout()

            # Save the current plot as a vector graphics file (SVG format) with fixed width and height
            # save_path_1 = os.path.join(os.getcwd(), "/home/bag/compare_test/" + image_name)
            # plt.savefig(save_path_1, format='svg', dpi=300, bbox_inches='tight')

            # Update the plot
            plt.pause(0.1)

        rospy.sleep(0.1)




if __name__ == '__main__':
    rospy.init_node('draw_error_node')

    _error_topic = rospy.get_param('~error_topic', '/error_topic')
    _cmd_topic = rospy.get_param('~cmd_topic', '/cmd_vel')
    
    rospy.Subscriber(_cmd_topic, Twist, cmd_vel_callback)
    rospy.Subscriber(_error_topic, Pose, error_callback)


    # Set the figure size and position
    fig = plt.figure(figsize=(8, 8))
    fig.canvas.set_window_title('Position Error Plot')
    # fig.subplots_adjust(left=0.4, bottom=0.0, right=1.0, top=0.9, wspace=0.4, hspace=0.4)

    # Plot position error, linear velocity, and angular velocity
    plt.ion()
    plot_position_error()
    
    plt.ioff()

    plt.figure()
    plt.show()
