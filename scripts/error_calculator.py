#!/usr/bin/env python3

from error_msgs.msg import Error


import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import Float64
from math import atan2, asin, sin, cos, pi, copysign

def move_base_goal_callback(msg):
    """
    Save the latest goal position and orientation.

    Args:
        msg (PoseStamped): The message containing the goal position and orientation.
    """
    global goal_pos, goal_orient
    goal_pos = msg.pose.position
    goal_orient = msg.pose.orientation

def move_base_result_callback(msg):
    global goal_pos, goal_orient, is_goal_reached
    """
    Check if the robot has reached its goal successfully.

    Args:
        msg (MoveBaseActionResult): The message containing the status of the move_base action.
    """
    if goal_pos is not None and msg.status.status == 3:
        is_goal_reached = True
    else:
        is_goal_reached = False

def odom_callback(msg):
    global goal_pos, goal_orient, is_goal_reached
    """
    Calculate the error in x, y, and z directions between the goal and actual position.
    Calculate the error in roll, pitch, and yaw angles between the goal and actual orientation.
    Publish the errors on the /error_calculator topic.

    Args:
        msg (Odometry): The message containing the current position and orientation of the robot.
    """
    if is_goal_reached:
        error_x = msg.pose.pose.position.x - goal_pos.x
        error_y = msg.pose.pose.position.y - goal_pos.y
        error_z = msg.pose.pose.position.z - goal_pos.z

        # Calculate the error in roll, pitch, and yaw angles between the goal and actual orientation
        error_roll, error_pitch, error_yaw = quaternion_to_euler(msg.pose.pose.orientation)
        goal_roll, goal_pitch, goal_yaw = quaternion_to_euler(goal_orient)

        error_roll = normalize_angle(error_roll - goal_roll)
        error_pitch = normalize_angle(error_pitch - goal_pitch)
        error_yaw = normalize_angle(error_yaw - goal_yaw)

        # Publish the errors on the /error_calculator topic
        error_msg = Error()
        error_msg.error_x = error_x
        error_msg.error_y = error_y
        error_msg.error_z = error_z
        error_msg.error_roll = error_roll
        error_msg.error_pitch = error_pitch
        error_msg.error_yaw = error_yaw

        error_pub.publish(error_msg)



        # Reset the goal position, orientation, and goal reached flag
        goal_pos = None
        goal_orient = None
        is_goal_reached = False

def quaternion_to_euler(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = copysign(pi / 2, sinp)
    else:
        pitch = asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def normalize_angle(angle):
    return atan2(sin(angle), cos(angle))

if __name__ == '__main__':
    # Initialize the node and subscribers
    rospy.init_node('error')
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, move_base_goal_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, move_base_result_callback)


    # Initialize the goal position, orientation, and goal reached flag
    goal_pos = None
    goal_orient = None
    is_goal_reached = False

    # Initialize the publishers for the error values
    error_pub = rospy.Publisher('/error', Error, queue_size=10)

    # Set the loop rate
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()

