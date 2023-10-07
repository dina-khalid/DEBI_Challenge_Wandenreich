#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np



def clamp(ang, min, max):
    if ang > max:
        return ang - (max - min)
    if ang < min:
        return ang + (max) - (min)
    else:
        return ang


class NavigationNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('navigation_node')

        # Subscribe to the /odom topic to get the robot's position and orientation
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Initialize the current pose to None
        self.current_pose = None

        # Set up the velocity publisher
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Define a list of goal positions to navigate to
        self.goals = [(0.3, 1),(0.3,-0.0),(0.3,1.0),(0.0, 1),(0.0,-0.0),(0, 1),(-0.25, 1),(-0.25, -0.0)]

    def odom_callback(self, msg):
        # Update the current pose with the robot's position and orientation
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2])
    def navigate_to_goal(self, x, y, tolerance=0.1):
        # Wait for the current pose to be initialized
        while self.current_pose is None:
            rospy.sleep(0.1)

        # Start moving towards the goal position
        cmd_vel = Twist()
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            # Get the current position and orientation of the robot
            current_x, current_y, current_yaw = self.current_pose
            # Calculate the distance to the goal position
            distance = math.sqrt((x - current_x)**2 + (y - current_y)**2)
            # Check if we have reached the goal position
            if distance < tolerance:
                break
            # Calculate the desired angle to the goal position
            angle_to_goal = math.atan2(y - current_y, x - current_x)
            # Calculate the difference between the desired angle and the robot's current orientation
            angle_error = clamp(clamp(angle_to_goal, -math.pi, math.pi) -
                        clamp(current_yaw, -math.pi, math.pi), -math.pi, math.pi)
            # Set the angular velocity proportional to the angle error
            if np.abs(angle_error)>np.pi/12:
                cmd_vel.angular.z = 0.65 * angle_error
                # Set the linear velocity proportional to the distance to the goal
                cmd_vel.linear.x = 0
                self.velocity_publisher.publish(cmd_vel)
                rate.sleep()
                continue
            cmd_vel.angular.z = 0.3 * angle_error
            # Set the linear velocity proportional to the distance to the goal
            cmd_vel.linear.x = 0.35 * (distance+1)
            print(f"distance = ${distance}")
            print(f"angle = ${angle_error}")
            # Publish the velocity command
            self.velocity_publisher.publish(cmd_vel)
            # Sleep for a short duration to control the rate at which the loop runs
            rate.sleep()

        # Stop the robot when we have reached the goal position
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.velocity_publisher.publish(cmd_vel)

  

    def navigate_to_goals(self):
        # Navigate to each goal in turn
        for goal in self.goals:
            x, y = goal
            self.navigate_to_goal(x, y)

if __name__ == '__main__':
    # Create an instance of the NavigationNode class
    node = NavigationNode()

    # Navigate to each goal in turn
    node.navigate_to_goals()

    # Spin the node to prevent it from exiting
    rospy.spin()