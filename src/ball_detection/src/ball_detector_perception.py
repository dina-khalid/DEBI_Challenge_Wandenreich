#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion
import math
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import numpy as np
class BallDetector:
    def __init__(self):
        #initialize the node
        rospy.init_node('ball_detector')
        #world dimensions
        self.world_width = 420 #in cm
        self.world_height = 300 #in cm
        self.wall_height = 31 #in cm
        self.forbidden_width = 210 #in cm
        self.ball_diameter = 5.5 #in cm
        self.image_width = 480
        self.image_height = 640
        self.pixel_size = self.ball_diameter / max(self.image_width, self.image_height)
        #lidar data
        self.lidar_data = None
        rospy.Subscriber('/scan', LaserScan, self.process_lidar_data)
        #image data
        self.bridge = CvBridge()
        self.image_received = False
        self.camera_data = None 
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image,self.process_camera_data)
        self.ball_pub = rospy.Publisher('/ball_detection/ball', Image, queue_size=10)
        # Define ball detection parameters
        self.min_depth = 1.0  # minimum depth in meters
        self.max_depth = 0.5  # maximum depth in meters
        self.radius = 50  # radius of the ball in pixels
        self.cmd_vel_pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.rot=Twist()
        self.rate=rospy.Rate(1)
        self.cmd_vel_timer = rospy.Timer(rospy.Duration(1/80), self.publish_cmd_vel)
       #odom data
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.process_odometry_data)
        self.odom_data = None
        self.current_pose = None
    def process_odometry_data(self,msg):
        self.odom_data = msg
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2])
    def publish_cmd_vel(self, event):
        self.cmd_vel_pub.publish(self.rot)  # Publish at 50 Hz
            # Allow up to one second to connection
    def process_lidar_data(self, msg):
        self.lidar_data = msg
    def process_camera_data(self, msg):
        self.camera_data = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    def clamp(self,ang, min = -math.pi, max = math.pi):
        if ang > max:
            return ang - (max - min)
        if ang < min:
            return ang + (max) - (min)
        else:
            return ang
    def find_ball_coordinates(self):
        circle_world_x = 0
        circle_world_y = 0
        #Convert LiDAR data to 2D depth map
        image = self.camera_data
        # depth_map = np.array(self.lidar_data).reshape((-1, 1))
        # # Segment depth map to isolate area of interest
        # depth_map[depth_map > self.max_depth] = 0
        # depth_map[depth_map < self.min_depth] = 0
        # depth_mask = np.tile(depth_map, (1, 3))
        # depth_mask_resized = cv2.resize(depth_mask, (image.shape[1], image.shape[0]))
        # # Create 3-channel binary mask by duplicating single channel mask
        # depth_mask_binary_3ch = np.repeat(depth_mask_resized[..., np.newaxis], 3, axis=2)
        # depth_masked_image = cv2.bitwise_and(np.array(image, dtype = np.uint8), np.array(depth_mask_binary_3ch,  dtype = np.uint8))       
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 7, 2) 
        circles = cv2.HoughCircles(thresh, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=self.radius-35, maxRadius=self.radius+30)
        image_center_x = float(image.shape[1]) / 2
        image_center_y = float(image.shape[0]) / 2
        # Draw circles around the detected balls
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                cv2.circle(image, (x, y), r, (0,255, 0), 2)
                for circle_index in range(len(circles)):
                    # Index of the circle you want to extract
                    x, y, r = circles[circle_index]  # Extract the (x, y) coordinates and radius of the circle
                    circle_robot_x = (x - image_center_x) * self.pixel_size
                    circle_robot_y = (y - image_center_y) * self.pixel_size
                    # Define the transformation between the robot's coordinate frame and the world coordinate frame
                    robot_x = self.odom_data.pose.pose.position.x
                    robot_y = self.odom_data.pose.pose.position.y
                    robot_yaw = math.atan2(2 * (self.odom_data.pose.pose.orientation.w * self.odom_data.pose.pose.orientation.z +
                                                self.odom_data.pose.pose.orientation.x * self.odom_data.pose.pose.orientation.y),
                                            1 - 2 * (self.odom_data.pose.pose.orientation.y ** 2 + self.odom_data.pose.pose.orientation.z ** 2))
                    world_x = 0
                    world_y = 0
                    world_yaw = math.pi / 2
                    robot_world_x = robot_x * math.cos(world_yaw) + robot_y * math.sin(world_yaw) - world_x * math.cos(world_yaw) - world_y * math.sin(world_yaw)
                    robot_world_y = -robot_x * math.sin(world_yaw) + robot_y * math.cos(world_yaw) + world_x * math.sin(world_yaw) - world_y * math.cos(world_yaw)
                    # Convert the range and bearing to the circle from the LIDAR readings to Cartesian coordinates
                    angle_increment = self.lidar_data.angle_increment
                    angle_min = self.lidar_data.angle_min
                    ranges = self.lidar_data.ranges
                    # num_readings = len(ranges)
                    circle_angle = math.atan2(circle_robot_y, circle_robot_x)
                    circle_distance = math.sqrt(circle_robot_x ** 2 + circle_robot_y ** 2)
                    angle_index = int((circle_angle - angle_min) / angle_increment)
                    range_value = ranges[angle_index]
                    if range_value < circle_distance:
                        circle_world_x = robot_world_x + range_value * math.cos(circle_angle + robot_yaw)
                        circle_world_y = robot_world_y + range_value * math.sin(circle_angle + robot_yaw)
                        print(f"Circle found at ({circle_world_x}, {circle_world_y})")
                    else:
                        print("Circle not found")
            else:
                print("No circles detected")
        cv2.imshow('image', image)
        cv2.imshow('thresh', thresh)
        cv2.waitKey(1)
        return(circle_world_x, circle_world_y)
    def navigate_to_goal(self, tolerance=0.1):
        x, y = self.find_ball_coordinates()
        if (x == 0) and (y == 0):
            # No circles detected, perform search behavior rotate
            self.rot.angular.z = 0.4
            self.rot.linear.x = 0
        # Wait for the current pose to be initialized
        else :
            # while self.current_pose is None:
            #     rospy.sleep(0.01)
            # Start moving towards the goal position
            cmd_vel = Twist()
            rate = rospy.Rate(100) 
            # x, y = self.send_ball_coordinates()
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
                first_clamp = self.clamp(angle_to_goal) 
                second_clamp =  self.clamp(current_yaw)
                final_calmp = first_clamp - second_clamp
                angle_error = self.clamp(final_calmp)
                # Set the angular velocity proportional to the angle error
                if np.abs(angle_error)>np.pi/12:
                    cmd_vel.angular.z = 0.65 * angle_error
                    # Set the linear velocity proportional to the distance to the goal
                    cmd_vel.linear.x = 0
                    self.cmd_vel_pub.publish(cmd_vel)
                    # rate.sleep()
                    continue
                cmd_vel.angular.z = 0.3 * angle_error
                # Set the linear velocity proportional to the distance to the goal
                cmd_vel.linear.x = 0.35 * (distance+1)
                # print(f"distance = ${distance}")
                # print(f"angle = ${angle_error}")
                # Publish the velocity command
                self.cmd_vel_pub.publish(cmd_vel)
                # Sleep for a short duration to control the rate at which the loop runs
                # rate.sleep()
            # Stop the robot when we have reached the goal position
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
    # def move_to_ball(self):
    #     circles = self.find_ball()
    # # Check if any circles were detected
    #     if circles is None:
    #         # No circles detected, perform search behavior
    #         text = "searching"
    #         self.rot.angular.z = 0.4
    #         self.rot.linear.x = 0
    #     else:
    #         # Move towards closest circle   
    #         text = ""
    #         total_circles = len(circles[0])
    #         print(total_circles)
    #         closest_circle = None 
    #         closest_dist = float('inf')
    #         for i, circle in enumerate(circles[0]):
    #             # Compute distance to circle
    #             distance = math.sqrt((circle[0] - 320)**2 + (circle[1] - 240)**2)
    #             if distance < closest_dist:
    #                 closest_dist = distance
    #                 closest_circle = circle
    #         #Navigate to closest circle      
    #         if closest_circle is not None:
    #             for i, circle in enumerate(self.circles[0]):
    #                 # Compute the distance and angle to the center of the circle
    #                 cx, cy = int(circle[0]), int(circle[1])
    #                 obj_x = cx - 320
    #                 obj_y = cy - 240
    #                 distance = math.sqrt(obj_x**2 + obj_y**2)
    #                 angle = math.atan2(obj_y, obj_x)
    #                 # Adjust the movement behavior based on the distance and angle to the circle
    #                 if distance <= 30:
    #                     # Move straight towards the circle
    #                     text += f"Circle {i+1}: straight "
    #                     self.rot.angular.z = 0
    #                     self.rot.linear.x = 0.2 / total_circles
    #                 else:
    #                     # Turn towards the circle and move forward
    #                     text += f"Circle {i+1}: turn "
    #                     self.rot.angular.z = 0.5 * angle
    #                     self.rot.linear.x = 0.2 / total_circles
    #         # Print the movement behavior to the console
    #             print(f"Moving: {text}")
    #             # Publish the movement command
    #             self.cmd_vel_pub.publish(self.rot)
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.lidar_data is not None and self.camera_data is not None:
                self.navigate_to_goal()   
if __name__ == '__main__':
    node = BallDetector()
    node.run() 