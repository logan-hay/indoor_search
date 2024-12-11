import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import os
import time

# Hardware Constants
LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05

# Index Constants
LEFT_SIDE_INDEX = 90
LEFT_FRONT_INDEX = 135
FRONT_INDEX = 180
RIGHT_FRONT_INDEX = 225
RIGHT_SIDE_INDEX = 270
RIGHT_BACK_INDEX = 315



# Reverse Check Constants
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
COLLISION_WIDTH = 0.18#0.2

# Collision Check Constants
LIDAR_AVOID_DISTANCE = 0.6#0.77

# Wall Follow Constants
WALL_FOLLOW_DISTANCE = 0.3
ANGLE_TOLERENCE = 0.01
DISTANCE_TOLERANCE = 0.075
SCAN_WIDTH = 10
STRAIGHT_AGGRESSION = 1

# Wall Check Constants
WALL_SCAN_WIDTH = 5
WALL_CHECK_WIDTH = 0.8
WALL_CHECK_DISTANCE = (WALL_FOLLOW_DISTANCE + DISTANCE_TOLERANCE) * 1.25

# Space Check Constants
SPACE_CHECK_DEPTH = (WALL_FOLLOW_DISTANCE + DISTANCE_TOLERANCE) * 1.05
SPACE_DEAD_ZONE = 0.10
SAFE_TURN_DISTANCE = 0.25#0.30
OPENING_CHECK_OFFSET = 29


class RandomWalk(Node):

    def __init__(self):
        # Initialize the publisher
        super().__init__('random_walk_node')

        self.scan_cleaned = []
        self.stall = False
        self.turtlebot_moving = False
        self.turtlebot_turning = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10))
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10))
        
        self.laser_forward = 0
        self.odom_data = 0
        timer_period = 0.5
        self.pose_saved=''
        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def listener_callback1(self, msg1):
        scan = msg1.ranges
        self.scan_cleaned = [] 
        for reading in scan:
            if reading == float('Inf'):
                self.scan_cleaned.append(3.5)
            elif math.isnan(reading):
                self.scan_cleaned.append(0.0)
            else:
            	self.scan_cleaned.append(reading)

    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        self.pose_saved=position


    
    def get_min_cos(self, min, max, zero):
        min_distance = 3.5
        for i in range(min, max):
            current = math.cos(math.radians(abs(i - zero))) * self.scan_cleaned[i]   
            if current < min_distance:
                min_distance = current
        return min_distance
    
    def opening_check(self):
        for i in range(FRONT_INDEX, RIGHT_SIDE_INDEX - OPENING_CHECK_OFFSET):
            if math.cos(math.radians(abs(RIGHT_SIDE_INDEX - i))) * self.scan_cleaned[i] < SPACE_CHECK_DEPTH:    
                if math.sin(math.radians(abs(RIGHT_SIDE_INDEX - i))) * self.scan_cleaned[i] < SAFE_TURN_DISTANCE and math.sin(math.radians(abs(RIGHT_SIDE_INDEX - i))) * self.scan_cleaned[i] > SPACE_DEAD_ZONE:
                    return False
        return True
    
    def wall_check(self):
        for i in range(RIGHT_FRONT_INDEX, RIGHT_BACK_INDEX):
            if math.sin(math.radians(abs(RIGHT_SIDE_INDEX - i))) * self.scan_cleaned[i] < WALL_CHECK_WIDTH:
                if math.cos(math.radians(abs(RIGHT_SIDE_INDEX - i))) * self.scan_cleaned[i] < WALL_CHECK_DISTANCE:                
                    return True
        return False
    
    def collision_check(self):
        for i in range(LEFT_FRONT_INDEX, RIGHT_SIDE_INDEX):
            if math.sin(math.radians(abs(FRONT_INDEX - i))) * self.scan_cleaned[i] < COLLISION_WIDTH:
                if self.scan_cleaned[i] < LIDAR_AVOID_DISTANCE:
                    return True
        return False
    
    def reverse_check(self):
        for i in range(LEFT_SIDE_INDEX, RIGHT_SIDE_INDEX):
            if math.sin(math.radians(abs(FRONT_INDEX - i))) * self.scan_cleaned[i] < COLLISION_WIDTH:
                if self.scan_cleaned[i] < SAFE_STOP_DISTANCE:
                    return True
        return False
        
    def timer_callback(self):
        if (len(self.scan_cleaned)==0):
    	    self.turtlebot_moving = False
    	    return
        
        if self.reverse_check(): # Reverse     
                self.cmd.linear.x = -0.2
                self.cmd.angular.z = 0.2
                self.publisher_.publish(self.cmd)
                self.turtlebot_turning = False
                self.get_logger().info('Reversing')
        elif self.collision_check() and not self.turtlebot_turning: # Avoid Collision (Turn Left) 
            self.cmd.angular.z = 0.35                    
            self.cmd.linear.x = 0.06 
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
            self.get_logger().info('Avoiding Collision (Turning Left)')
        elif self.wall_check(): 
            if self.opening_check(): # Enter Opening (Turn Right)
                self.cmd.angular.z = -0.3                                   
                self.cmd.linear.x = 0.105#0.125
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = True
                self.turtlebot_turning = True
                self.get_logger().info('Enter Opening (Turning Right)')
            else:       # Follow Wall (Move Foward)                                  
                from_wall = self.get_min_cos(RIGHT_SIDE_INDEX - SCAN_WIDTH, RIGHT_SIDE_INDEX + SCAN_WIDTH, RIGHT_SIDE_INDEX)
                if(from_wall < WALL_FOLLOW_DISTANCE - DISTANCE_TOLERANCE):
                    self.cmd.angular.z = 0.15               
                    self.cmd.linear.x = 0.23
                    self.get_logger().info('Following Wall (Foward - Hard Left)')
                elif(from_wall > WALL_FOLLOW_DISTANCE + DISTANCE_TOLERANCE):
                    self.cmd.angular.z = -0.15
                    self.cmd.linear.x = 0.23
                    self.get_logger().info('Following Wall (Foward - Hard Right)')
                else:
                    front_distance = self.get_min_cos(RIGHT_FRONT_INDEX - SCAN_WIDTH, RIGHT_FRONT_INDEX + SCAN_WIDTH, RIGHT_SIDE_INDEX)                
                    back_distance = self.get_min_cos(RIGHT_BACK_INDEX - SCAN_WIDTH, RIGHT_BACK_INDEX + SCAN_WIDTH, RIGHT_SIDE_INDEX)
                    difference = (1 - (front_distance/back_distance)) * STRAIGHT_AGGRESSION
                    if difference < -0.275:
                        difference = -0.275
                    elif difference > 0.275:
                        difference = 0.275
                    self.cmd.angular.z = float(difference)
                    self.cmd.linear.x = 0.22
                    self.get_logger().info(f"Following Wall ({difference})")
                    
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = True
                self.turtlebot_turning = False
        else:
            self.cmd.linear.x = 0.3
            self.cmd.linear.z = 0.0
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
            self.turtlebot_turning = False
            self.get_logger().info('Finding Wall (Foward)')

def main(args=None):
    rclpy.init(args=args)
    random_walk_node = RandomWalk()
    
    try:
        rclpy.spin(random_walk_node)
    except KeyboardInterrupt:
        random_walk_node.get_logger().info('Terminating trial...')
    random_walk_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
