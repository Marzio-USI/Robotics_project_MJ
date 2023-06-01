import rclpy
from rclpy.node import Node
import tf_transformations
import numpy as np
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import time
import sys
from sensor_msgs.msg import Range
import math
from math import sqrt

dir_path1 = '/home/robotics23/dev_ws/src/assignment2'
dir_path2 = '/home/robotics23/dev_ws/src/assignment2/assignment2'
sys.path.append(dir_path1)
sys.path.append(dir_path2)
from maze_spawner import spawn_maze, get_thymio_position, get_thymio_orientation
import os

from PathPlanning import *

FACING_UP = 1.56
FACING_DOWN = -1.56
FACING_LEFT = -3.13
FACING_RIGHT = 0.0
COMPLETE_ROTATION= 2* math.pi
HALF_ROTATION = math.pi

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameter('algo_name', 'default_value')

        algo_name = self.get_parameter('algo_name').get_parameter_value().string_value


        nodes, nodes_coords, edges, start_point, end_point, node_start, node_end = spawn_maze(size=54)


        self.algo = get_algorithm(algo_name, nodes, edges, node_start, node_end)
        self.nodes_coords = nodes_coords
        self.nodes = nodes

        self.path = self.algo.compute()
        self.prev_node = None
        self.next_node = self.path.pop(0)
        
        

        # realtive_node_coords = self.global_to_local(start_point, nodes_coords)

        self.start_point = start_point
        self.end_point = end_point

        
    
                
        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)


        self.proximity_center_subscriber = self.create_subscription(Range, 'proximity/center', self.proximity_callback, 10)
        self.proximity_left_subscriber = self.create_subscription(Range, 'proximity/left', self.proximity_callback_left, 10)
        self.proximity_right_subscriber = self.create_subscription(Range, 'proximity/right', self.proximity_callback_right, 10)


        # self.proximity_rear_left_subscriber = self.create_subscription(Range, 'proximity/rear_left',self.proximity_callback_back, 10)
        # self.proximity_rear_right_subscriber = self.create_subscription(Range, 'proximity/rear_right', self.proximity_callback_back, 10)
        # NOTE: we're using relative names to specify the topics (i.e., without a 
        # leading /). ROS resolves relative names by concatenating them with the 
        # namespace in which this node has been started, thus allowing us to 
        # specify which Thymio should be controlled.
        self.info_stop = -1.0
        self.info_stop_left = -1.0
        self.info_stop_right = -1.0
  

    def coords_to_pose2d(self, coords):
        x : float = coords[0]
        y : float= coords[1]
        return (
            x, y, 0.0
        )
    
    def linear_vel(self, angle):
        angle = abs(angle)
        return (1.0 / ((angle+1)**2 ))*0.3

  

    def rotate(self, orientation, to_orientation):
        z:float = orientation[-1]
        difference = (to_orientation - z + HALF_ROTATION) % COMPLETE_ROTATION - HALF_ROTATION
        return difference + COMPLETE_ROTATION if difference <= -HALF_ROTATION else difference


    def update_callback(self):
        thymio_position = self.pose3d_to_2d(get_thymio_position())
        thymio_orientation = get_thymio_orientation()
        if self.euclidean_distance(thymio_position, self.coords_to_pose2d(self.nodes_coords[self.next_node])) < 0.20:
            if len(self.path) == 0:
                raise KeyboardInterrupt()
            self.prev_node = self.next_node
            self.next_node = self.path.pop(0)
        
        prev_node_x, prev_node_y , _ = self.coords_to_pose2d(self.nodes_coords[self.prev_node])
        next_node_x, next_node_y , _ = self.coords_to_pose2d(self.nodes_coords[self.next_node])
        # up x is the same, y is higher
        cmd_vel = Twist() 
        if (prev_node_x == next_node_x) and (prev_node_y < next_node_y):
            angle = self.rotate(thymio_orientation, FACING_UP) 
        elif (prev_node_x == next_node_x) and (prev_node_y > next_node_y):
            # down (x is the same, y is lower)
            angle = self.rotate(thymio_orientation, FACING_DOWN)

        elif (prev_node_x < next_node_x) and (prev_node_y == next_node_y):
            # right (x is higher for next), y is the same
            angle = self.rotate(thymio_orientation, FACING_RIGHT)
        else:
            angle = self.rotate(thymio_orientation, FACING_LEFT)
        cmd_vel.angular.z = angle
        cmd_vel.linear.x = self.linear_vel(angle)
         if self.info_stop_left > 0 or self.info_stop_right > 0:
            cmd_vel.linear.x *= 0.5
            
        if abs(angle) < 0.01:
            diff = abs(self.info_stop_left - self.info_stop_right)
            # correction of pose
            if diff > 0.01:
                if (self.info_stop_left > self.info_stop_right):
                    cmd_vel.angular.z += ((math.pi) / 6)
                else:
                    cmd_vel.angular.z -= ((math.pi) / 6)

        self.vel_publisher.publish(cmd_vel)

         

    def proximity_callback(self, message):
        distance  = message.range
        if distance >= 0 and distance < 0.2:
            self.info_stop = distance
            # self.get_logger().info('detrecting front')
        else:
            self.info_stop = -1.0

    def proximity_callback_left(self, message):
        distance  = message.range
        if distance >= 0 and distance < 0.3:
            self.info_stop_left = distance
            # self.get_logger().info('detrecting left')
        else:
            self.info_stop_left = -1.0

    def proximity_callback_right(self, message):
        distance  = message.range
        if distance >= 0 and distance < 0.3:
            self.info_stop_right = distance
            # self.get_logger().info('detrecting right')
        else:
            self.info_stop_right = -1.0

        
    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1/60, self.update_callback)
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
  
    
    def pose3d_to_2d(self, position):
        pose2 = (
            position[0],  # x position
            position[1],  # y position
            # yaw                # theta orientation
        )
        
        return pose2
    
    def euclidean_distance(self, goal_pose, current_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose[0] - current_pose[0]), 2) +
                    pow((goal_pose[1] - current_pose[1]), 2))
    


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = ControllerNode()
    node.start()
    
    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Ensure the Thymio is stopped before exiting
    node.stop()


if __name__ == '__main__':
    main()